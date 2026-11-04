"""Isolated ArduCopter SITL test. Starts its own local process; never a real FC.

This tool is separate from the production runtime and is allowed to inject
synthetic obstacle measurements only into its own localhost SITL process.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import threading

sys.path[:0]=[str(Path(__file__).resolve().parents[1]),str(Path(__file__).resolve().parent)]

os.environ['MAVLINK20']='1'
from pymavlink import mavutil

M=mavutil.mavlink


class SITL:
    def __init__(self,binary,repo,directory,enabled=True,port=16410):
        self.binary,self.repo,self.directory=Path(binary).resolve(),Path(repo).resolve(),Path(directory).resolve()
        if sys.platform!='linux' or not self.binary.is_file():
            raise ValueError('Requires a local Linux SITL ELF binary')
        if self.binary.read_bytes()[:4]!=b'\x7fELF' or '/build/sitl/' not in str(self.binary):
            raise ValueError('Expected a /build/sitl/ ELF binary; hardware firmware is not accepted')
        if (self.directory/'eeprom.bin').exists():
            raise ValueError('Use a new isolated run directory; do not reuse existing EEPROM')
        self.directory.mkdir(parents=True,exist_ok=True)
        self.port=port
        with socket.socket() as s:
            if s.connect_ex(('127.0.0.1',port))==0:
                raise ValueError('Local test port is already occupied')
        self.parameters={
            'SERIAL0_PROTOCOL':2,'MAV_SYSID':42,'MAV_GCS_SYSID':250,'PRX1_TYPE':2,'PRX_LOG_RAW':1,
            'PRX_ALT_MIN':0,'PRX_IGN_GND':0,'PRX_FILT':0,
            'AVOID_ENABLE':2 if enabled else 0,'AVOID_BEHAVE':1,'AVOID_MARGIN':1,
            'AVOID_BACKUP_SPD':0,'ARMING_CHECK':0,'FS_THR_ENABLE':0,
            'FS_GCS_ENABLE':0,'FENCE_ENABLE':0,'LOG_DISARMED':1,
            'LOIT_SPEED':100,'LOIT_ACC_MAX':100,'LOIT_BRK_ACCEL':100,
            'LOIT_BRK_DELAY':0.1,'RC_OPTIONS':0,'RC7_OPTION':0,
            'RC_OVERRIDE_TIME':3,'SIM_SPEEDUP':1,
        }
        defaults=self.directory/'sitl_test.parm'
        defaults.write_text('\n'.join(f'{k} {v}' for k,v in self.parameters.items())+'\n')
        self.command=[str(self.binary),'--model','quad','--speedup','1',
                      '--base-port','16000','--instance','41','--serial0',f'tcp:{port}',
                      '--sysid','42','--home','-35.363261,149.165230,584,0',
                      '--defaults',f'{self.repo}/Tools/autotest/default_params/copter.parm,{defaults}']
        self.stdout=(self.directory/'sitl_stdout.log').open('w')
        self.process=subprocess.Popen(self.command,cwd=self.directory,stdout=self.stdout,stderr=subprocess.STDOUT)
        self.connection=None
        self.rc=[1500,1500,1000,1500,1500,1500,1000,1500]
        self.sender=None
        self.last_hb=self.last_rc=self.last_obstacle=0
        self.state={}
        self.messages=[]
        self.statuses=[]
        self.param_values={}
        self.start=time.monotonic()
        self.log=(self.directory/'telemetry.jsonl').open('w')
        self.input_log=(self.directory/'obstacle_inputs.jsonl').open('w')
        self.obstacle_sent=0
        self.transmitted=[]
        self.geo_origin=None
        self.native_local_seen=False

    def connect(self):
        deadline=time.monotonic()+20
        while time.monotonic()<deadline:
            if self.process.poll() is not None:
                raise RuntimeError('Owned SITL process exited; inspect sitl_stdout.log')
            try:
                self.connection=mavutil.mavlink_connection(f'tcp:127.0.0.1:{self.port}',
                    source_system=250,source_component=190)
                break
            except (OSError,ConnectionError):
                time.sleep(.2)
        if self.connection is None:
            raise TimeoutError('Owned SITL TCP did not open')
        hb=self.connection.wait_heartbeat(timeout=15)
        if hb is None or hb.get_srcSystem()!=42 or hb.autopilot!=M.MAV_AUTOPILOT_ARDUPILOTMEGA:
            raise RuntimeError('Expected isolated ArduPilot sysid=42 heartbeat')
        self.state['HEARTBEAT']=hb
        self.connection.mav.param_request_list_send(42,1)
        self.connection.mav.request_data_stream_send(42,1,M.MAV_DATA_STREAM_ALL,20,1)
        for msgid,interval in ((32,50000),(33,100000),(30,50000),(132,100000),(193,200000)):
            self.command_long(M.MAV_CMD_SET_MESSAGE_INTERVAL,msgid,interval)
        self.command_long(M.MAV_CMD_REQUEST_MESSAGE,148)
        self.pump(8)
        if 'SIM_SPEEDUP' not in self.param_values or self.process.poll() is not None:
            raise RuntimeError('SITL identity parameter not verified; commands blocked')
        print(json.dumps({'event':'connected','pid':self.process.pid,
                          'sysid':42,'firmware':self.statuses[:5]}),flush=True)

    def command_long(self,command,*values):
        values=list(values)+[0]*(7-len(values))
        self.connection.mav.command_long_send(42,1,command,0,*values[:7])

    def send_obstacles(self,distances,max_cm=1000,production_encoder=False,capture_epoch_ns=None):
        if len(distances)!=72:
            raise ValueError('Expected 72 sectors')
        if production_encoder:
            from omniavoid.protocol import Snapshot, obstacle_message
            snap=Snapshot(time.monotonic_ns(),capture_epoch_ns or time.time_ns(),tuple(map(int,distances)))
            self.connection.mav.send(obstacle_message(snap,max_cm/100))
        else:
            self.connection.mav.obstacle_distance_send(time.time_ns()//1000,M.MAV_DISTANCE_SENSOR_UNKNOWN,
                list(map(int,distances)),5,0,max_cm,5.0,0.0,M.MAV_FRAME_BODY_FRD)
        self.obstacle_sent+=1
        now=time.monotonic()-self.start
        self.transmitted.append(now)
        self.input_log.write(json.dumps({'wall_s':now,'distances_cm':list(map(int,distances)),
            'production_encoder':production_encoder,'capture_epoch_ns':capture_epoch_ns})+'\n')

    def handle(self,message):
        if message.get_srcSystem()!=42:
            return
        name=message.get_type()
        self.state[name]=message
        row={'wall_s':time.monotonic()-self.start,**message.to_dict()}
        self.log.write(json.dumps(row)+'\n')
        self.messages.append(row)
        if name=='LOCAL_POSITION_NED':
            self.native_local_seen=True
        if name=='GLOBAL_POSITION_INT' and not self.native_local_seen:
            # This custom firmware acknowledges MSG32 but does not emit it.
            # Derive a short-distance tangent-plane position from received GPS/EKF data.
            if self.geo_origin is None:
                self.geo_origin=(message.lat/1e7,message.lon/1e7)
            lat0,lon0=self.geo_origin
            local=M.MAVLink_local_position_ned_message(message.time_boot_ms,
                math.radians(message.lat/1e7-lat0)*6378137,
                math.radians(message.lon/1e7-lon0)*6378137*math.cos(math.radians(lat0)),
                -message.relative_alt/1000,message.vx/100,message.vy/100,message.vz/100)
            self.state['LOCAL_POSITION_NED']=local
            derived={'wall_s':row['wall_s'],**local.to_dict(),'derived_from':'GLOBAL_POSITION_INT'}
            self.messages.append(derived)
            self.log.write(json.dumps(derived)+'\n')
        if name=='PARAM_VALUE':
            self.param_values[message.param_id]=message.param_value
        if name=='STATUSTEXT':
            self.statuses.append(message.text)
            print('FC:',message.text,flush=True)

    def pump(self,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            if self.process.poll() is not None:
                raise RuntimeError('Owned SITL process exited')
            now=time.monotonic()
            if now-self.last_hb>=1:
                self.connection.mav.heartbeat_send(M.MAV_TYPE_GCS,M.MAV_AUTOPILOT_INVALID,0,0,0)
                self.last_hb=now
            if now-self.last_rc>=.1:
                self.connection.mav.rc_channels_override_send(42,1,*self.rc)
                self.last_rc=now
            if self.sender and now-self.last_obstacle>=.05:
                payload=self.sender()
                if payload is not None:
                    self.send_obstacles(*payload)
                self.last_obstacle=now
            msg=self.connection.recv_match(blocking=True,timeout=.01)
            if msg:
                self.handle(msg)
        self.log.flush()

    def wait(self,predicate,timeout,label):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            if predicate():
                return
            self.pump(.1)
        raise TimeoutError(label+'; recent status: '+' | '.join(self.statuses[-6:]))

    def set_mode(self,mode):
        mapping=self.connection.mode_mapping()
        mode_number=mapping[mode]
        self.connection.mav.set_mode_send(42,M.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,mode_number)
        self.wait(lambda:self.state.get('HEARTBEAT') is not None and self.state['HEARTBEAT'].custom_mode==mode_number,
                  8,'Mode '+mode)

    def takeoff(self,altitude=3):
        self.wait(lambda:'LOCAL_POSITION_NED' in self.state and 'GPS_RAW_INT' in self.state and self.state['GPS_RAW_INT'].fix_type>=3,
                  35,'GPS/EKF local position')
        self.set_mode('GUIDED')
        self.rc[2]=1000
        # GPS fix precedes EKF readiness. Retry normal arming while alignment finishes.
        arm_deadline=time.monotonic()+60
        while not self.state['HEARTBEAT'].base_mode & M.MAV_MODE_FLAG_SAFETY_ARMED:
            self.command_long(M.MAV_CMD_COMPONENT_ARM_DISARM,1)
            self.pump(2)
            if time.monotonic()>arm_deadline:
                raise TimeoutError('Normal SITL arming: '+' | '.join(self.statuses[-6:]))
        self.command_long(M.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,altitude)
        self.rc[2]=1500
        self.wait(lambda:'LOCAL_POSITION_NED' in self.state and -self.state['LOCAL_POSITION_NED'].z>altitude-.25,
                  25,'SITL takeoff')
        self.set_mode('LOITER')
        self.pump(3)
        rc=self.state.get('RC_CHANNELS')
        if rc is None or rc.chan3_raw!=1500 or -self.state['LOCAL_POSITION_NED'].z<2:
            raise RuntimeError('Loiter did not hold altitude or RC override was not accepted')

    def close(self):
        if self.connection:
            self.connection.close()
        if self.process.poll() is None:
            self.process.terminate()
            try:self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill();self.process.wait(timeout=5)
        self.log.close()
        self.input_log.close()
        self.stdout.close()

    def metadata(self):
        return {'binary':str(self.binary),'binary_sha256':hashlib.sha256(self.binary.read_bytes()).hexdigest(),
                'command':self.command,'directory':str(self.directory),
                'parameter_defaults':self.parameters,'parameter_readback':{k:self.param_values.get(k) for k in self.parameters},
                'firmware_statuses':self.statuses,'obstacle_messages_sent':self.obstacle_sent,
                'publication_mean_hz':(len(self.transmitted)-1)/(self.transmitted[-1]-self.transmitted[0]) if len(self.transmitted)>1 else None,
                'autopilot_version':self.state['AUTOPILOT_VERSION'].to_dict() if 'AUTOPILOT_VERSION' in self.state else None,
                'position_source':'native LOCAL_POSITION_NED' if self.native_local_seen else 'tangent plane derived from GLOBAL_POSITION_INT; 1e-7 degree coordinate quantization',
                'safety':'Own subprocess, fixed localhost TCP, isolated EEPROM, sysid 42 verified; no real FC connection'}


def reception(test):
    cases=[]
    for label,distances in (
        ('cardinal_metres',[100 if i==0 else 200 if i==18 else 300 if i==36 else 400 if i==54 else 65535 for i in range(72)]),
        ('zero_contact',[0 if i==0 else 65535 for i in range(72)]),
        ('production_zero_contact',[0 if i==0 else 65535 for i in range(72)]),
        ('one_centimetre',[1 if i==0 else 65535 for i in range(72)]),
        ('all_unknown',[65535]*72),
        ('no_messages',None)):
        test.sender=(lambda d=distances,production=label=='production_zero_contact':(d,1000,production)) if distances is not None else None
        marker=len(test.messages)
        test.pump(2)
        messages=[m for m in test.messages[marker:] if m['mavpackettype']=='DISTANCE_SENSOR']
        result={'label':label,'distance_sensor_messages':messages[-16:],
                'latest_sys_status':test.state['SYS_STATUS'].to_dict() if 'SYS_STATUS' in test.state else None}
        cases.append(result)
        print(json.dumps({'case':label,'echoes':[(m['orientation'],m['current_distance']) for m in messages[-8:]]}),flush=True)
    return {'kind':'ground reception and semantics','cases':cases}


def wall_payload(test,wall_x,max_cm=1000):
    pos=test.state.get('LOCAL_POSITION_NED')
    attitude=test.state.get('ATTITUDE')
    if pos is None or attitude is None:
        return [65535]*72,max_cm
    output=[]
    for i in range(72):
        angle=attitude.yaw+math.radians(i*5)
        cosine=math.cos(angle)
        d=(wall_x-pos.x)/cosine if cosine>1e-3 else -1
        output.append(max(1,int(d*100)) if 0<d<=max_cm/100 else 65535)
    return output,max_cm


def flight(test,enabled,duration=16,dropout=False):
    wall=[None]
    test.sender=lambda:wall_payload(test,wall[0]) if wall[0] is not None else ([65535]*72,1000)
    test.takeoff()
    origin=test.state['LOCAL_POSITION_NED'].x
    wall[0]=origin+4
    test.pump(1)
    begin=time.monotonic()
    if dropout:
        test.sender=lambda:wall_payload(test,wall[0]) if time.monotonic()-begin<7 else ([65535]*72,1000)
    marker=len(test.messages)
    test.rc[1]=1100
    # Constant forward pilot input is maintained even when proximity avoidance stops motion.
    print(json.dumps({'event':'forward_start','avoidance_enabled':enabled,'wall_x':wall[0],'origin':origin}),flush=True)
    test.pump(duration)
    test.rc[1]=1500
    test.pump(2)
    positions=[m for m in test.messages[marker:] if m['mavpackettype']=='LOCAL_POSITION_NED']
    moving=[p for p in positions if p['wall_s']<begin-test.start+duration]
    last_window=[p for p in moving if p['wall_s']>begin-test.start+duration-2]
    if min(-p['z'] for p in moving)<2:
        raise RuntimeError('Flight test invalid: aircraft descended below 2 m')
    if not enabled and max(p['vx'] for p in moving)<0.2:
        raise RuntimeError('Baseline invalid: forward pilot input did not cause motion')
    result={'kind':'real ArduCopter SITL Loiter, analytic virtual wall','avoidance_enabled':enabled,
            'wall_x_m':wall[0],'start_x_m':origin,'pilot_pitch_pwm':1100,'command_duration_s':duration,
            'crossed_virtual_wall':any(p['x']>=wall[0] for p in moving),
            'minimum_gap_m':min(wall[0]-p['x'] for p in moving),
            'last_2s_mean_north_speed_mps':sum(p['vx'] for p in last_window)/max(1,len(last_window)),
            'maximum_north_speed_mps':max(p['vx'] for p in moving),
            'dropout_after_s':7 if dropout else None,
            'forward_start_wall_s':begin-test.start,
            'positions':positions,'note':'Virtual wall is sensor input only, not a physical collision mesh; crossing is detected geometrically.'}
    test.set_mode('LAND')
    test.wait(lambda:not(test.state['HEARTBEAT'].base_mode&M.MAV_MODE_FLAG_SAFETY_ARMED),25,'Land and disarm')
    return result


class VisualSensor:
    """Slow software renderer + actual stereo; independent of flight controller loop."""
    def __init__(self,test,config):
        import cv2
        from omniavoid.config import load_config
        from omniavoid.dataset import demo_config
        self.cfg=demo_config(load_config(config),640,400)
        self.cfg['stereo'].update(pair_workers=4,opencv_threads=1)
        self.opencv_version=cv2.__version__
        self.test=test
        self.pose=(-1.6,0,0,0)
        self.lock=threading.Lock()
        self.latest=None
        self.rows=[]
        self.error=None
        self.stop=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True)
        self.max_age_s=.65  # simulation setting, NOT the production capture deadline

    def run(self):
        try:
            from simulate_motion import Renderer,thumbnail
            from omniavoid.stereo import StereoPipeline
            from omniavoid.runtime import process_images
            renderer=Renderer(self.cfg)
            pipeline=StereoPipeline(self.cfg)
            try:
                while not self.stop.is_set():
                    with self.lock:pose=self.pose
                    started=time.monotonic()
                    epoch=time.time_ns()
                    images,render_ms=renderer.render(pose[:3],pose[3])
                    result,_,_=process_images(pipeline,self.cfg,images)
                    result.update(capture_wall_s=started-self.test.start,
                                  capture_epoch_ns=epoch,
                                  ready_wall_s=time.monotonic()-self.test.start,
                                  scene_pose=list(pose),render_ms=render_ms,thumbnail=thumbnail(images))
                    with self.lock:
                        self.latest=(started,result['distances_cm'],epoch)
                        self.rows.append(result)
            finally:pipeline.close()
        except Exception as error:
            self.error=repr(error)

    def payload(self,origin):
        if self.error:raise RuntimeError(self.error)
        p=self.test.state['LOCAL_POSITION_NED']
        yaw=math.degrees(self.test.state['ATTITUDE'].yaw)
        with self.lock:
            self.pose=(p.x-origin[0]-1.6,p.y-origin[1],0,yaw)
            latest=self.latest
        values=latest[1] if latest and time.monotonic()-latest[0]<=self.max_age_s else [65535]*72
        return values,int(self.cfg['range']['max_m']*100),True,latest[2] if latest else None


def visual_flight(test,config,duration=16):
    test.takeoff()
    p=test.state['LOCAL_POSITION_NED']
    origin=(p.x,p.y)
    sensor=VisualSensor(test,config)
    sensor.thread.start()
    test.sender=lambda:sensor.payload(origin)
    try:
        test.wait(lambda:len(sensor.rows)>=2 or sensor.error,15,'First optical frame')
        test.pump(1)
        marker=len(test.messages)
        begin=time.monotonic()
        test.rc[1]=1100
        test.pump(duration)
        test.rc[1]=1500
        test.pump(2)
    finally:
        test.sender=None
        sensor.stop.set()
        sensor.thread.join(timeout=15)
    if sensor.error:raise RuntimeError(sensor.error)
    if sensor.thread.is_alive():raise RuntimeError('Visual worker failed to stop')
    positions=[m for m in test.messages[marker:] if m['mavpackettype']=='LOCAL_POSITION_NED']
    moving=[p for p in positions if p['wall_s']<begin-test.start+duration]
    last=[p for p in moving if p['wall_s']>begin-test.start+duration-2]
    if min(-p['z'] for p in moving)<2:raise RuntimeError('Visual flight descended below 2m')
    result={'kind':'real ArduCopter SITL + actual four-fisheye stereo pipeline',
            'avoidance_enabled':True,'completed':True,'positions':positions,'visual_frames':sensor.rows,
            'wall_x_m':origin[0]+3,'start_x_m':origin[0],'forward_start_wall_s':begin-test.start,
            'command_duration_s':duration,'pilot_pitch_pwm':1100,
            'crossed_virtual_wall':any(p['x']>=origin[0]+3 for p in moving),
            'minimum_gap_m':min(origin[0]+3-p['x'] for p in moving),
            'last_2s_mean_north_speed_mps':sum(p['vx'] for p in last)/max(1,len(last)),
            'maximum_north_speed_mps':max(p['vx'] for p in moving),
            'sensor_max_age_s':sensor.max_age_s,
            'opencv_version':sensor.opencv_version,
            'note':'Ray-rendered 640x400 four-fisheye frames, actual stereo code, production MAVLink encoder, flight controller closes loop. Renderer is slow; repeated packets at 20Hz are NOT 20Hz fresh images. Camera roll/pitch and vertical movement omitted; scene fixed at flight height. Box spans +/-0.35m east. No collision mesh.'}
    test.set_mode('LAND')
    test.wait(lambda:not(test.state['HEARTBEAT'].base_mode&M.MAV_MODE_FLAG_SAFETY_ARMED),25,'Land and disarm')
    return result


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--binary',default='/home/eft/ardupilot/build/sitl/bin/arducopter')
    parser.add_argument('--repo',default='/home/eft/ardupilot')
    parser.add_argument('--output',required=True)
    parser.add_argument('--scenario',choices=['reception','avoid_on','avoid_off','avoid_dropout','visual_on'],required=True)
    parser.add_argument('--config',default=str(Path(__file__).resolve().parents[1]/'configs/rig.example.json'))
    args=parser.parse_args()
    test=SITL(args.binary,args.repo,args.output,args.scenario!='avoid_off')
    result={}
    try:
        test.connect()
        if args.scenario=='reception':result=reception(test)
        elif args.scenario=='visual_on':result=visual_flight(test,args.config)
        else:result=flight(test,args.scenario!='avoid_off',dropout=args.scenario=='avoid_dropout')
        result['completed']=True
    except Exception as error:
        result.update(completed=False,error=repr(error))
        print('ERROR',repr(error),flush=True)
        raise
    finally:
        result['metadata']=test.metadata()
        (test.directory/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
        test.close()
        print(json.dumps({k:v for k,v in result.items() if k not in ('metadata','cases','positions','visual_frames')},indent=2),flush=True)


if __name__=='__main__':
    main()

