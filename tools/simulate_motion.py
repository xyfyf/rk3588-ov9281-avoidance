"""Dynamic four-fisheye optical simulation with the real stereo pipeline.

All physics/control here is an explicitly simplified one-dimensional point mass.
No MAVLink transmission, real vehicle control, PX4 or ArduPilot is involved.
"""
from __future__ import annotations

import argparse
import base64
from collections import deque
import copy
import json
from pathlib import Path
import platform
import time
import cv2
import numpy as np
from omniavoid.config import load_config
from omniavoid.dataset import demo_config, intersect_scene, scene_surfaces
from omniavoid.geometry import Camera, unproject
from omniavoid.runtime import process_images
from omniavoid.stereo import StereoPipeline


class Renderer:
    def __init__(self, cfg):
        self.cameras = [Camera.from_dict(d) for d in cfg['cameras']]
        self.rays, self.valid = {}, {}
        for c in self.cameras:
            u,v = np.meshgrid(np.arange(c.width),np.arange(c.height))
            rays,valid = unproject(c,np.stack((u,v),axis=-1))
            self.rays[c.name] = (rays@c.R.T).astype(np.float32)
            self.valid[c.name] = valid
        self.textures = []
        for i in range(len(scene_surfaces())):
            rng = np.random.default_rng(1000+i)
            texture = rng.integers(25,230,size=(512,512),dtype=np.uint8)
            self.textures.append(cv2.GaussianBlur(texture,(3,3),0.5))

    def render(self, position, yaw_deg=0, low_texture=False, dropout=None):
        yaw = np.deg2rad(yaw_deg)
        R = np.array([[np.cos(yaw),-np.sin(yaw),0],[np.sin(yaw),np.cos(yaw),0],[0,0,1]],dtype=np.float32)
        position = np.asarray(position,dtype=np.float32)
        images = {}
        start = time.perf_counter()
        for c in self.cameras:
            directions = self.rays[c.name]@R.T
            origin = R@c.t+position
            distance,which = intersect_scene(origin,directions)
            hit = origin+np.where(np.isfinite(distance),distance,0)[...,None]*directions
            image = np.zeros((c.height,c.width),np.uint8)
            for i,(axis,_,_) in enumerate(scene_surfaces()):
                mask=(which==i)&self.valid[c.name]
                if not mask.any():
                    continue
                if low_texture:
                    image[mask]=140
                    continue
                axes=[j for j in range(3) if j!=axis]
                # Sample only visible pixels, not the whole sensor once per surface.
                mx=np.remainder(hit[...,axes[0]][mask]*100+256,510).astype(np.float32).reshape(1,-1)
                my=np.remainder(hit[...,axes[1]][mask]*100+256,510).astype(np.float32).reshape(1,-1)
                # OpenCV remap caps dimensions at SHRT_MAX; process bounded chunks.
                values=np.empty(mx.shape[1],np.uint8)
                for offset in range(0,len(values),30000):
                    values[offset:offset+30000]=cv2.remap(self.textures[i],mx[:,offset:offset+30000],my[:,offset:offset+30000],cv2.INTER_LINEAR).ravel()
                image[mask]=values
            images[c.name]=np.zeros_like(image) if dropout==c.name else image
        return images,(time.perf_counter()-start)*1000


def thumbnail(images):
    tiles=[]
    for name in ('front','right','back','left'):
        tile=cv2.resize(images[name],(256,160))
        tile=cv2.cvtColor(tile,cv2.COLOR_GRAY2BGR)
        cv2.putText(tile,name,(8,20),cv2.FONT_HERSHEY_SIMPLEX,0.55,(80,255,120),1,cv2.LINE_AA)
        tiles.append(tile)
    contact=np.vstack((np.hstack(tiles[:2]),np.hstack(tiles[2:])))
    ok,encoded=cv2.imencode('.jpg',contact,[cv2.IMWRITE_JPEG_QUALITY,65])
    if not ok:
        raise IOError('Thumbnail encoding failed')
    return 'data:image/jpeg;base64,'+base64.b64encode(encoded).decode('ascii')


def front_clearance(distances):
    values=[distances[i] for i in (70,71,0,1,2)]
    known=[v for v in values if v!=65535]
    # Explicit demo policy, not either flight controller's actual logic.
    return min(known)/100 if distances[0]!=65535 and len(known)>=3 else None


def summarize(rows):
    timing=np.array([r['processing_ms'] for r in rows])
    render=np.array([r['render_ms'] for r in rows])
    return {'frames':len(rows),'processing_p50_ms':float(np.median(timing)),
            'processing_p95_ms':float(np.percentile(timing,95)),
            'processing_max_ms':float(timing.max()),'render_p50_ms':float(np.median(render)),
            'valid_sectors_min':min(r['valid_sectors'] for r in rows),
            'valid_sectors_median':float(np.median([r['valid_sectors'] for r in rows])),
            'front_unknown_frames':sum(r['front_clearance_m'] is None for r in rows)}


def run_open_loop(cfg, renderer, pipeline):
    cases=[]
    for name,poses in (
        ('approach',[(float(x),0.0,False,None) for x in np.linspace(-1.6,0.8,25)]),
        ('yaw', [(-0.5,float(yaw),False,None) for yaw in np.linspace(-90,90,19)]),
        ('texture_and_dropout',[(-0.5,0.0,i>=4,'front' if i<4 else None) for i in range(8)])):
        rows=[]
        for i,(x,yaw,low_texture,dropout) in enumerate(poses):
            images,render_ms=renderer.render([x,0,0],yaw,low_texture,dropout)
            result,_,_=process_images(pipeline,cfg,images)
            result.update(t_s=i*0.1,x_m=x,yaw_deg=yaw,speed_mps=0.0,
                          true_front_box_clearance_m=1.4-x,
                          front_clearance_m=front_clearance(result['distances_cm']),
                          render_ms=render_ms,thumbnail=thumbnail(images),
                          fault='low_texture' if low_texture else ('front_black_frame' if dropout else ''),
                          controller_state='open_loop_prescribed_pose',
                          truth_note='Box x clearance; only same as forward range at yaw=0')
            rows.append(result)
        case={'name':name,'kind':'prescribed trajectory, no flight controller','summary':summarize(rows),'frames':rows}
        if name=='approach':
            case['summary']['front_lower_than_half_true_gap_frames']=sum(
                r['front_clearance_m'] is not None and r['front_clearance_m']<r['true_front_box_clearance_m']/2 for r in rows)
        cases.append(case)
        print(json.dumps({'case':name,**case['summary']}),flush=True)
    return cases


def run_point_mass(cfg,renderer,pipeline,initial_speed):
    # Sensor period/delay/deceleration are simulation assumptions, not measurements.
    period,delay,a,margin=0.1,0.15,1.0,1.0
    t,x,v=0.0,-1.6,float(initial_speed)
    pending=deque()
    active=None
    rows=[]
    collision=False
    frame_index=0
    while t<6:
        images,render_ms=renderer.render([x,0,0])
        result,_,_=process_images(pipeline,cfg,images)
        clearance=front_clearance(result['distances_cm'])
        pending.append((t+delay,clearance,t))
        # Model capture-to-controller latency in simulation time. Render wall time is excluded.
        while pending and pending[0][0]<=t+1e-9:
            _,active,capture_t=pending.popleft()
        stopping=v*v/(2*a)+margin+v*(delay+period)
        brake=active is None or active<=stopping
        # First result arrives after initial delay; model keeps initial command in this window.
        if t<delay:
            brake=False
        state='brake_unknown' if brake and active is None else ('braking' if brake else 'hold_speed')
        result.update(t_s=round(t,3),x_m=x,yaw_deg=0.0,speed_mps=v,
                      true_front_box_clearance_m=1.4-x,front_clearance_m=clearance,
                      render_ms=render_ms,thumbnail=thumbnail(images),fault='',
                      controller_state=state,controller_distance_m=active)
        rows.append(result)
        # Exact 1D constant-deceleration update, no vehicle attitude or motor model.
        for _ in range(10):
            dt=period/10
            next_v=max(0,v-a*dt) if brake else v
            x+=(v+next_v)*0.5*dt
            v=next_v
            t+=dt
            if x>=1.4:
                collision=True
                break
        frame_index+=1
        if collision or (v<1e-6 and t>delay+period):
            break
    case={'name':f'point_mass_{initial_speed:g}mps',
          'kind':'closed-loop simplified point mass, NOT PX4/ArduPilot SITL',
          'assumptions':{'sensor_period_s':period,'capture_to_control_delay_s':delay,
                         'deceleration_mps2':a,'margin_m':margin,'initial_speed_mps':initial_speed,
                         'vehicle_radius_m':0,'continuous_unmodelled_real_vehicle_dynamics':False},
          'summary':{**summarize(rows),'collision':collision,'final_speed_mps':v,
                     'final_true_box_clearance_m':1.4-x,'final_time_s':t},'frames':rows}
    print(json.dumps({'case':case['name'],**case['summary']}),flush=True)
    return case


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--config',default='configs/rig.example.json')
    parser.add_argument('--output',required=True)
    parser.add_argument('--benchmark')
    parser.add_argument('--sustained',help='Optional sustained benchmark JSON to embed in the report')
    parser.add_argument('--sensor-width',type=int,default=640)
    parser.add_argument('--sensor-height',type=int,default=400)
    parser.add_argument('--pair-workers',type=int,default=4)
    parser.add_argument('--opencv-threads',type=int,default=1)
    args=parser.parse_args()
    cfg=demo_config(load_config(args.config),args.sensor_width,args.sensor_height)
    cfg['stereo'].update(pair_workers=args.pair_workers,opencv_threads=args.opencv_threads)
    output=Path(args.output)
    output.mkdir(parents=True,exist_ok=True)
    renderer,pipeline=Renderer(cfg),StereoPipeline(cfg)
    started=time.perf_counter()
    try:
        cases=run_open_loop(cfg,renderer,pipeline)
        cases+=[run_point_mass(cfg,renderer,pipeline,s) for s in (1,3)]
    finally:
        pipeline.close()
    result={'schema_version':1,'source':'synthetic four fisheye ray casting, actual CPU stereo code',
            'platform':platform.platform(),'opencv':cv2.__version__,
            'sensor_size':[args.sensor_width,args.sensor_height],
            'virtual_size':[cfg['stereo']['width'],cfg['stereo']['height']],
            'wall_seconds':time.perf_counter()-started,'cases':cases,
            'not_tested':['PX4 controller','ArduPilot controller','real camera','RK3588','flight dynamics'],
            'note':'Frame slider time is model time. Playback speed is not live compute speed. Rendering is timed separately.'}
    if args.benchmark:
        result['benchmark']=json.loads(Path(args.benchmark).read_text(encoding='utf-8'))
    if args.sustained:
        result['sustained']=json.loads(Path(args.sustained).read_text(encoding='utf-8'))
    (output/'simulation.json').write_text(json.dumps(result,ensure_ascii=False),encoding='utf-8')
    (output/'rig.simulation.json').write_text(json.dumps(cfg,indent=2),encoding='utf-8')
    # Import sibling tool without changing the import path or installing a package.
    from simulation_report import write_report
    write_report(output/'simulation.html',result)
    print(json.dumps({'output':str(output),'wall_seconds':result['wall_seconds']}),flush=True)


if __name__=='__main__':
    main()
