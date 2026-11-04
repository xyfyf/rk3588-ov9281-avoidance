import numpy as np
from omniavoid.sync import Frame, Synchronizer


def frame(name, sequence, milliseconds):
    return Frame(name, sequence, int(milliseconds * 1e6), int(milliseconds * 1e6), np.zeros((1, 1), np.uint8))


def test_pairing_skew_and_oldest_discard():
    sync = Synchronizer(["a", "b"], 2, 100)
    sync.add(frame("a", 0, 100))
    sync.add(frame("b", 0, 110))
    assert sync.pop(int(111e6)) is None
    sync.add(frame("a", 1, 111))
    group = sync.pop(int(112e6))
    assert set(group) == {"a", "b"}
    assert group["a"].sequence == 1


def test_duplicate_regression_and_sequence_wrap():
    sync = Synchronizer(["a"], 2, 100)
    assert sync.add(frame("a", 0xFFFFFFFF, 100))
    assert sync.add(frame("a", 0, 101))
    assert not sync.add(frame("a", 0, 102))
    assert not sync.add(frame("a", 1, 100))
    assert not sync.add(frame("a", 0xFFFFFFFE, 103))
    assert sync.rejected == 3


def test_stale_future_and_missing_camera_are_rejected():
    sync = Synchronizer(["a", "b"], 2, 100)
    sync.add(frame("a", 0, 0))
    sync.add(frame("b", 0, 0))
    assert sync.pop(int(101e6)) is None
    sync.add(frame("a", 1, 300))
    sync.add(frame("b", 1, 300))
    assert sync.pop(int(200e6)) is None
    sync.add(frame("a", 2, 400))
    assert sync.pop(int(401e6)) is None


def test_queues_remain_bounded():
    sync = Synchronizer(["a", "b"], 2, 100)
    for i in range(100):
        sync.add(frame("a", i, i))
    assert len(sync.queues["a"]) == 4
    assert sync.discarded == 96


def test_latest_complete_group_prevents_processing_backlog_latency():
    sync = Synchronizer(["a", "b"], 2, 100)
    for i in range(3):
        sync.add(frame("a", i, 100 + i * 10))
        sync.add(frame("b", i, 101 + i * 10))
    group = sync.pop(int(122e6))
    assert group["a"].sequence == group["b"].sequence == 2
    assert sync.discarded == 4
