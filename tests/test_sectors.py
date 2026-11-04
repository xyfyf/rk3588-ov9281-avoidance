import copy
import numpy as np
from omniavoid.sectors import UNKNOWN, aggregate, advisory_speed_mps, sector_indices


def points_at(angle, distance, count=4):
    a = np.deg2rad(angle)
    return np.tile([distance * np.cos(a), distance * np.sin(a), 0], (count, 1))


def test_unknown_is_not_free(cfg):
    distances, _ = aggregate(np.empty((0, 3)), np.empty(0), cfg["range"])
    assert np.all(distances == UNKNOWN)
    points = points_at(0, cfg["range"]["max_m"] + 1)
    distances, _ = aggregate(points, np.zeros(4), cfg["range"])
    assert np.all(distances == UNKNOWN)


def test_frd_cardinals_and_wrap():
    points = np.concatenate([points_at(a, 2, 1) for a in (0, 90, 180, 270, 359, 2.49, 2.51)])
    assert sector_indices(points).tolist() == [0, 18, 36, 54, 0, 0, 1]


def test_nearest_supported_cluster_beats_far_background(cfg):
    points = np.concatenate([points_at(0, 1.5, 4), points_at(0, 4, 20)])
    distances, _ = aggregate(points, np.zeros(len(points)), cfg["range"])
    assert distances[0] == 150
    assert np.sum(distances != UNKNOWN) == 1


def test_unconfirmed_foreground_makes_sector_unknown_instead_of_background(cfg):
    points = np.concatenate([points_at(0, 0.3, 1), points_at(0, 1.5, 4)])
    distances, _ = aggregate(points, np.zeros(len(points)), cfg["range"])
    assert distances[0] == UNKNOWN


def test_sector_angle_offset_rotates_body_cardinals():
    points = np.concatenate([points_at(a, 2, 1) for a in (0, 90, 180, 270)])
    assert sector_indices(points, 90).tolist() == [54, 0, 18, 36]


def test_uncertainty_subtracted_and_saturates_at_zero(cfg):
    points = points_at(90, 1)
    distances, _ = aggregate(points, np.full(4, 0.1), cfg["range"])
    assert distances[18] == 80
    distances, _ = aggregate(points, np.full(4, 10), cfg["range"])
    assert distances[18] == 0


def test_height_band_and_nonfinite_do_not_create_free_space(cfg):
    points = points_at(0, 2)
    points[:, 2] = 2
    distances, _ = aggregate(points, np.zeros(4), cfg["range"])
    assert np.all(distances == UNKNOWN)
    points[:] = np.nan
    distances, _ = aggregate(points, np.zeros(4), cfg["range"])
    assert np.all(distances == UNKNOWN)


def test_optional_inflation_wraps_front(cfg):
    config = copy.deepcopy(cfg["range"])
    config["angular_inflation_radius_m"] = 0.3
    distances, _ = aggregate(points_at(0, 1), np.zeros(4), config)
    assert distances[0] == distances[71] == distances[1] == 100
    assert distances[36] == UNKNOWN


def test_braking_formula_is_dimensionally_consistent():
    v = advisory_speed_mps(3, 0.2, 1, 1)
    assert np.isclose(v * 0.2 + v * v / 2 + 1, 3)
    assert advisory_speed_mps(0.5, 0.2, 1, 1) == 0
    assert advisory_speed_mps(np.nan, 0.2, 1, 1) == 0
