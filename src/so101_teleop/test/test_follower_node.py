import math
from so101_teleop.follower_node import leader_to_ticks, clamp, ramp

Z = [2000, 2100, 900, 2400, 2000]
S = [1, -1, 1, 1, 1]
G = [1500, 2700]


def test_zero_pose_maps_to_zero_ticks():
    assert leader_to_ticks([0] * 5 + [0.0], Z, S, G) == Z + [1500]


def test_sign_and_quarter_turn():
    t = leader_to_ticks([0, math.pi / 2, 0, 0, 0, 1.0], Z, S, G)
    assert t[1] == 2100 - 1024 and t[5] == 2700


def test_gripper_saturates():
    assert leader_to_ticks([0] * 5 + [3.0], Z, S, G)[5] == 2700


def test_clamp():
    lim = [1000, 3000] * 6
    assert clamp([0, 4000, 2000, 1000, 3000, 5000], lim) == [1000, 3000, 2000, 1000, 3000, 3000]


def test_ramp_endpoints():
    assert ramp([0, 100], [100, 0], 0) == [0, 100]
    assert ramp([0, 100], [100, 0], 0.5) == [50, 50]
    assert ramp([0, 100], [100, 0], 5) == [100, 0]
