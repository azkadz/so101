import os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from so101_teleop.latency import xcorr_lag


def test_known_delay_recovered():
    rng = np.random.default_rng(0)
    t = np.arange(0, 10, 0.01)  # 100 Hz like the bag
    a = np.stack([np.sin(1.3 * t), np.cos(0.7 * t), 0.3 * np.sin(2.1 * t + 1)], 1)
    delay = 0.080
    tb = t + 0.003  # unaligned sample times
    b = np.stack([np.interp(tb - delay, t, a[:, i]) for i in range(3)], 1) - 0.007  # + constant sag
    b += 1e-4 * rng.standard_normal(b.shape)
    # resolution is bounded by the 100 Hz sampling: +-half a sample period
    assert abs(xcorr_lag(t, a, tb, b) - delay) <= 0.005


if __name__ == "__main__":
    test_known_delay_recovered(); print("ok")
