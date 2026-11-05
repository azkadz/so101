"""Headless check that the SO-101 scene can pick the cube and drop it in the bin.
Joint targets come from a small least-squares IK used only by this test.
Run: python3 test/test_so101_pick_place.py [path/to/pick_place.xml]"""
import os, sys
import mujoco
import numpy as np
from scipy.optimize import least_squares
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from so101_teleop.so101_scene import load_model

XML = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    os.path.join(os.path.dirname(__file__), "..", "models", "so101", "pick_place.xml"))
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
OPEN, CLOSED = 1.2, -0.17


def run():
    m = load_model(XML)
    d, k = mujoco.MjData(m), mujoco.MjData(m)
    qa = [m.joint(n).qposadr[0] for n in ARM]
    site = m.site("grasp_center").id
    lo, hi = m.jnt_range[[m.joint(n).id for n in ARM]].T

    def ik(p, q0):  # grasp_center at p, fingers pointing straight down
        def f(q):
            k.qpos[qa] = q
            mujoco.mj_kinematics(m, k)
            return np.r_[k.site_xpos[site] - p, 0.05 * (k.site_xmat[site].reshape(3, 3)[:, 2] - [0, 0, 1])]
        return least_squares(f, q0, bounds=(lo, hi)).x

    def go(q, g, T):
        q0 = d.ctrl[:5].copy()
        n = int(T / m.opt.timestep)
        for i in range(n):
            d.ctrl[:5] = q0 + (q - q0) * min(1, 2 * i / n)
            d.ctrl[5] = g
            mujoco.mj_step(m, d)

    mujoco.mj_forward(m, d)
    cube, bin_c = m.body("cube").id, m.body("bin").pos.copy()
    c = d.xpos[cube].copy()
    q_above = ik(c + [0, 0, 0.06], np.array([0, 0, 0, 1.0, 0]))
    q_at, q_bin = ik(c, q_above), ik(bin_c + [0, 0, 0.08], q_above)
    d.qpos[qa] = d.ctrl[:5] = q_above
    d.ctrl[5] = OPEN
    mujoco.mj_forward(m, d)
    for name, (q, g, T) in [("down", (q_at, OPEN, 1.5)), ("close", (q_at, CLOSED, 1.0)),
                            ("lift", (q_above, CLOSED, 1.5)), ("over bin", (q_bin, CLOSED, 2.0)),
                            ("release", (q_bin, OPEN, 1.0))]:
        go(q, g, T)
        print(f"{name:9s} cube {np.round(d.xpos[cube], 3)}")
        if name == "lift":
            lifted = d.xpos[cube][2] > 0.05
    go(q_bin, OPEN, 1.0)
    in_bin = np.all(np.abs(d.xpos[cube][:2] - bin_c[:2]) < 0.044) and d.xpos[cube][2] < 0.03
    print("lifted:", lifted, "in bin:", in_bin)
    return lifted and in_bin


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
