"""Headless Cube Rush: a scripted IK player bins 3 cubes and the game must count 3.
Also checks every spawn is reachable. Run: python3 test/test_cube_rush.py [pick_place.xml]"""
import os, sys
import mujoco
import numpy as np
from scipy.optimize import least_squares
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from so101_teleop.so101_scene import load_model
import so101_teleop.cube_rush as cr

XML = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/wsp/playground/SO-ARM100/Simulation/SO101/pick_place.xml")
ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
OPEN, CLOSED = 1.2, -0.17


def run():
    cr.HS_FILE = os.devnull  # don't touch the real high score
    m = load_model(XML)
    d, k = mujoco.MjData(m), mujoco.MjData(m)
    qa = [m.joint(n).qposadr[0] for n in ARM]
    site = m.site("grasp_center").id
    lo, hi = m.jnt_range[[m.joint(n).id for n in ARM]].T

    def ik(p):
        def f(q):
            k.qpos[qa] = q
            mujoco.mj_kinematics(m, k)
            return np.r_[k.site_xpos[site] - p, 0.05 * (k.site_xmat[site].reshape(3, 3)[:, 2] - [0, 0, 1])]
        r = least_squares(f, [np.arctan2(p[1], p[0]), 0, 0, 1.0, 0], bounds=(lo, hi))
        assert np.linalg.norm(r.fun[:3]) < 0.004, f"unreachable {p}"
        return r.x

    game = cr.CubeRush(m, d, seed=1)
    events = []

    def go(q, g, T):
        q0 = d.ctrl[:5].copy()
        n = int(T / m.opt.timestep)
        for i in range(n):
            d.ctrl[:5] = q0 + (q - q0) * min(1, 2 * i / n)
            d.ctrl[5] = g
            mujoco.mj_step(m, d)
            if i % 50 == 0 and (e := game.update()):
                events.append(e)
                print(f"t={d.time:5.1f}  {e}")

    q_up = ik(np.array([0.2, 0, 0.08]))
    d.qpos[qa] = d.ctrl[:5] = q_up
    mujoco.mj_forward(m, d)
    print(game.start())
    for _ in range(3):
        c, b = d.xpos[game.cube].copy(), m.body_pos[game.bin].copy()
        go(ik(c + [0, 0, 0.06]), OPEN, 1.0)
        go(ik(c), OPEN, 1.0)
        go(ik(c), CLOSED, 0.8)
        go(ik(c + [0, 0, 0.07]), CLOSED, 1.0)
        go(ik(b + [0, 0, 0.08]), CLOSED, 1.5)
        go(ik(b + [0, 0, 0.08]), OPEN, 1.5)
        go(ik(b + [0, 0, 0.12]), OPEN, 0.5)  # clear the bin before it jumps
    print("score:", game.score)
    for _ in range(300):  # 300 more spawns, all must be reachable
        game._respawn()
        ik(d.xpos[game.cube] + [0, 0, 0.0]); ik(m.body_pos[game.bin] + [0, 0, 0.08])
    print("300 spawns reachable")
    return game.score == 3


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
