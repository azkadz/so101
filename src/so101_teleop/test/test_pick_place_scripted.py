"""Headless check that the scene + IK + gripper can actually pick the cube and drop it in the bin.
Run: python3 test/test_pick_place_scripted.py [path/to/pick_place.xml]"""
import os, sys
import mujoco
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from so101_teleop.panda_ik import PandaIK

XML = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser(
    "~/wsp/playground/mujoco_menagerie/franka_emika_panda/pick_place.xml")


def run():
    m = mujoco.MjModel.from_xml_path(XML)
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, m.key("pick_home").id)
    mujoco.mj_forward(m, d)
    ik = PandaIK(m)
    _, R0 = ik.tcp(d)
    quat = np.zeros(4); mujoco.mju_mat2Quat(quat, R0.flatten())  # keep home (fingers-down) orientation
    cube = m.body("cube").id
    q_cmd = d.qpos[ik.qadr].copy()
    c0 = d.xpos[cube].copy()
    bin_c = m.body("bin").pos
    # (target xyz, gripper ctrl 0..255, seconds)
    plan = [(c0 + [0, 0, 0.15], 255, 2.0), (c0 + [0, 0, 0.0], 255, 1.5), (c0, 0, 1.0),
            (c0 + [0, 0, 0.20], 0, 1.5), (bin_c + [0, 0, 0.20], 0, 2.0), (bin_c + [0, 0, 0.20], 255, 1.0)]
    log = {}
    for i, (p, g, T) in enumerate(plan):
        for _ in range(int(T / m.opt.timestep)):
            q_cmd = ik.solve(d, q_cmd, p, quat, iters=1)
            d.ctrl[:7] = q_cmd
            d.ctrl[7] = g
            mujoco.mj_step(m, d)
        tcp, _ = ik.tcp(d)
        log[i] = (np.linalg.norm(tcp - p), d.xpos[cube].copy())
        print(f"step {i}: tcp err {log[i][0]*1000:.1f} mm, cube {np.round(d.xpos[cube], 3)}")
    for _ in range(500):
        mujoco.mj_step(m, d)
    lifted = log[3][1][2] > 0.15
    in_bin = np.all(np.abs(d.xpos[cube][:2] - bin_c[:2]) < 0.07) and d.xpos[cube][2] < 0.06
    print("lifted:", lifted, "in bin:", in_bin)
    return lifted and in_bin


if __name__ == "__main__":
    sys.exit(0 if run() else 1)
