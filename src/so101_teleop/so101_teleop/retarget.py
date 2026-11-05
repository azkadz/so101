"""SO-101 leader joints -> Panda TCP target, by a scaled offset from a fixed anchor.

The anchor is the leader at q = 0 (its zero_ticks pose, which was set in the Panda-home shape),
so the same leader pose always gives the same Panda pose. reanchor() moves it to the current pose (clutch).

p_des = p_panda_home + scale * (p_leader - p_leader_anchor)
R_des = (R_leader R_leader_anchor^T) R_panda_home
Both base frames face +x with z up, so the base-to-base rotation is identity.
"""
import mujoco
import numpy as np

PANDA_HOME_P = np.array([0.5545, 0.0, 0.5211])        # TCP at pick_home keyframe (from panda_ik.tcp)
PANDA_HOME_QUAT = np.array([0.0, 0.7071068, 0.7071068, 0.0])  # fingers down


class SO101Retarget:
    def __init__(self, so101_xml, sign, offset, scale=2.0, track_orientation=True):
        self.m = mujoco.MjModel.from_xml_path(so101_xml)
        self.d = mujoco.MjData(self.m)
        self.site = self.m.site("gripperframe").id
        self.qadr = [self.m.joint(n).qposadr[0] for n in
                     ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")]
        self.sign, self.offset = np.asarray(sign, float), np.asarray(offset, float)
        self.scale, self.track_orientation = scale, track_orientation
        self.anchor = self.leader_fk(np.zeros(5))
        self.R_home = np.zeros(9)
        mujoco.mju_quat2Mat(self.R_home, PANDA_HOME_QUAT)
        self.R_home = self.R_home.reshape(3, 3)

    def leader_fk(self, q_leader5):
        """Leader angles (leader_node convention) -> gripperframe pose in SO-101 base frame."""
        self.d.qpos[self.qadr] = self.sign * np.asarray(q_leader5) + self.offset
        mujoco.mj_kinematics(self.m, self.d)
        return self.d.site_xpos[self.site].copy(), self.d.site_xmat[self.site].reshape(3, 3).copy()

    def reanchor(self, q_leader5=None):
        """No argument: back to the q = 0 anchor. With the current q: clutch, so this pose maps to Panda home."""
        self.anchor = self.leader_fk(np.zeros(5) if q_leader5 is None else q_leader5)

    def update(self, q_leader5):
        p, R = self.leader_fk(q_leader5)
        p0, R0 = self.anchor
        p_des = PANDA_HOME_P + self.scale * (p - p0)
        p_des[2] = max(p_des[2], 0.005)  # never ask the fingertips to go through the table
        R_des = (R @ R0.T) @ self.R_home if self.track_orientation else self.R_home
        quat = np.zeros(4)
        mujoco.mju_mat2Quat(quat, R_des.flatten())
        return p_des, quat