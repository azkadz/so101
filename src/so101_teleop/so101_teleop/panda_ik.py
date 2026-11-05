"""Damped-least-squares IK with nullspace posture, after kevinzakka/mjctrl diffik_nullspace.py.

Runs on the sim's own MjData: reads the current q, returns new 7-joint position targets.
"""
import mujoco
import numpy as np

HOME = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
TCP_OFFSET = np.array([0.0, 0.0, 0.1034])  # hand frame -> point between fingertip pads


class PandaIK:
    def __init__(self, model, damping=1e-4, k_null=1.0, max_dq=0.05):
        self.m = model
        self.hand = model.body("hand").id
        self.dofs = np.array([model.joint(f"joint{i}").dofadr[0] for i in range(1, 8)])
        self.qadr = np.array([model.joint(f"joint{i}").qposadr[0] for i in range(1, 8)])
        self.lo, self.hi = model.jnt_range[[model.joint(f"joint{i}").id for i in range(1, 8)]].T
        self.damping, self.k_null, self.max_dq = damping, k_null, max_dq
        self.jacp, self.jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
        self.dk = mujoco.MjData(model)  # scratch data for IK so the sim state is untouched

    def tcp(self, d):
        R = d.xmat[self.hand].reshape(3, 3)
        return d.xpos[self.hand] + R @ TCP_OFFSET, R

    def solve(self, d, q_cmd, p_des, quat_des, iters=3):
        """One small DLS update per call (call every sim step). Iterates on q_cmd, not on d.qpos,
        so the command converges even while the PD controller lags."""
        q = q_cmd.copy()
        dk = self.dk
        dk.qpos[:] = d.qpos
        for _ in range(iters):
            dk.qpos[self.qadr] = q
            mujoco.mj_kinematics(self.m, dk)
            mujoco.mj_comPos(self.m, dk)
            p, R = self.tcp(dk)
            err = np.zeros(6)
            err[:3] = p_des - p
            q_cur = np.zeros(4)
            mujoco.mju_mat2Quat(q_cur, R.flatten())
            q_err = np.zeros(4)
            mujoco.mju_negQuat(q_cur, q_cur)
            mujoco.mju_mulQuat(q_err, quat_des, q_cur)
            mujoco.mju_quat2Vel(err[3:], q_err, 1.0)
            mujoco.mj_jac(self.m, dk, self.jacp, self.jacr, p, self.hand)
            J = np.vstack([self.jacp, self.jacr])[:, self.dofs]
            dq = J.T @ np.linalg.solve(J @ J.T + self.damping * np.eye(6), err)
            N = np.eye(7) - np.linalg.pinv(J) @ J
            dq += N @ (self.k_null * (HOME - q))
            dq = np.clip(dq, -self.max_dq, self.max_dq)
            q = np.clip(q + dq, self.lo, self.hi)
        return q
