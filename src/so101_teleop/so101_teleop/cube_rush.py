"""Cube Rush: bin as many cubes as you can in 60 s. After each score the cube respawns
and the bin jumps somewhere new. Knocking the cube off the table costs a point.
Pure game logic on (model, data), no ROS, so it can be tested headless."""
import json
import os
import mujoco
import numpy as np

ROUND_S = 60.0
R_MIN, R_MAX, ANG = 0.15, 0.28, np.radians(50)  # reachable fingers-down band on the table
HALF_IN = 0.044  # bin inner half-width
HS_FILE = os.path.expanduser("~/.so101_cube_rush.json")


class CubeRush:
    def __init__(self, m, d, seed=None):
        self.m, self.d = m, d
        self.rng = np.random.default_rng(seed)
        self.cube = m.body("cube").id
        j = m.joint("cube_joint")
        self.cq, self.cv = j.qposadr[0], j.dofadr[0]
        self.bin = m.body("bin").id
        self.pads = {m.geom("fixed_pad").id, m.geom("moving_pad").id}
        self.tip = m.site("grasp_center").id
        self.best = self._load_best()
        self.playing, self.score, self.t_end, self.in_bin_since = False, 0, 0.0, None
        self.splits = []  # seconds per cube, for the end-of-round summary

    def _load_best(self):
        try:
            return json.load(open(HS_FILE))["best"]
        except (OSError, ValueError, KeyError):
            return 0

    def _spot(self, avoid, min_dist):
        for _ in range(100):
            r, a = self.rng.uniform(R_MIN, R_MAX), self.rng.uniform(-ANG, ANG)
            p = np.array([r * np.cos(a), r * np.sin(a)])
            if all(np.linalg.norm(p - q) > min_dist for q in avoid):
                return p
        return p

    def _respawn(self):
        tip = self.d.site_xpos[self.tip][:2]
        b = self._spot([tip], 0.12)  # never drop the bin onto the gripper
        self.m.body_pos[self.bin][:2] = b
        c = self._spot([b, tip], 0.11)
        yaw = self.rng.uniform(0, np.pi / 2)
        self.d.qpos[self.cq:self.cq + 7] = [c[0], c[1], 0.0125, np.cos(yaw / 2), 0, 0, np.sin(yaw / 2)]
        self.d.qvel[self.cv:self.cv + 6] = 0
        mujoco.mj_forward(self.m, self.d)
        self.t_spawn, self.in_bin_since = self.d.time, None

    def start(self):
        self.playing, self.score, self.splits = True, 0, []
        self.t_end = self.d.time + ROUND_S
        self._respawn()
        return "GO! 60 s"

    def held(self):
        touching = set()
        for c in self.d.contact[: self.d.ncon]:
            if self.m.geom_bodyid[c.geom1] == self.cube:
                touching.add(c.geom2)
            elif self.m.geom_bodyid[c.geom2] == self.cube:
                touching.add(c.geom1)
        return bool(touching & self.pads)

    def update(self):
        """Call ~10 Hz. Returns an event string or None."""
        if not self.playing:
            return None
        if self.d.time >= self.t_end:
            self.playing = False
            new_best = self.score > self.best
            if new_best:
                self.best = self.score
                json.dump({"best": self.best}, open(HS_FILE, "w"))
            avg = f", {np.mean(self.splits):.1f} s per cube" if self.splits else ""
            return f"TIME! {self.score} cubes{avg}" + ("  NEW BEST!" if new_best else f"  (best {self.best})")
        cube = self.d.xpos[self.cube]
        if cube[2] < -0.05:
            self.score = max(0, self.score - 1)
            self._respawn()
            return "Off the table! -1"
        rel = np.abs(cube[:2] - self.m.body_pos[self.bin][:2])
        resting = (np.all(rel < HALF_IN) and cube[2] < 0.04 and not self.held()
                   and np.linalg.norm(self.d.qvel[self.cv:self.cv + 3]) < 0.05)
        if not resting:
            self.in_bin_since = None
            return None
        if self.in_bin_since is None:
            self.in_bin_since = self.d.time
        if self.d.time - self.in_bin_since > 0.3:  # must settle, not bounce through
            self.score += 1
            self.splits.append(self.d.time - self.t_spawn)
            self._respawn()
            return f"+1  ({self.splits[-1]:.1f} s)"
        return None

    def hud(self):
        if not self.playing:
            return "CUBE RUSH", f"press SPACE in the viewer to start\nbest: {self.best}"
        left = max(0.0, self.t_end - self.d.time)
        return "CUBE RUSH", f"time  {left:4.1f}\nscore {self.score}\nbest  {self.best}"
