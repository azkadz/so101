"""Load the SO-101 pick-and-place scene with graspable jaws.

MuJoCo collides meshes by their convex hulls. The fixed jaw's hull covers the whole gap between
the jaws, so a cube can never get in. We make both jaw meshes visual-only and add thin box pads
where the real fingers are (measured from the mesh vertices at gripper = 0).
"""
import mujoco

FRICTION = [1.5, 0.01, 0.0002]


def load_model(xml):
    spec = mujoco.MjSpec.from_file(xml)
    for g in spec.geoms:
        if g.meshname in ("wrist_roll_follower_so101_v1", "moving_jaw_so101_v1"):
            g.contype, g.conaffinity = 0, 0
    box = mujoco.mjtGeom.mjGEOM_BOX
    # fixed finger, in the gripper body frame: inner face at x = -0.008, tips at z = -0.103
    spec.body("gripper").add_geom(name="fixed_pad", type=box, size=[0.003, 0.008, 0.015],
                                  pos=[-0.011, 0, -0.088], friction=FRICTION, rgba=[0, 0, 0, 0])
    # moving finger, in the jaw body frame (jaw x = gripper x, jaw y = gripper z)
    spec.body("moving_jaw_so101_v1").add_geom(name="moving_pad", type=box, size=[0.003, 0.015, 0.008],
                                              pos=[-0.0092, -0.0646, 0.0188], friction=FRICTION,
                                              rgba=[0, 0, 0, 0])
    # where a held 2.5 cm cube's centre sits (fixed pad inner face + 12.5 mm), for logging and tests
    spec.body("gripper").add_site(name="grasp_center", pos=[0.0045, 0, -0.09], size=[0.004] * 3,
                                  rgba=[0, 1, 0, 0.5])
    return spec.compile()
