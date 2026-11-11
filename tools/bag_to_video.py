"""Render a teleop bag to a demo video: camera | MuJoCo sim, joint tracking plots underneath.

python3 tools/bag_to_video.py bags/demo_01            # -> bags/demo_01.mp4 + bags/demo_01_tracking.png

The sim panel is re-simulated from the recorded leader commands (same mapping as so101_sim_node),
so it starts from the scene's initial state at the start of the bag.
"""
import argparse
import os

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
TOPICS = ['/leader/joint_states', '/real/joint_states', '/camera/image_raw']
W, H, PLOT_H, FPS = 640, 480, 320, 30


def read_bag(path):
    """-> {'/leader/joint_states': (t, q[N,6]), '/real/joint_states': (t, q), '/camera/image_raw': (t, [bgr])}"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types()}
    raw = {k: ([], []) for k in TOPICS}
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic not in raw:
            continue
        msg = deserialize_message(data, types[topic])
        if topic == '/camera/image_raw':
            img = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, -1)
            img = cv2.cvtColor(img, cv2.COLOR_YUV2BGR_YUY2) if msg.encoding == 'yuv422_yuy2' else \
                cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            val = cv2.resize(img, (W, H))
        else:
            pos = dict(zip(msg.name, msg.position))
            val = [pos[j] for j in JOINTS]
        raw[topic][0].append(t * 1e-9)
        raw[topic][1].append(val)
    return {k: (np.array(ts), np.array(v) if k != '/camera/image_raw' else v) for k, (ts, v) in raw.items()}


def latest(ts, t):
    """Index of the last sample at or before t (0 if none yet)."""
    return max(int(np.searchsorted(ts, t, side='right')) - 1, 0)


def tracking_plot(data, t0, path=None):
    """Leader vs real follower per joint. Returns (rgb image, figure, axes) for cursor drawing."""
    fig, axes = plt.subplots(1, 6, figsize=(2 * W / 100, PLOT_H / 100), dpi=100, sharex=True)
    for i, ax in enumerate(axes):
        for topic, style, label in [('/leader/joint_states', '-', 'leader'), ('/real/joint_states', '--', 'real')]:
            ts, q = data[topic]
            if len(ts):
                ax.plot(ts - t0, q[:, i], style, lw=1.2, label=label)
        ax.set_title(JOINTS[i], fontsize=8)
        ax.tick_params(labelsize=6)
    axes[0].set_ylabel('rad (gripper 0..1)', fontsize=7)
    axes[0].legend(fontsize=6, loc='upper left')
    fig.supxlabel('time (s)', fontsize=7)
    fig.tight_layout()
    if path:
        fig.savefig(path, dpi=150)
    fig.canvas.draw()
    img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR), fig, axes


def cursor_x(fig, axes, t):
    """Pixel x of time t in each axis (figure pixel coords, origin top-left)."""
    return [int(ax.transData.transform((t, 0))[0]) for ax in axes]


def render(data, m, load_ctrl, out_mp4, plot_png=None):
    """data from read_bag; load_ctrl(q6) -> ctrl vector for the sim (same mapping as the live node)."""
    starts = [ts[0] for ts, _ in data.values() if len(ts)]
    ends = [ts[-1] for ts, _ in data.values() if len(ts)]
    t0, t1 = min(starts), max(ends)
    plot, fig, axes = tracking_plot(data, t0, plot_png)
    d = mujoco.MjData(m)
    lt, lq = data['/leader/joint_states']
    ct, frames = data['/camera/image_raw']
    if len(lq):  # start the arm where the leader is, so it doesn't swing in from zero
        ctrl = load_ctrl(lq[0])
        d.ctrl[:] = ctrl
        for j, v in zip(JOINTS[:5], lq[0][:5]):
            d.qpos[m.joint(j).qposadr[0]] = v
        mujoco.mj_forward(m, d)
    renderer = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(m, cam)
    cam.lookat[:] = [0.15, 0.0, 0.15]  # arm base + table workspace, room for upright poses
    cam.distance, cam.azimuth, cam.elevation = 0.95, 120, -25
    vw = cv2.VideoWriter(out_mp4, cv2.VideoWriter_fourcc(*'mp4v'), FPS, (2 * W, H + plot.shape[0]))
    sim_t = t0
    for t in np.arange(t0, t1, 1 / FPS):
        while sim_t < t:  # physics catches up to the video clock, using the latest leader command
            if len(lq):
                d.ctrl[:] = load_ctrl(lq[latest(lt, sim_t)])
            mujoco.mj_step(m, d)
            sim_t += m.opt.timestep
        renderer.update_scene(d, cam)
        sim = cv2.cvtColor(renderer.render(), cv2.COLOR_RGB2BGR)
        camf = frames[latest(ct, t)].copy() if len(frames) else np.zeros((H, W, 3), np.uint8)
        for img, label in [(camf, 'camera'), (sim, 'MuJoCo sim')]:
            cv2.putText(img, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.putText(camf, f'{t - t0:5.1f} s', (W - 110, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        p = plot.copy()
        for x in cursor_x(fig, axes, t - t0):
            cv2.line(p, (x, 0), (x, p.shape[0] - 1), (0, 0, 255), 1)
        vw.write(np.vstack([np.hstack([camf, sim]), p]))
    vw.release()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag')
    ap.add_argument('--model', default=None, help='scene XML (default: installed pick_place.xml)')
    args = ap.parse_args()
    from so101_teleop import so101_sim_node as sn  # reuse the live node's model loader + gripper mapping
    model = args.model
    if model is None:
        from ament_index_python.packages import get_package_share_directory
        model = os.path.join(get_package_share_directory('so101_teleop'), 'models', 'so101', 'pick_place.xml')
    m = sn.load_model(model)
    act = [m.actuator(n).id for n in JOINTS]
    lo, hi = m.actuator_ctrlrange[act].T

    def load_ctrl(q):
        ctrl = np.zeros(m.nu)
        g = float(np.clip(q[5], 0.0, 1.0))
        ctrl[act] = np.clip(np.r_[q[:5], sn.G_CLOSED + g * (sn.G_OPEN - sn.G_CLOSED)], lo, hi)
        return ctrl

    base = args.bag.rstrip('/')
    render(read_bag(base), m, load_ctrl, base + '.mp4', base + '_tracking.png')
    print(f'wrote {base}.mp4 and {base}_tracking.png')


if __name__ == '__main__':
    main()
