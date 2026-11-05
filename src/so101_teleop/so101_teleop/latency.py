"""Leader -> follower latency from a ros2 bag of /leader/joint_states and /follower/joint_states.

Both topics use the same joint names and zero (so101_sim_node publishes in the leader's convention).
Usage: ros2 run so101_teleop latency <bag_dir> [--plot out.png]
"""
import argparse
import numpy as np

ARM = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def xcorr_lag(t_a, a, t_b, b, max_lag=0.5, dt=0.001):
    """Delay (s) of b behind a. a, b are (N, k) signals on their own timestamps.
    Correlates velocities, so a constant tracking offset (e.g. gravity sag) does not bias the result."""
    t = np.arange(max(t_a[0], t_b[0]), min(t_a[-1], t_b[-1]), dt)
    A = np.stack([np.interp(t, t_a, a[:, i]) for i in range(a.shape[1])], 1)
    B = np.stack([np.interp(t, t_b, b[:, i]) for i in range(b.shape[1])], 1)
    dA, dB = np.diff(A, axis=0), np.diff(B, axis=0)
    lags = np.arange(int(max_lag / dt) + 1)
    N = len(dA)
    # normalised over each lag's own overlap, so the finite window does not bias the peak
    score = [np.sum(dA[:N - k] * dB[k:]) / (np.linalg.norm(dA[:N - k]) * np.linalg.norm(dB[k:]) + 1e-12)
             for k in lags]
    return lags[int(np.argmax(score))] * dt


def read_joints(bag, topic):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import JointState
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag), rosbag2_py.ConverterOptions("", ""))
    r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    t_stamp, t_recv, q = [], [], []
    while r.has_next():
        _, data, t_ns = r.read_next()
        m = deserialize_message(data, JointState)
        idx = {n: i for i, n in enumerate(m.name)}
        t_stamp.append(m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec)
        t_recv.append(1e-9 * t_ns)
        q.append([m.position[idx[n]] for n in ARM])
    return np.array(t_stamp), np.array(t_recv), np.array(q)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--plot", default=None)
    a = ap.parse_args()
    ts_l, tr_l, q_l = read_joints(a.bag, "/leader/joint_states")
    ts_f, _, q_f = read_joints(a.bag, "/follower/joint_states")
    transport = np.median(tr_l - ts_l)
    lag = xcorr_lag(ts_l, q_l, ts_f, q_f)
    err = np.median(np.abs(np.stack([np.interp(ts_f, ts_l + lag, q_l[:, i]) for i in range(5)], 1) - q_f), axis=0)
    print(f"leader read -> bag (median transport): {transport * 1e3:.1f} ms")
    print(f"leader -> follower joint motion (xcorr, end to end): {lag * 1e3:.0f} +- 5 ms (100 Hz sampling)")
    print("median joint error after removing lag [deg]:", np.round(np.degrees(err), 2))
    if a.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(5, 1, sharex=True, figsize=(9, 9))
        t0 = ts_l[0]
        for i, n in enumerate(ARM):
            ax[i].plot(ts_l - t0, q_l[:, i], label="leader")
            ax[i].plot(ts_f - t0, q_f[:, i], "--", label="follower (sim)")
            ax[i].set_ylabel(f"{n}\n[rad]")
        ax[0].legend(loc="upper right")
        ax[0].set_title(f"Leader vs simulated follower, end-to-end lag {lag * 1e3:.0f} ms")
        ax[-1].set_xlabel("time [s]")
        fig.tight_layout()
        fig.savefig(a.plot, dpi=150)
        print("saved", a.plot)


if __name__ == "__main__":
    main()
