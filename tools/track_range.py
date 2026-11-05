"""Live min/max tracker for SO-101 servo ticks (leader or follower). No ROS needed.

Usage: python3 track_range.py /dev/ttyACM1
Move every joint to both stops. Finish by holding the L pose and pressing Ctrl-C:
the last reading is printed as the zero pose.
"""
import sys
import time

import scservo_sdk as sdk

NAMES = ['pan', 'lift', 'elbow', 'wrist_flex', 'wrist_roll', 'gripper']
ADDR_TORQUE_ENABLE, ADDR_PRESENT_POS = 40, 56


def update(lo, hi, ticks):
    return [min(a, t) for a, t in zip(lo, ticks)], [max(b, t) for b, t in zip(hi, ticks)]


def table(lo, hi, cur):
    rows = [f'{"joint":<11}{"now":>6}{"min":>6}{"max":>6}']
    rows += [f'{n:<11}{c:>6}{a:>6}{b:>6}' for n, c, a, b in zip(NAMES, cur, lo, hi)]
    return '\n'.join(rows)


def main():
    port_name = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM1'
    port, pkt = sdk.PortHandler(port_name), sdk.PacketHandler(0)
    if not (port.openPort() and port.setBaudRate(1_000_000)):
        sys.exit(f'cannot open {port_name}')
    for i in range(1, 7):
        pkt.write1ByteTxRx(port, i, ADDR_TORQUE_ENABLE, 0)  # limp, so you can move it
    lo, hi, cur = [4095] * 6, [0] * 6, None
    try:
        while True:
            # Read servos one by one: a servo's error flag (e.g. voltage) doesn't block the others.
            reads = [pkt.read2ByteTxRx(port, i, ADDR_PRESENT_POS) for i in range(1, 7)]
            missing = [i for i, (_, res, _) in enumerate(reads, 1) if res != sdk.COMM_SUCCESS]
            if missing:
                print(f'\rno reply from servo(s) {missing} -- powered? right port?', end='', flush=True)
            else:
                cur = [val for val, _, _ in reads]
                lo, hi = update(lo, hi, cur)
                print('\033[H\033[J' + table(lo, hi, cur) + '\n\nCtrl-C to finish (hold the L pose)')
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        port.closePort()
    if cur is None:
        sys.exit('no readings')
    print('\n' + table(lo, hi, cur))
    print('\nL pose (last reading):', cur)


if __name__ == '__main__':
    main()