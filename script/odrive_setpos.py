### example usage:
### python odrive_setpos.py --pos 0.1
### python odrive_setpos.py --pos 0.1 --start-id 3 --end-id 7
### python odrive_setpos.py --pos 0.1 --send-period 0.02

import argparse
import can
import struct
import time
from collections import defaultdict

AXIS_STATE_CLOSED_LOOP_CONTROL = 8

CMD_HEARTBEAT = 0x01
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_AXIS_STATE = 0x07
CMD_SET_INPUT_POS = 0x0C


def make_can_id(node_id: int, cmd_id: int) -> int:
    return (node_id << 5) | cmd_id


def send_set_axis_state(bus, node_id: int, state: int):
    bus.send(can.Message(
        arbitration_id=make_can_id(node_id, CMD_SET_AXIS_STATE),
        data=struct.pack('<I', state),
        is_extended_id=False
    ))


def send_set_input_pos(bus, node_id: int, pos_turn: float, vel_ff: int = 0, torque_ff: int = 0):
    bus.send(can.Message(
        arbitration_id=make_can_id(node_id, CMD_SET_INPUT_POS),
        data=struct.pack('<fhh', pos_turn, vel_ff, torque_ff),
        is_extended_id=False
    ))


def wait_for_closed_loop(bus, node_id: int, timeout: float = 3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue

        if msg.arbitration_id == make_can_id(node_id, CMD_HEARTBEAT):
            error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
            if error != 0:
                raise RuntimeError(f"node {node_id}: heartbeat error={error}")
            if state == AXIS_STATE_CLOSED_LOOP_CONTROL:
                print(f"node {node_id}: entered CLOSED_LOOP_CONTROL")
                return

    raise TimeoutError(f"node {node_id}: failed to enter CLOSED_LOOP_CONTROL")


def wait_for_initial_encoder_zero(bus, node_ids, timeout: float = 3.0):
    zero_offsets = {}
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline and len(zero_offsets) < len(node_ids):
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue

        cmd_id = msg.arbitration_id & 0x1F
        node_id = msg.arbitration_id >> 5

        if node_id in node_ids and cmd_id == CMD_GET_ENCODER_ESTIMATES and len(msg.data) >= 8:
            pos, vel = struct.unpack('<ff', bytes(msg.data[:8]))
            if node_id not in zero_offsets:
                zero_offsets[node_id] = pos
                print(f"node {node_id}: zero offset captured = {pos:.3f} turns")

    missing = [nid for nid in node_ids if nid not in zero_offsets]
    if missing:
        raise TimeoutError(
            "failed to capture initial encoder estimate for nodes: "
            + ", ".join(map(str, missing))
            + "\nODrive側で Get_Encoder_Estimates の cyclic message が有効か確認してください。"
        )

    return zero_offsets


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--channel", type=str, default="can0")
    parser.add_argument("--interface", type=str, default="socketcan")
    parser.add_argument("--bitrate", type=int, default=500000)
    parser.add_argument("--start-id", type=int, default=0, help="開始 node_id")
    parser.add_argument("--end-id", type=int, default=11, help="終了 node_id")
    parser.add_argument("--pos", type=float, required=True, help="現在位置を0とした相対目標位置 [turn]")
    parser.add_argument("--send-period", type=float, default=0.05, help="位置指令送信周期 [s]")
    parser.add_argument("--print-period", type=float, default=0.2, help="表示周期 [s]")
    args = parser.parse_args()

    node_ids = list(range(args.start_id, args.end_id + 1))

    encoder_estimates = defaultdict(lambda: {"pos": None, "vel": None, "t": None})
    heartbeats = defaultdict(lambda: {"error": None, "state": None, "result": None, "traj_done": None, "t": None})

    bus = can.interface.Bus(
        channel=args.channel,
        interface=args.interface,
        bitrate=args.bitrate
    )

    while bus.recv(timeout=0) is not None:
        pass

    for node_id in node_ids:
        send_set_axis_state(bus, node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)

    for node_id in node_ids:
        wait_for_closed_loop(bus, node_id)

    # ここで「現在位置を0」とするためのオフセットを取得
    zero_offsets = wait_for_initial_encoder_zero(bus, node_ids, timeout=3.0)

    print("start control loop")
    print(f"relative target position = {args.pos} turns")
    print(f"nodes = {node_ids}")

    next_send_time = time.monotonic()
    next_print_time = time.monotonic() + args.print_period

    try:
        while True:
            now = time.monotonic()

            if now >= next_send_time:
                for node_id in node_ids:
                    # 現在位置を0とした相対目標 -> ODriveに送る絶対目標へ変換
                    absolute_target = zero_offsets[node_id] + args.pos
                    send_set_input_pos(bus, node_id, absolute_target, vel_ff=0, torque_ff=0)
                next_send_time += args.send_period

            msg = bus.recv(timeout=0.001)
            if msg is not None:
                cmd_id = msg.arbitration_id & 0x1F
                node_id = msg.arbitration_id >> 5

                if node_id in node_ids:
                    if cmd_id == CMD_GET_ENCODER_ESTIMATES and len(msg.data) >= 8:
                        pos, vel = struct.unpack('<ff', bytes(msg.data[:8]))
                        encoder_estimates[node_id]["pos"] = pos
                        encoder_estimates[node_id]["vel"] = vel
                        encoder_estimates[node_id]["t"] = time.monotonic()

                    elif cmd_id == CMD_HEARTBEAT and len(msg.data) >= 7:
                        error, state, result, traj_done = struct.unpack('<IBBB', bytes(msg.data[:7]))
                        heartbeats[node_id]["error"] = error
                        heartbeats[node_id]["state"] = state
                        heartbeats[node_id]["result"] = result
                        heartbeats[node_id]["traj_done"] = traj_done
                        heartbeats[node_id]["t"] = time.monotonic()

            if now >= next_print_time:
                print("-" * 100)
                print(f"time={now:.2f}, relative target={args.pos:.3f} turns (current position treated as 0)")
                for node_id in node_ids:
                    hb = heartbeats[node_id]
                    enc = encoder_estimates[node_id]

                    raw_pos = enc["pos"]
                    rel_pos = None if raw_pos is None else (raw_pos - zero_offsets[node_id])
                    vel = enc["vel"]
                    err = hb["error"]
                    state = hb["state"]

                    raw_pos_str = f"{raw_pos:.3f}" if raw_pos is not None else "None"
                    rel_pos_str = f"{rel_pos:.3f}" if rel_pos is not None else "None"
                    vel_str = f"{vel:.3f}" if vel is not None else "None"
                    err_str = str(err) if err is not None else "None"
                    state_str = str(state) if state is not None else "None"

                    print(
                        f"node {node_id:2d} | "
                        f"rel_pos={rel_pos_str:>8} turns | "
                        f"raw_pos={raw_pos_str:>8} turns | "
                        f"vel={vel_str:>8} turns/s | "
                        f"state={state_str:>4} | "
                        f"error={err_str}"
                    )

                next_print_time += args.print_period

    except KeyboardInterrupt:
        print("\nstopped by user")

    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()