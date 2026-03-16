import can
import struct
import time

INTERFACE = "socketcan"
CHANNEL = "can0"
NODE_IDS = set(range(12))   # 0..11

PRINT_INTERVAL = 0.2
CAPTURE_TIMEOUT = 3.0
CLOSED_LOOP_TIMEOUT = 3.0
MAX_DRAIN = 200

REQUIRE_ALL_NODES = True

ENCODER_FRESH_SEC = 0.3
HEARTBEAT_FRESH_SEC = 0.5
REBOOT_SETTLE_SEC = 2.0
POST_CLOSED_LOOP_WAIT_SEC = 0.3

# ODrive CANSimple command IDs
CMD_HEARTBEAT = 0x01
CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_CONTROLLER_MODE = 0x0B
CMD_SET_INPUT_POS = 0x0C
CMD_REBOOT = 0x16
CMD_SET_POS_GAIN = 0x1A
CMD_SET_VEL_GAINS = 0x1B


# ODrive AxisState
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# ODrive ControlMode / InputMode
CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_PASSTHROUGH = 1

RAMP_DT_SEC = 0.05
RAMP_DURATION_SEC = 5.0

init_poss = [-7.2356, -0.2882, -4.9643, -7.2440, 
             -4.3099, -4.193, -5.2545, -3.2615,
             -7.5931, -2.6076, -7.9566, -0.4975]

bus = can.interface.Bus(CHANNEL, interface=INTERFACE)

# 古い受信データを捨てる
while bus.recv(timeout=0) is not None:
    pass

latest = {
    node_id: {
        "pos_rev": None,
        "vel_rev_s": None,
        "last_update": None,
        "last_encoder_update": None,
        "last_heartbeat_update": None,
        "axis_error": None,
        "axis_state": None,
        "initial_pos_rev": None,   # target決定時点の基準位置
        "target_pos_rev": None,
        "prepared": False,
    }
    for node_id in NODE_IDS
}

def get_offset_rev(node_id: int) -> float:
    if node_id == 4:
        return +0.57
    elif node_id == 5:
        return -0.57
    elif node_id == 6:
        return +0.57
    elif node_id == 7:
        return -0.57
    elif node_id == 8:
        return -1.8
    elif node_id == 9:
        return +1.8
    elif node_id == 10:
        return -1.8
    elif node_id == 11:
        return +1.8
    # elif node_id == 0:
    #     return +0.15
    # elif node_id == 1:
    #     return -0.15
    # elif node_id == 2:
    #     return -0.15
    # elif node_id == 3:
    #     return +0.15
    return 0.0

def send_set_axis_state(node_id: int, requested_state: int) -> None:
    bus.send(
        can.Message(
            arbitration_id=((node_id << 5) | CMD_SET_AXIS_STATE),
            data=struct.pack('<I', requested_state),
            is_extended_id=False,
        )
    )


def send_set_controller_mode(
    node_id: int,
    control_mode: int = CONTROL_MODE_POSITION_CONTROL,
    input_mode: int = INPUT_MODE_PASSTHROUGH,
) -> None:
    bus.send(
        can.Message(
            arbitration_id=((node_id << 5) | CMD_SET_CONTROLLER_MODE),
            data=struct.pack('<II', control_mode, input_mode),
            is_extended_id=False,
        )
    )


def send_set_input_pos(
    node_id: int,
    input_pos_rev: float,
    vel_ff_rev_s: float = 0.0,
    torque_ff_nm: float = 0.0,
) -> None:
    vel_ff_i16 = int(round(vel_ff_rev_s * 1000.0))
    torque_ff_i16 = int(round(torque_ff_nm * 1000.0))

    vel_ff_i16 = max(-32768, min(32767, vel_ff_i16))
    torque_ff_i16 = max(-32768, min(32767, torque_ff_i16))

    bus.send(
        can.Message(
            arbitration_id=((node_id << 5) | CMD_SET_INPUT_POS),
            data=struct.pack('<fhh', input_pos_rev, vel_ff_i16, torque_ff_i16),
            is_extended_id=False,
        )
    )


def parse_message(msg: can.Message) -> None:
    node_id = msg.arbitration_id >> 5
    cmd_id = msg.arbitration_id & 0x1F

    if node_id not in NODE_IDS:
        return

    now = time.monotonic()

    if cmd_id == CMD_GET_ENCODER_ESTIMATES and len(msg.data) >= 8:
        pos_rev, vel_rev_s = struct.unpack('<ff', bytes(msg.data[:8]))
        latest[node_id]["pos_rev"] = pos_rev
        latest[node_id]["vel_rev_s"] = vel_rev_s
        latest[node_id]["last_update"] = now
        latest[node_id]["last_encoder_update"] = now

    elif cmd_id == CMD_HEARTBEAT and len(msg.data) >= 7:
        axis_error, axis_state, procedure_result, traj_done = struct.unpack(
            '<IBBB', bytes(msg.data[:7])
        )
        latest[node_id]["axis_error"] = axis_error
        latest[node_id]["axis_state"] = axis_state
        latest[node_id]["last_update"] = now
        latest[node_id]["last_heartbeat_update"] = now


def drain_rx_buffer(max_drain: int = MAX_DRAIN) -> int:
    drained = 0
    while drained < max_drain:
        msg = bus.recv(timeout=0.0)
        if msg is None:
            break
        parse_message(msg)
        drained += 1
    return drained


def reboot_node(node_id: int) -> None:
    bus.send(
        can.Message(
            arbitration_id=((node_id << 5) | CMD_REBOOT),
            data=[],
            is_extended_id=False,
        )
    )
    print(f"Reboot sent to node {node_id}")


def has_fresh_encoder(node_id: int, now: float, fresh_sec: float = ENCODER_FRESH_SEC) -> bool:
    t = latest[node_id]["last_encoder_update"]
    return (t is not None) and ((now - t) <= fresh_sec)


def has_fresh_heartbeat(node_id: int, now: float, fresh_sec: float = HEARTBEAT_FRESH_SEC) -> bool:
    t = latest[node_id]["last_heartbeat_update"]
    return (t is not None) and ((now - t) <= fresh_sec)


def wait_until_nodes_ready(timeout_sec: float = CAPTURE_TIMEOUT) -> set[int]:
    print("Waiting for fresh encoder + heartbeat from all nodes...")

    attempt = 0

    while True:
        attempt += 1
        print(f"Ready check attempt #{attempt}")

        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline:
            msg = bus.recv(timeout=0.05)
            if msg is not None:
                parse_message(msg)
                drain_rx_buffer()

            now = time.monotonic()
            ready_nodes = {
                nid for nid in NODE_IDS
                if has_fresh_encoder(nid, now) and has_fresh_heartbeat(nid, now)
            }

            if ready_nodes == NODE_IDS:
                print("All nodes are ready.")
                return ready_nodes

        now = time.monotonic()
        missing = sorted(
            nid for nid in NODE_IDS
            if not (has_fresh_encoder(nid, now) and has_fresh_heartbeat(nid, now))
        )

        print(f"ready でない node: {missing}")
        print(f"missing node を reboot します: {missing}")

        for nid in missing:
            reboot_node(nid)

        time.sleep(REBOOT_SETTLE_SEC)
        drain_rx_buffer()


def configure_controller_mode(nodes: set[int]) -> None:
    print(f"Setting controller mode for nodes: {sorted(nodes)}")

    for node_id in sorted(nodes):
        send_set_controller_mode(
            node_id,
            control_mode=CONTROL_MODE_POSITION_CONTROL,
            input_mode=INPUT_MODE_PASSTHROUGH,
        )
        time.sleep(0.002)
        send_set_pos_gain(
            node_id,
            pos_gain=200.0
        )
        time.sleep(0.002)
        send_set_vel_gains(
            node_id,
            vel_gain=0.16,
            # vel_gain=0.3,
            vel_integrator_gain=0.0
        )

def send_set_pos_gain(node_id: int, pos_gain: float) -> None:
    bus.send(
        can.Message(
            arbitration_id=((node_id << 5) | CMD_SET_POS_GAIN),
            data=struct.pack('<f', pos_gain),
            is_extended_id=False,
        )
    )

def send_set_vel_gains(node_id: int, vel_gain: float, vel_integrator_gain: float) -> None:
    bus.send(
        can.Message(
            arbitration_id=((node_id << 5) | CMD_SET_VEL_GAINS),
            data=struct.pack('<ff', vel_gain, vel_integrator_gain),
            is_extended_id=False,
        )
    )


def enter_closed_loop(nodes: set[int], timeout_sec: float = CLOSED_LOOP_TIMEOUT) -> set[int]:
    if not nodes:
        print("closed loop に入れる対象ノードがありません。")
        return set()

    print(f"Requesting CLOSED_LOOP_CONTROL for nodes: {sorted(nodes)}")

    for node_id in sorted(nodes):
        send_set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.005)

    deadline = time.monotonic() + timeout_sec
    remaining = set(nodes)

    while time.monotonic() < deadline and remaining:
        msg = bus.recv(timeout=0.05)
        if msg is not None:
            parse_message(msg)
            drain_rx_buffer()

        for node_id in list(remaining):
            if latest[node_id]["axis_state"] == AXIS_STATE_CLOSED_LOOP_CONTROL:
                remaining.remove(node_id)

    entered = nodes - remaining

    if remaining:
        print(f"Closed loop に入らなかった node: {sorted(remaining)}")
    else:
        print("All target nodes entered CLOSED_LOOP_CONTROL.")

    return entered


def refresh_positions_after_closed_loop(nodes: set[int], wait_sec: float = POST_CLOSED_LOOP_WAIT_SEC) -> set[int]:
    print(f"Waiting {wait_sec:.2f}s after closed loop, then capturing current positions...")

    time.sleep(wait_sec)

    deadline = time.monotonic() + CAPTURE_TIMEOUT

    while time.monotonic() < deadline:
        msg = bus.recv(timeout=0.05)
        if msg is not None:
            parse_message(msg)
            drain_rx_buffer()

        now = time.monotonic()
        fresh_nodes = {
            nid for nid in nodes
            if has_fresh_encoder(nid, now)
        }
        if fresh_nodes == nodes:
            break

    now = time.monotonic()
    captured = {
        nid for nid in nodes
        if has_fresh_encoder(nid, now) and latest[nid]["pos_rev"] is not None
    }

    missing = sorted(nodes - captured)
    if missing:
        print(f"closed loop 後に現在角度を取得できなかった node: {missing}")
    else:
        print("All current positions captured after closed loop.")
        for i in range(12):
            print("node ", "i", ": ", latest[i]["pos_rev"])

    return captured


# def prepare_targets_from_current_positions(target_nodes: set[int]) -> None:
#     print("Sending current pos_estimate to input_pos...")

#     for node_id in sorted(target_nodes):
#         current_pos = latest[node_id]["pos_rev"]
#         if current_pos is None:
#             continue

#         target_pos = current_pos

#         latest[node_id]["initial_pos_rev"] = current_pos
#         latest[node_id]["target_pos_rev"] = target_pos

#         # target_pos = init_poss[node_id]

#         # latest[node_id]["initial_pos_rev"] = init_poss[node_id]
#         # latest[node_id]["target_pos_rev"] = target_pos

#         # send_set_input_pos(
#         #     node_id, 
#         #     input_pos_rev=target_pos,
#         #     vel_ff_rev_s=0.0, 
#         #     torque_ff_nm=0.0,
#         # )

#         if node_id in {4}:
#             send_set_input_pos(
#                 node_id,
#                 input_pos_rev=target_pos + 0.47,
#                 vel_ff_rev_s=0.0,
#                 torque_ff_nm=0.0,
#             )
#         elif node_id in {5}:
#             send_set_input_pos(
#                 node_id,
#                 input_pos_rev=target_pos - 0.47,
#                 vel_ff_rev_s=0.0,
#                 torque_ff_nm=0.0,
#             )
#         elif node_id in {6}:
#             send_set_input_pos(
#                 node_id,
#                 input_pos_rev=target_pos + 0.57,
#                 vel_ff_rev_s=0.0,
#                 torque_ff_nm=0.0,
#             )
#         elif node_id in {7}:
#             send_set_input_pos(
#                 node_id,
#                 input_pos_rev=target_pos - 0.57,
#                 vel_ff_rev_s=0.0,
#                 torque_ff_nm=0.0,
#             )
#         elif node_id in {8}:
#             send_set_input_pos(
#                 node_id,
#                 input_pos_rev=target_pos - 1.8,
#                 vel_ff_rev_s=0.0,
#                 torque_ff_nm=0.0,
#             )
#         elif node_id in {9}:
#             send_set_input_pos(
#                 node_id,
#                 input_pos_rev=target_pos + 1.8,
#                 vel_ff_rev_s=0.0,
#                 torque_ff_nm=0.0,
#             )
#         elif node_id in {10}:
#             send_set_input_pos(
#                 node_id,
#                 input_pos_rev=target_pos - 1.8,
#                 vel_ff_rev_s=0.0,
#                 torque_ff_nm=0.0,
#             )
#         elif node_id in {11}:
#             send_set_input_pos(
#                 node_id,
#                 input_pos_rev=target_pos + 1.8,
#                 vel_ff_rev_s=0.0,
#                 torque_ff_nm=0.0,
#             )
#         else:
#             send_set_input_pos(
#                 node_id,
#                 input_pos_rev=target_pos,
#                 vel_ff_rev_s=0.0,
#                 torque_ff_nm=0.0,
#             )

#         latest[node_id]["prepared"] = True
#         time.sleep(0.002)

#     print(f"Prepared nodes: {sorted(target_nodes)}")

def prepare_targets_from_current_positions(
    target_nodes: set[int],
    duration_sec: float = RAMP_DURATION_SEC,
    dt_sec: float = RAMP_DT_SEC,
) -> None:

    print(f"Ramping relative offsets over {duration_sec:.1f}s")

    steps = int(duration_sec / dt_sec)

    current_pos = {}
    offset = {}
    step_offset = {}

    for node_id in sorted(target_nodes):
        pos = latest[node_id]["pos_rev"]
        if pos is None:
            continue

        off = get_offset_rev(node_id)

        current_pos[node_id] = pos
        offset[node_id] = off
        step_offset[node_id] = off / steps

        latest[node_id]["initial_pos_rev"] = pos
        latest[node_id]["target_pos_rev"] = pos + off
        latest[node_id]["prepared"] = True

    if not current_pos:
        print("Prepared nodes: []")
        return

    cmd_pos = current_pos.copy()

    for _ in range(steps):

        for node_id in cmd_pos:
            cmd_pos[node_id] += step_offset[node_id]

            send_set_input_pos(
                node_id,
                input_pos_rev=cmd_pos[node_id],
                vel_ff_rev_s=0.0,
                torque_ff_nm=0.0,
            )

        # CAN受信も回す
        msg = bus.recv(timeout=0.0)
        if msg is not None:
            parse_message(msg)
            drain_rx_buffer()

        time.sleep(dt_sec)

    # 最終位置をもう一度送る
    for node_id in cmd_pos:
        send_set_input_pos(
            node_id,
            input_pos_rev=current_pos[node_id] + offset[node_id],
            vel_ff_rev_s=0.0,
            torque_ff_nm=0.0,
        )

    print(f"Prepared nodes: {sorted(cmd_pos)}")


def print_table() -> None:
    print("\n" + "=" * 132)
    print(
        f"{'node':>4} | {'init [rev]':>12} | {'pos [rev]':>12} | {'target [rev]':>12} | "
        f"{'vel [rev/s]':>12} | {'prep':>5} | {'state':>5} | {'error':>10} | {'enc_age':>8} | {'hb_age':>8}"
    )
    print("-" * 132)

    now = time.monotonic()

    for node_id in sorted(NODE_IDS):
        item = latest[node_id]

        init_str = f"{item['initial_pos_rev']:.4f}" if item["initial_pos_rev"] is not None else "---"
        pos_str = f"{item['pos_rev']:.4f}" if item["pos_rev"] is not None else "---"
        tgt_str = f"{item['target_pos_rev']:.4f}" if item["target_pos_rev"] is not None else "---"
        vel_str = f"{item['vel_rev_s']:.4f}" if item["vel_rev_s"] is not None else "---"
        prepared_str = "yes" if item["prepared"] else "no"
        state_str = str(item["axis_state"]) if item["axis_state"] is not None else "---"
        error_str = hex(item["axis_error"]) if item["axis_error"] is not None else "---"

        enc_age_str = (
            f"{now - item['last_encoder_update']:.2f}"
            if item["last_encoder_update"] is not None else "---"
        )
        hb_age_str = (
            f"{now - item['last_heartbeat_update']:.2f}"
            if item["last_heartbeat_update"] is not None else "---"
        )

        print(
            f"{node_id:>4} | {init_str:>12} | {pos_str:>12} | {tgt_str:>12} | "
            f"{vel_str:>12} | {prepared_str:>5} | {state_str:>5} | {error_str:>10} | "
            f"{enc_age_str:>8} | {hb_age_str:>8}"
        )


def main() -> None:
    last_print = 0.0

    # 1. まず全 node が生きていて fresh な heartbeat / encoder を出しているか確認
    ready_nodes = wait_until_nodes_ready(timeout_sec=CAPTURE_TIMEOUT)

    if REQUIRE_ALL_NODES:
        missing = sorted(NODE_IDS - ready_nodes)
        if missing:
            print(f"全ノードの準備がそろっていないので中止: {missing}")
            return

    # 2. controller mode 設定
    configure_controller_mode(ready_nodes)

    # 3. closed loop に入れる
    closed_nodes = enter_closed_loop(ready_nodes, timeout_sec=CLOSED_LOOP_TIMEOUT)

    if REQUIRE_ALL_NODES:
        missing = sorted(ready_nodes - closed_nodes)
        if missing:
            print(f"closed loop に入れなかったので中止: {missing}")
            return

    # 4. closed loop 後に少し待って、その時点の現在角度を取得
    captured_nodes = refresh_positions_after_closed_loop(closed_nodes)

    if REQUIRE_ALL_NODES:
        missing = sorted(closed_nodes - captured_nodes)
        if missing:
            print(f"closed loop 後の現在角度がそろっていないので中止: {missing}")
            return

    # 5. その現在角度を init / target に反映
    prepare_targets_from_current_positions(captured_nodes)

    # 6. 監視ループ
    while True:
        msg = bus.recv(timeout=0.05)
        if msg is not None:
            parse_message(msg)
            drain_rx_buffer()

        now = time.monotonic()
        if now - last_print >= PRINT_INTERVAL:
            print_table()
            last_print = now


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        bus.shutdown()