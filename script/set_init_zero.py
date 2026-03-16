import can
import struct
import time

INTERFACE = "socketcan"
CHANNEL = "can0"
NODE_IDS = set(range(12))   # 0..11
# NODE_IDS = {0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11}

PRINT_INTERVAL = 0.2
CAPTURE_TIMEOUT = 3.0
CLOSED_LOOP_TIMEOUT = 3.0
MAX_DRAIN = 200

# True: 1台でも初期位置が取れなければ全体を止める
REQUIRE_ALL_NODES = True

# ODrive CANSimple command IDs
CMD_HEARTBEAT = 0x01
CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_CONTROLLER_MODE = 0x0B
CMD_SET_INPUT_POS = 0x0C

# ODrive AxisState
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

# ODrive ControlMode / InputMode
CONTROL_MODE_POSITION_CONTROL = 3
INPUT_MODE_PASSTHROUGH = 1

bus = can.interface.Bus(CHANNEL, interface=INTERFACE)

# 古い受信データを捨てる
while bus.recv(timeout=0) is not None:
    pass

latest = {
    node_id: {
        "pos_rev": None,
        "vel_rev_s": None,
        "last_update": None,              # 最後に何か受信した時刻
        "last_encoder_update": None,      # 最後に encoder estimate を受信した時刻
        "last_heartbeat_update": None,    # 最後に heartbeat を受信した時刻
        "axis_error": None,
        "axis_state": None,
        "initial_pos_rev": None,          # 起動時に最初に取れた位置
        "target_pos_rev": None,           # 送った input_pos
        "prepared": False,                # 初期位置取得 + input_pos送信済み
    }
    for node_id in NODE_IDS
}


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
    # ODrive CANSimple の Set_Input_Pos は
    # Input_Pos: float32
    # Vel_FF: int16
    # Torque_FF: int16
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

    # 0x09: Get_Encoder_Estimates
    if cmd_id == CMD_GET_ENCODER_ESTIMATES and len(msg.data) >= 8:
        pos_rev, vel_rev_s = struct.unpack('<ff', bytes(msg.data[:8]))
        latest[node_id]["pos_rev"] = pos_rev
        latest[node_id]["vel_rev_s"] = vel_rev_s
        latest[node_id]["last_update"] = now
        latest[node_id]["last_encoder_update"] = now

        if latest[node_id]["initial_pos_rev"] is None:
            latest[node_id]["initial_pos_rev"] = pos_rev

    # 0x01: Heartbeat
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

def reboot_node(node_id: int):
    arb_id = (node_id << 5) | 0x16
    msg = can.Message(
        arbitration_id=arb_id,
        data=[],
        is_extended_id=False,
    )
    bus.send(msg)
    print(f"Reboot sent to node {node_id}")


def wait_for_initial_positions(timeout_sec: float = CAPTURE_TIMEOUT) -> set[int]:
    print("Waiting until all nodes are confirmed in the same round...")

    attempt = 0

    while True:
        attempt += 1
        print(f"Capture attempt #{attempt}")

        # 毎回全 node を再確認するため、ラウンド開始時にリセット
        for nid in NODE_IDS:
            latest[nid]["initial_pos_rev"] = None

        seen_this_round = set()
        deadline = time.monotonic() + timeout_sec

        while time.monotonic() < deadline and seen_this_round != NODE_IDS:
            msg = bus.recv(timeout=0.05)
            if msg is not None:
                parse_message(msg)
                drain_rx_buffer()

                # parse_message 後に、今回ラウンドで取得できた node を記録
                for nid in NODE_IDS:
                    if latest[nid]["initial_pos_rev"] is not None:
                        seen_this_round.add(nid)

        missing = sorted(NODE_IDS - seen_this_round)

        if not missing:
            print("All initial positions captured in this round.")
            return seen_this_round

        print(f"このラウンドで取得できなかった node: {missing}")
        print(f"missing node を reboot します: {missing}")

        for nid in missing:
            reboot_node(nid)

        # reboot 後の復帰待ち
        time.sleep(2.0)

        # 溜まっている古いフレームを掃除
        drain_rx_buffer()

# def wait_for_initial_positions(timeout_sec: float = CAPTURE_TIMEOUT) -> set[int]:
#     print("Waiting for initial encoder estimates before entering closed loop...")

#     deadline = time.monotonic() + timeout_sec
#     remaining = {nid for nid in NODE_IDS if latest[nid]["initial_pos_rev"] is None}

#     while time.monotonic() < deadline and remaining:
#         msg = bus.recv(timeout=0.05)
#         if msg is not None:
#             parse_message(msg)
#             drain_rx_buffer()

#         remaining = {nid for nid in NODE_IDS if latest[nid]["initial_pos_rev"] is None}

#     captured = {nid for nid in NODE_IDS if latest[nid]["initial_pos_rev"] is not None}
#     missing = sorted(NODE_IDS - captured)

#     if missing:
#         print(f"初期位置を取得できなかった node: {missing}")
#     else:
#         print("All initial positions captured.")

#     return captured


def prepare_targets_from_initial_positions(target_nodes: set[int]) -> None:
    print("Sending initial pos_estimate to input_pos...")

    for node_id in sorted(target_nodes):
        init_pos = latest[node_id]["initial_pos_rev"]
        if init_pos is None:
            continue

        # 位置制御モードを先に設定
        send_set_controller_mode(
            node_id,
            control_mode=CONTROL_MODE_POSITION_CONTROL,
            input_mode=INPUT_MODE_PASSTHROUGH,
        )
        time.sleep(0.002)

        # 現在位置をそのまま目標位置にする
        send_set_input_pos(
            node_id,
            input_pos_rev=init_pos,
            vel_ff_rev_s=0.0,
            torque_ff_nm=0.0,
        )

        latest[node_id]["target_pos_rev"] = init_pos
        latest[node_id]["prepared"] = True
        time.sleep(0.002)

    print(f"Prepared nodes: {sorted(target_nodes)}")


def enter_closed_loop_only_prepared(timeout_sec: float = CLOSED_LOOP_TIMEOUT) -> None:
    targets = {nid for nid in NODE_IDS if latest[nid]["prepared"]}

    if not targets:
        print("closed loop に入れる対象ノードがありません。")
        return

    print(f"Requesting CLOSED_LOOP_CONTROL only for prepared nodes: {sorted(targets)}")

    for node_id in sorted(targets):
        send_set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.005)

    deadline = time.monotonic() + timeout_sec
    remaining = set(targets)

    while time.monotonic() < deadline and remaining:
        msg = bus.recv(timeout=0.05)
        if msg is not None:
            parse_message(msg)
            drain_rx_buffer()

        for node_id in list(remaining):
            if latest[node_id]["axis_state"] == AXIS_STATE_CLOSED_LOOP_CONTROL:
                remaining.remove(node_id)

    if remaining:
        print(f"Closed loop に入らなかった node: {sorted(remaining)}")
    else:
        print("All prepared nodes entered CLOSED_LOOP_CONTROL.")


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

    # 1. まず closed loop に入る前に位置を取る
    captured_nodes = wait_for_initial_positions(timeout_sec=CAPTURE_TIMEOUT)

    if REQUIRE_ALL_NODES:
        missing = sorted(NODE_IDS - captured_nodes)
        if missing:
            print(f"全ノードの初期位置がそろっていないので中止: {missing}")
            return

    # 2. 取れた位置を input_pos に入れる
    prepare_targets_from_initial_positions(captured_nodes)

    # 3. prepared なノードだけ closed loop に入れる
    enter_closed_loop_only_prepared(timeout_sec=CLOSED_LOOP_TIMEOUT)

    # 4. 監視ループ
    while True:
        # まず1件待つ
        msg = bus.recv(timeout=0.05)

        if msg is not None:
            parse_message(msg)

            # バッファに残っている分を一気に処理
            drain_rx_buffer()

        # 表示更新
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