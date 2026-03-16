import can
import struct
import time

INTERFACE = "socketcan"
CHANNEL = "can0"
NODE_IDS = set(range(12))   # 0..11
PRINT_INTERVAL = 0.2        # 表示更新周期 [s]
ENTER_CLOSED_LOOP = True
CLOSED_LOOP_TIMEOUT = 10.0   # 各ノードが closed loop に入るまで待つ秒数

# ODrive CANSimple command IDs
CMD_HEARTBEAT = 0x01
CMD_SET_AXIS_STATE = 0x07
CMD_GET_ENCODER_ESTIMATES = 0x09

# ODrive AxisState
AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

bus = can.interface.Bus(CHANNEL, interface=INTERFACE)

# 古い受信データを捨てる
while bus.recv(timeout=0) is not None:
    pass

latest = {
    node_id: {
        "pos_rev": None,
        "vel_rev_s": None,
        "last_update": None,
        "axis_error": None,
        "axis_state": None,
    }
    for node_id in NODE_IDS
}

def send_set_axis_state(node_id: int, requested_state: int) -> None:
    bus.send(can.Message(
        arbitration_id=((node_id << 5) | CMD_SET_AXIS_STATE),
        data=struct.pack('<I', requested_state),
        is_extended_id=False
    ))

def parse_message(msg: can.Message):
    node_id = msg.arbitration_id >> 5
    cmd_id = msg.arbitration_id & 0x1F

    if node_id not in NODE_IDS:
        return

    # 0x09: Get_Encoder_Estimates
    if cmd_id == CMD_GET_ENCODER_ESTIMATES and len(msg.data) >= 8:
        pos_rev, vel_rev_s = struct.unpack('<ff', bytes(msg.data[:8]))
        latest[node_id]["pos_rev"] = pos_rev
        latest[node_id]["vel_rev_s"] = vel_rev_s
        latest[node_id]["last_update"] = time.monotonic()

    # 0x01: Heartbeat
    elif cmd_id == CMD_HEARTBEAT and len(msg.data) >= 7:
        axis_error, axis_state, procedure_result, traj_done = struct.unpack(
            '<IBBB', bytes(msg.data[:7])
        )
        latest[node_id]["axis_error"] = axis_error
        latest[node_id]["axis_state"] = axis_state
        latest[node_id]["last_update"] = time.monotonic()

def enter_closed_loop_for_all(timeout_sec: float = 3.0):
    print("Requesting CLOSED_LOOP_CONTROL for all nodes...")

    # 全ノードに closed loop 要求
    for node_id in sorted(NODE_IDS):
        send_set_axis_state(node_id, AXIS_STATE_CLOSED_LOOP_CONTROL)
        time.sleep(0.005)  # 少し間隔を空ける

    # state=8 を待つ
    deadline = time.monotonic() + timeout_sec
    remaining = set(NODE_IDS)

    while time.monotonic() < deadline and remaining:
        msg = bus.recv(timeout=0.05)
        if msg is None:
            continue

        parse_message(msg)

        node_id = msg.arbitration_id >> 5
        cmd_id = msg.arbitration_id & 0x1F

        if node_id in remaining and cmd_id == CMD_HEARTBEAT:
            if latest[node_id]["axis_state"] == AXIS_STATE_CLOSED_LOOP_CONTROL:
                remaining.remove(node_id)

    if remaining:
        print(f"Closed loop に入らなかった node: {sorted(remaining)}")
    else:
        print("All nodes entered CLOSED_LOOP_CONTROL.")

def print_table():
    print("\n" + "=" * 78)
    print(f"{'node':>4} | {'pos [rev]':>12} | {'vel [rev/s]':>12} | {'state':>5} | {'error':>10} | {'age [s]':>8}")
    print("-" * 78)

    now = time.monotonic()
    for node_id in sorted(NODE_IDS):
        item = latest[node_id]

        pos_str = f"{item['pos_rev']:.4f}" if item["pos_rev"] is not None else "---"
        vel_str = f"{item['vel_rev_s']:.4f}" if item["vel_rev_s"] is not None else "---"
        state_str = str(item["axis_state"]) if item["axis_state"] is not None else "---"
        error_str = hex(item["axis_error"]) if item["axis_error"] is not None else "---"
        age_str = f"{now - item['last_update']:.2f}" if item["last_update"] is not None else "---"

        print(f"{node_id:>4} | {pos_str:>12} | {vel_str:>12} | {state_str:>5} | {error_str:>10} | {age_str:>8}")

last_print = 0.0

try:
    if ENTER_CLOSED_LOOP:
        enter_closed_loop_for_all(timeout_sec=CLOSED_LOOP_TIMEOUT)

    while True:
        msg = bus.recv(timeout=0.05)
        if msg is not None:
            parse_message(msg)

        now = time.monotonic()
        if now - last_print >= PRINT_INTERVAL:
            print_table()
            last_print = now

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    bus.shutdown()