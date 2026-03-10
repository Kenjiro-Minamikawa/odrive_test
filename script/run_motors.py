import can
import time
from odrive_motors import ODriveFleet

bus = can.interface.Bus("can0", interface="socketcan", bitrate=500000)
fleet = ODriveFleet(bus, list(range(12)))

# 古い受信を捨てる
while bus.recv(timeout=0) is not None:
    pass

# エラークリア
fleet.clear_errors_all()

# closed loopへ
fleet.set_closed_loop_all()

# 少し受信して状態更新
t0 = time.monotonic()
while time.monotonic() - t0 < 2.0:
    msg = bus.recv(timeout=0.01)
    if msg is not None:
        fleet.handle_message(msg)

# 初期位置をゼロとして保持
fleet.hold_current_as_zero_all()

# 全台に +1.5 turn を送り続ける
try:
    while True:
        fleet.send_same_relative_position(0.1)

        msg = bus.recv(timeout=0.001)
        if msg is not None:
            fleet.handle_message(msg)

        print(fleet.summary())
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    bus.shutdown()