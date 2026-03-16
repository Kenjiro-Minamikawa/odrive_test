import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--can_id", type=int, required=True)
args = parser.parse_args()

print("Finding ODrive...")
odrv0 = odrive.find_any()
axis = odrv0.axis0

# 既存エラーをクリア
odrv0.clear_errors()

# setup中は watchdog 無効
axis.config.enable_watchdog = False

# 基本設定
axis.motor.config.current_lim = 25
axis.controller.config.vel_limit = 30
axis.controller.config.enable_torque_mode_vel_limit = True
axis.motor.config.calibration_current = 5

axis.motor.config.pole_pairs = 21
axis.motor.config.torque_constant = 1.0

axis.motor.motor_thermistor.config.enabled = True
axis.motor.motor_thermistor.config.temp_limit_lower = 20
axis.motor.motor_thermistor.config.temp_limit_upper = 100

axis.motor.fet_thermistor.config.enabled = True
axis.motor.fet_thermistor.config.temp_limit_lower = 20
axis.motor.fet_thermistor.config.temp_limit_upper = 100

axis.controller.config.pos_gain = 20.0
axis.controller.config.vel_gain = 0.16
axis.controller.config.vel_integrator_gain = 0.32

axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

odrv0.config.enable_can_a = True
odrv0.can.config.baud_rate = 500000
axis.config.can.node_id = args.can_id
odrv0.can.config.enable_r120 = False

# モータキャリブレーション
print("Motor calibration...")
odrv0.clear_errors()
axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION

# for i in range(5000):
#     print(i, ":", axis.current_state, AXIS_STATE_IDLE)
#     time.sleep(0.001)
time.sleep(5)

while axis.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)
# time.sleep(10)
time.sleep(0.1)
dump_errors(odrv0)

# # エラーがあるなら止める
# # if axis.active_errors != 0:
# #     raise RuntimeError(f"Motor calibration failed: active_errors={axis.active_errors}")

axis.motor.config.pre_calibrated = True

# # エンコーダキャリブレーション
# print("Encoder calibration...")
odrv0.clear_errors()
axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
time.sleep(6)

while axis.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

dump_errors(odrv0)

# # if axis.active_errors != 0:
# #     raise RuntimeError(f"Encoder calibration failed: active_errors={axis.active_errors}")

axis.encoder.config.pre_calibrated = True

print("Saving configuration...")
try:
    odrv0.save_configuration()
except Exception as e:
    print(f"ODrive likely rebooted after save: {e}")

print("Setup completed.")