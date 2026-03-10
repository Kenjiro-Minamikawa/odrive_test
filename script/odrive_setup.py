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

# =========================
# モータ基本設定
# =========================

axis.motor.config.current_lim = 25
axis.motor.config.calibration_current = 2

axis.motor.config.pole_pairs = 21
axis.motor.config.torque_constant = 1.0

# =========================
# 速度制限
# =========================

axis.controller.config.vel_limit = 30
axis.controller.config.enable_torque_mode_vel_limit = True

# =========================
# PIDゲイン
# =========================

axis.controller.config.pos_gain = 20.0
axis.controller.config.vel_gain = 0.16
axis.controller.config.vel_integrator_gain = 0.32

# =========================
# 制御モード
# =========================

axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
axis.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

# =========================
# CAN設定
# =========================

odrv0.config.enable_can_a = True
odrv0.can.config.baud_rate = 500000
axis.config.can.node_id = args.can_id

# =========================
# ウォッチドッグ設定
# =========================
axis.config.enable_watchdog = True
axis.config.watchdog_timeout = 0.5

# =========================
# モータキャリブレーション
# =========================

print("Motor calibration...")
axis.requested_state = AXIS_STATE_MOTOR_CALIBRATION
while axis.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

dump_errors(odrv0)

axis.motor.config.pre_calibrated = True

# =========================
# エンコーダキャリブレーション
# =========================

print("Encoder calibration...")
axis.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
while axis.current_state != AXIS_STATE_IDLE:
    time.sleep(0.1)

dump_errors(odrv0)

axis.encoder.config.pre_calibrated = True

# =========================
# 設定保存
# =========================

print("Saving configuration...")
odrv0.save_configuration()

print("Setup completed.")