import can

bus = can.interface.Bus("can0", interface="socketcan")

msg = can.Message(
    arbitration_id=0x7E6,
    is_extended_id=False,
    is_remote_frame=True
)

bus.send(msg)
