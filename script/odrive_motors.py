from dataclasses import dataclass
from typing import Optional
import time


@dataclass
class MotorState:
    node_id: int
    pos_raw: Optional[float] = None
    pos_rel: Optional[float] = None
    vel: Optional[float] = None
    error: Optional[int] = None
    state: Optional[int] = None
    traj_done: Optional[int] = None
    last_heartbeat_time: Optional[float] = None
    last_encoder_time: Optional[float] = None
    zero_offset: Optional[float] = None


@dataclass
class PositionCommand:
    pos: float
    vel_ff: int = 0
    torque_ff: int = 0

import can
import struct
import time


AXIS_STATE_IDLE = 1
AXIS_STATE_CLOSED_LOOP_CONTROL = 8

CMD_HEARTBEAT = 0x01
CMD_GET_ENCODER_ESTIMATES = 0x09
CMD_SET_AXIS_STATE = 0x07
CMD_SET_INPUT_POS = 0x0C
CMD_CLEAR_ERRORS = 0x18


class ODriveMotorNode:
    def __init__(self, bus: can.Bus, node_id: int):
        self.bus = bus
        self.node_id = node_id
        self.state = MotorState(node_id=node_id)
        self.command = PositionCommand(pos=0.0)

    def make_can_id(self, cmd_id: int) -> int:
        return (self.node_id << 5) | cmd_id

    def clear_errors(self):
        self.bus.send(can.Message(
            arbitration_id=self.make_can_id(CMD_CLEAR_ERRORS),
            data=b"\x00",
            is_extended_id=False
        ))

    def set_closed_loop(self):
        self.bus.send(can.Message(
            arbitration_id=self.make_can_id(CMD_SET_AXIS_STATE),
            data=struct.pack("<I", AXIS_STATE_CLOSED_LOOP_CONTROL),
            is_extended_id=False
        ))

    def set_input_pos_absolute(self, pos: float, vel_ff: int = 0, torque_ff: int = 0):
        self.bus.send(can.Message(
            arbitration_id=self.make_can_id(CMD_SET_INPUT_POS),
            data=struct.pack("<fhh", pos, vel_ff, torque_ff),
            is_extended_id=False
        ))

    def set_input_pos_relative(self, rel_pos: float, vel_ff: int = 0, torque_ff: int = 0):
        if self.state.zero_offset is None:
            raise RuntimeError(f"node {self.node_id}: zero_offset is not set")
        abs_pos = self.state.zero_offset + rel_pos
        self.set_input_pos_absolute(abs_pos, vel_ff, torque_ff)

    def hold_current_as_zero(self):
        if self.state.pos_raw is None:
            raise RuntimeError(f"node {self.node_id}: encoder value is not available")
        self.state.zero_offset = self.state.pos_raw

    def update_from_message(self, msg: can.Message):
        cmd_id = msg.arbitration_id & 0x1F
        node_id = msg.arbitration_id >> 5
        if node_id != self.node_id:
            return

        now = time.monotonic()

        if cmd_id == CMD_HEARTBEAT and len(msg.data) >= 7:
            error, state, result, traj_done = struct.unpack("<IBBB", bytes(msg.data[:7]))
            self.state.error = error
            self.state.state = state
            self.state.traj_done = traj_done
            self.state.last_heartbeat_time = now

        elif cmd_id == CMD_GET_ENCODER_ESTIMATES and len(msg.data) >= 8:
            pos, vel = struct.unpack("<ff", bytes(msg.data[:8]))
            self.state.pos_raw = pos
            self.state.vel = vel
            self.state.last_encoder_time = now
            if self.state.zero_offset is not None:
                self.state.pos_rel = pos - self.state.zero_offset

    def is_alive(self, timeout: float = 0.5) -> bool:
        if self.state.last_heartbeat_time is None:
            return False
        return (time.monotonic() - self.state.last_heartbeat_time) < timeout

    def is_closed_loop(self) -> bool:
        return self.state.state == AXIS_STATE_CLOSED_LOOP_CONTROL

    def has_error(self) -> bool:
        return self.state.error not in (None, 0)


class ODriveFleet:
    def __init__(self, bus: can.Bus, node_ids: list[int]):
        self.bus = bus
        self.nodes = {node_id: ODriveMotorNode(bus, node_id) for node_id in node_ids}

    def get_node(self, node_id: int) -> ODriveMotorNode:
        return self.nodes[node_id]

    def handle_message(self, msg: can.Message):
        node_id = msg.arbitration_id >> 5
        node = self.nodes.get(node_id)
        if node is not None:
            node.update_from_message(msg)

    def clear_errors_all(self):
        for node in self.nodes.values():
            node.clear_errors()

    def set_closed_loop_all(self):
        for node in self.nodes.values():
            node.set_closed_loop()

    def hold_current_as_zero_all(self):
        for node in self.nodes.values():
            node.hold_current_as_zero()

    def send_relative_positions(self, target_dict: dict[int, float]):
        for node_id, rel_pos in target_dict.items():
            self.nodes[node_id].set_input_pos_relative(rel_pos)

    def send_same_relative_position(self, rel_pos: float):
        for node in self.nodes.values():
            node.set_input_pos_relative(rel_pos)

    def all_closed_loop(self) -> bool:
        return all(node.is_closed_loop() for node in self.nodes.values())

    def any_error(self) -> bool:
        return any(node.has_error() for node in self.nodes.values())

    def summary(self) -> str:
        lines = []
        for node_id in sorted(self.nodes):
            s = self.nodes[node_id].state
            pos_rel = "None" if s.pos_rel is None else f"{s.pos_rel:.3f}"
            vel = "None" if s.vel is None else f"{s.vel:.3f}"
            state = "None" if s.state is None else str(s.state)
            error = "None" if s.error is None else str(s.error)
            lines.append(
                f"node {node_id:2d} | rel_pos={pos_rel:>8} | vel={vel:>8} | state={state:>4} | error={error}"
            )
        return "\n".join(lines)