#!/usr/bin/env python3
"""Pretends to be a bus full of ODESCs (ODrive 0.5.6 CANSimple), for testing
rover_hardware/OdescSystem without real motor controllers.

Each fake axis answers Set_Axis_State / Set_Controller_Mode / Set_Input_Pos /
Set_Input_Vel / Clear_Errors, moves toward its target (crudely -- this is
not a physics sim), and broadcasts Heartbeat every 100 ms and
Get_Encoder_Estimates every 10 ms, same as a real ODESC's defaults.

Setup (Linux, needs the vcan kernel module -- usually not in WSL2):
    sudo modprobe vcan
    sudo ip link add dev vcan0 type vcan
    sudo ip link set up vcan0

Run:
    ros2 run rover_hardware fake_odesc.py --channel vcan0
    ros2 launch rover_bringup rover.launch.py can_interface:=vcan0

Watch the raw traffic with `candump vcan0` (from can-utils).
"""

import argparse
import struct
import time

import can

HEARTBEAT = 0x001
SET_AXIS_STATE = 0x007
ENCODER_ESTIMATES = 0x009
SET_CONTROLLER_MODE = 0x00B
SET_INPUT_POS = 0x00C
SET_INPUT_VEL = 0x00D
CLEAR_ERRORS = 0x018

STATE_IDLE = 1
STATE_CLOSED_LOOP = 8
CONTROL_VELOCITY = 2
CONTROL_POSITION = 3

# ODrive 0.5.x AXIS_ERROR_WATCHDOG_TIMER_EXPIRED
AXIS_ERROR_WATCHDOG = 0x800

# How fast a fake position-controlled axis can slew toward its target.
MAX_SLEW_TURNS_PER_S = 5.0


class FakeAxis:
    def __init__(self, node_id):
        self.node_id = node_id
        self.state = STATE_IDLE
        self.axis_error = 0
        self.control_mode = CONTROL_POSITION
        self.pos = 0.0
        self.vel = 0.0
        self.input_pos = 0.0
        self.input_vel = 0.0
        self.last_input = time.monotonic()

    def handle(self, cmd_id, data):
        if cmd_id == SET_AXIS_STATE:
            (requested,) = struct.unpack_from("<I", data)
            if requested == STATE_CLOSED_LOOP and self.axis_error:
                return  # a real ODrive refuses to arm with an active error
            self.state = requested
            self.last_input = time.monotonic()
        elif cmd_id == SET_CONTROLLER_MODE:
            self.control_mode, _input_mode = struct.unpack_from("<ii", data)
        elif cmd_id == SET_INPUT_POS:
            self.input_pos, _vel_ff, _torque_ff = struct.unpack_from("<fhh", data)
            self.last_input = time.monotonic()
        elif cmd_id == SET_INPUT_VEL:
            self.input_vel, _torque_ff = struct.unpack_from("<ff", data)
            self.last_input = time.monotonic()
        elif cmd_id == CLEAR_ERRORS:
            self.axis_error = 0

    def step(self, dt, watchdog_s):
        if self.state == STATE_CLOSED_LOOP and watchdog_s > 0:
            if time.monotonic() - self.last_input > watchdog_s:
                self.state = STATE_IDLE
                self.axis_error |= AXIS_ERROR_WATCHDOG
                print(f"node {self.node_id}: watchdog expired -> IDLE")

        if self.state != STATE_CLOSED_LOOP:
            self.vel = 0.0
        elif self.control_mode == CONTROL_VELOCITY:
            self.vel = self.input_vel
        else:
            err = self.input_pos - self.pos
            max_step = MAX_SLEW_TURNS_PER_S * dt
            step = max(-max_step, min(max_step, err))
            self.vel = step / dt if dt > 0 else 0.0
        self.pos += self.vel * dt


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--channel", default="vcan0")
    parser.add_argument("--nodes", type=int, default=8, help="node IDs 0..N-1")
    parser.add_argument(
        "--watchdog-ms", type=int, default=0,
        help="simulate the ODESC watchdog: go IDLE if no command arrives for this long (0 = off)")
    args = parser.parse_args()

    bus = can.Bus(interface="socketcan", channel=args.channel)
    axes = {n: FakeAxis(n) for n in range(args.nodes)}
    print(f"Faking {args.nodes} ODESCs (node IDs 0-{args.nodes - 1}) on {args.channel}")

    last_step = last_encoder = last_heartbeat = time.monotonic()
    while True:
        msg = bus.recv(timeout=0.001)
        if msg is not None and not msg.is_extended_id:
            node_id, cmd_id = msg.arbitration_id >> 5, msg.arbitration_id & 0x1F
            if node_id in axes:
                axes[node_id].handle(cmd_id, bytes(msg.data))

        now = time.monotonic()
        dt, last_step = now - last_step, now
        for axis in axes.values():
            axis.step(dt, args.watchdog_ms / 1000.0)

        if now - last_encoder >= 0.010:
            last_encoder = now
            for a in axes.values():
                bus.send(can.Message(
                    arbitration_id=(a.node_id << 5) | ENCODER_ESTIMATES,
                    data=struct.pack("<ff", a.pos, a.vel), is_extended_id=False))
        if now - last_heartbeat >= 0.100:
            last_heartbeat = now
            for a in axes.values():
                bus.send(can.Message(
                    arbitration_id=(a.node_id << 5) | HEARTBEAT,
                    data=struct.pack("<IBBBB", a.axis_error, a.state, 0, 0, 0), is_extended_id=False))


if __name__ == "__main__":
    main()
