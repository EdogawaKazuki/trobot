from pyvesc import VESC
import traceback
import time
import numpy as np
# import cv2
import threading

from jc import Joystick


class Robot:
    def __init__(self, steer_can_id_list, steer_wheel_directions, wheel_can_id_list, wheel_directions, base_can_id, steer_offsets):
        self.base_can_id = base_can_id
        self.offsets = steer_offsets
        self.steer_can_id_list = steer_can_id_list
        self.steer_pos_list = {can_id: 0.0 for can_id in steer_can_id_list}
        self.steer_wheel_directions = steer_wheel_directions
        self.drove_wheel_can_id_list = wheel_can_id_list
        self.wheel_directions = wheel_directions
        self.motor = None

        self.speed_list = {can_id: 0.0 for can_id in wheel_can_id_list}
        self.direction_list = {can_id: 0.0 for can_id in steer_can_id_list}

    
    def connect(self, serial_port="COM8"):
        try:
            self.motor = VESC(serial_port=serial_port)
            self.motor.__enter__()
            print("Motor connected:", serial_port)
            return True
        except Exception:
            traceback.print_exc()
            self.motor = None
            return False
    
    def _safe_float(self, value, default=0.0):
        try:
            if value is None:
                return default
            return float(value)
        except Exception:
            return default

    def _extract_position_deg(self, measured_values):
        """
        Extract position from different pyvesc message types.
        Some firmware/queries return GetValues(pid_pos_now), while others
        return GetRotorPosition(rotor_pos/value).
        """
        for attr in ("pid_pos_now", "rotor_pos", "pos", "value"):
            if hasattr(measured_values, attr):
                return self._safe_float(getattr(measured_values, attr), 0.0)
        raise AttributeError(
            f"Unsupported measurement message type: {type(measured_values).__name__}, "
            f"available attrs={dir(measured_values)}"
        )

    def _get_measurements_verified(self, can_id, retries=5):
        for _ in range(retries):
            try:
                if can_id == self.base_can_id:
                    msg = self.motor.get_measurements(can_id=None)
                else:
                    msg = self.motor.get_measurements(can_id=can_id)
            except Exception:
                traceback.print_exc()
                # time.sleep(0.003)
                continue
            # Best case: GetValues with source ID
            if hasattr(msg, "app_controller_id"):
                src = int.from_bytes(msg.app_controller_id, byteorder="big", signed=False)
                # print(src)
                if src == can_id:
                    return msg
                # local USB controller may report 0 depending firmware/config
                if can_id is None and src in (0,):
                    return msg
            # If we got a different packet type, retry
            time.sleep(0.003)
        raise RuntimeError(f"Cannot verify response source for can_id={can_id}")
    
    def _motor_pos_to_cmd_pos(self, absolute_position, can_id):
        return (absolute_position - 180 - self.offsets[can_id]) * self.steer_wheel_directions[can_id]
    def _cmd_pos_to_motor_pos(self, relative_position, can_id):
        return relative_position * self.steer_wheel_directions[can_id] + 180 + self.offsets[can_id]
    
    def _set_position(self, can_id, relative_position):
        absolute_position = self._cmd_pos_to_motor_pos(relative_position, can_id)
        if absolute_position < 0 - 180:
            absolute_position += 360
        if absolute_position > 360 + 180:
            absolute_position -= 360
        # smooth the position
        absolute_position = (self._cmd_pos_to_motor_pos(self.direction_list[can_id], can_id)) * 0.8 + absolute_position * 0.2
        # print(f"Setting position{can_id}: {absolute_position}")
        self.direction_list[can_id] = self._motor_pos_to_cmd_pos(absolute_position, can_id)
        # return
        if can_id == self.base_can_id:
            self.motor.set_pos(float(absolute_position), can_id=None)
        else:
            self.motor.set_pos(float(absolute_position), can_id=can_id)
    def _set_speed(self, can_id, speed):
        control_by_rpm = False
        control_by_current = not control_by_rpm
        if control_by_rpm:
            rpm_gain = 14
            if can_id == self.base_can_id:
                self.motor.set_rpm(speed * self.wheel_directions[can_id] * rpm_gain, can_id=None)
            else:
                self.motor.set_rpm(speed * self.wheel_directions[can_id] * rpm_gain, can_id=can_id)
        elif control_by_current:
            current_gain = 2
            if can_id == self.base_can_id:
                self.motor.set_current(speed * self.wheel_directions[can_id] * current_gain, can_id=None)
            else:
                self.motor.set_current(speed * self.wheel_directions[can_id] * current_gain, can_id=can_id)

    def _control_loop(self):
        while True:
            try:
                for i, can_id in enumerate(self.steer_can_id_list):
                    pos_read = self._extract_position_deg(self._get_measurements_verified(can_id=can_id))
                    # pos_rad = np.radians(pos_relative)
                    self.steer_pos_list[can_id] = self._motor_pos_to_cmd_pos(pos_read, can_id)
            except Exception:
                traceback.print_exc()
                continue
            time.sleep(0.01)
    
    def run(self):
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()


if __name__ == "__main__":

    # 76: right -> 0
    # 116: left -> 13
    # 108: rear -> 84
    steer_offsets = {
        # 108: 0.0,
        # 76: 25.0,
        # 116: 115.0,
        108: 0.0, 
        76: 0.0,
        116: 0.0,

    }
    base_can_id = 108
    steer_wheel_can_id_list = [108, 76, 116]
    drive_wheel_can_id_list = [84, 0, 13]
    steer_wheel_directions = {
        108: -1,
        76: -1,
        116: -1,
    }
    drive_wheel_directions = {
        84: -1,
        0: 1,
        13: 1,
    }
    steer_rotation_mode_pos = {
        108: -90.0,
        76: -30.0,
        116: 30.0,
    }
    steer_wheel_mount_angle = {
        108: np.radians(0.0),
        76: np.radians(120.0),
        116: np.radians(240.0),
    }
    steer_wheel_distance = {
        108: 0.1,
        76: 0.1,
        116: 0.1,
    }
    robot = Robot(steer_can_id_list=steer_wheel_can_id_list, steer_wheel_directions=steer_wheel_directions, wheel_can_id_list=drive_wheel_can_id_list, wheel_directions=drive_wheel_directions, base_can_id=base_can_id, steer_offsets=steer_offsets)
    if robot.connect("/dev/ttyACM0"):
        print("Robot connected")
    else:
        print("Robot connection failed")
    
    canvas = np.zeros((720, 720, 3), dtype=np.uint8)
    robot.run()
    controls = {"direction": 0, "speed": 0}
    joystick = Joystick(interface="/dev/input/js0", connecting_using_ds4drv=False)
    threading.Thread(target=joystick.listen, daemon=True).start()

    while True:
        calibrated = True
        for id in robot.steer_can_id_list:
            # print(f"Position {id}: {robot.steer_pos_list[id] + 180 + robot.offsets[id]}")
            if abs(robot.steer_pos_list[id]) > 2:
                robot._set_position(id, robot.steer_pos_list[id] * 0.1)
                calibrated = False
        # print(f"{robot.steer_pos_list}")
        if calibrated:
            # input("Calibrated, press enter to continue")
            break
    # for id in robot.steer_can_id_list:
    #     robot._set_position(id, 0.0)

    while True:
        print(f"Joystick: {joystick.direction}, {joystick.speed}")
        # input()
        controls["direction"] = joystick.direction * 360.0
        controls["speed"] = joystick.speed * 1.0
        # print(f"Direction: {controls['direction']}, Speed: {controls['speed']}")
        # print(f"V_x: {joystick.v_x}, V_y: {joystick.v_y}, V_theta: {joystick.v_theta}")
        


        if joystick.is_swerve_mode:
            v_x = joystick.v_x * 2
            v_y = joystick.v_y * 2
            v_theta = joystick.v_theta * np.pi
            # calculate vi for each wheel
            for i in range(len(steer_wheel_can_id_list)):
                steer_wheel_id = steer_wheel_can_id_list[i]
                v_ix = v_y - v_theta * steer_wheel_distance[steer_wheel_id] * np.sin(steer_wheel_mount_angle[steer_wheel_id])
                v_iy = v_x + v_theta * steer_wheel_distance[steer_wheel_id] * np.cos(steer_wheel_mount_angle[steer_wheel_id])
                # speed of the wheel
                speed_i = np.sqrt(v_ix**2 + v_iy**2)
                # direction of the wheel
                theta_i = np.degrees(np.arctan2(v_iy, v_ix))
                # find the nearest 90 degree angle
                # if theta_i > 90:
                #     theta_i = theta_i - 180
                #     speed_i = -speed_i
                # elif theta_i < -90:
                #     theta_i = theta_i + 180
                #     speed_i = -speed_i
                # set the speed of the wheel
                drive_wheel_id = drive_wheel_can_id_list[i]
                robot._set_speed(drive_wheel_id, speed_i)
                # set the direction of the wheel
                steering_wheel_id = steer_wheel_can_id_list[i]
                robot._set_position(steering_wheel_id, theta_i)
        elif joystick.is_rotation_mode:
            for id in robot.steer_can_id_list:
                robot._set_position(id, steer_rotation_mode_pos[id])
            robot._set_speed(13, controls["speed"])
            robot._set_speed(0, -controls["speed"])
            robot._set_speed(84, controls["speed"])
        elif joystick.is_driving_mode:
            robot._set_position(116, controls["direction"])
            robot._set_position(76, controls["direction"])
            robot._set_position(108, 0)
            for id in robot.drove_wheel_can_id_list:
                robot._set_speed(id, controls["speed"])
        else:
            for id in robot.steer_can_id_list:
                robot._set_position(id, controls["direction"])
                pass
            for id in robot.drove_wheel_can_id_list:
                robot._set_speed(id, controls["speed"])
        # print(f"Direction list: {robot.direction_list}")
        time.sleep(0.01)