from pyvesc import VESC
import traceback
import time
import numpy as np
import cv2
import threading


class Robot:
    def __init__(self, steer_can_id_list, wheel_can_id_list, wheel_directions, base_can_id, steer_offsets):
        self.base_can_id = base_can_id
        self.offsets = steer_offsets
        self.steer_can_id_list = steer_can_id_list
        self.steer_pos_list = {can_id: 0.0 for can_id in steer_can_id_list}
        self.wheel_can_id_list = wheel_can_id_list
        self.wheel_directions = wheel_directions
        self.motor = None

    
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
    
    def _set_position(self, can_id, relative_position):
        absolute_position = 180 - (relative_position + self.offsets[can_id])
        if absolute_position < 0:
            absolute_position += 360
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
            current_gain = 0.1
            if can_id == self.base_can_id:
                self.motor.set_current(speed * self.wheel_directions[can_id] * current_gain, can_id=None)
            else:
                self.motor.set_current(speed * self.wheel_directions[can_id] * current_gain, can_id=can_id)

    def _control_loop(self):
        while True:
            try:
                for i, can_id in enumerate(self.steer_can_id_list):
                    pos_read = self._extract_position_deg(self._get_measurements_verified(can_id=can_id))
                    if pos_read > 180: 
                        pos_relative = pos_read - 360
                    else:
                        pos_relative = pos_read
                    # pos_rad = np.radians(pos_relative)
                    self.steer_pos_list[can_id] = 180 - (pos_relative + self.offsets[can_id])
                    # reverse = 180 - self.motor_pos_list[can_id] - self.offsets[can_id]
                    # if reverse < 0:
                    #     reverse += 360
                    # if can_id == 76:
                    #     print(f"Read position: {pos_read}, Reverse: {reverse}")
                # print(f"Motor positions: {self.motor_pos_list}")
            except Exception:
                traceback.print_exc()
                continue
            time.sleep(0.01)
    
    def run(self):
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()


if __name__ == "__main__":
    steer_offsets = {
        # 108: 0.0,
        # 76: 25.0,
        # 116: 115.0,
        108: 180.0,
        76:155.0,
        116: 65.0,

    }
    base_can_id = 108
    steer_can_id_list = [108, 76, 116]
    wheel_can_id_list = [84, 0, 13]
    wheel_directions = {
        84: 1,
        0: 1,
        13: 1,
    }
    robot = Robot(steer_can_id_list=steer_can_id_list, wheel_can_id_list=wheel_can_id_list, wheel_directions=wheel_directions, base_can_id=base_can_id, steer_offsets=steer_offsets)
    if robot.connect():
        print("Robot connected")
    else:
        print("Robot connection failed")
    
    canvas = np.zeros((720, 720, 3), dtype=np.uint8)
    robot.run()
    controls = {"direction": 0, "speed": 0}
    max_speed = 100
    max_direction_angle = 90
    speed_trackbar_center = max_speed
    direction_trackbar_center = max_direction_angle

    def on_speed_change(val):
        controls["speed"] = val - speed_trackbar_center

    def on_direction_change(val):
        controls["direction"] = val - direction_trackbar_center

    cv2.namedWindow("Robot Control")
    cv2.createTrackbar("Robot Speed", "Robot Control", speed_trackbar_center, max_speed * 2, on_speed_change)
    cv2.createTrackbar("Robot Direction", "Robot Control", direction_trackbar_center, max_direction_angle * 2, on_direction_change)

    while True:
        robot_direction = controls["direction"]
        robot_speed = controls["speed"]
        canvas[:] = 0
        cv2.putText(canvas, "Robot Control", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2, cv2.LINE_AA)
        # cv2.putText(canvas, f"Robot Direction: {robot_direction}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2, cv2.LINE_AA)
        cv2.putText(canvas, f"Robot Speed: {robot_speed}", (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2, cv2.LINE_AA)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("d"):
            controls["direction"] = min(90, controls["direction"] + 1)
            cv2.setTrackbarPos("Robot Direction", "Robot Control", controls["direction"] + direction_trackbar_center)
        elif key == ord("a"):
            controls["direction"] = max(-90, controls["direction"] - 1)
            cv2.setTrackbarPos("Robot Direction", "Robot Control", controls["direction"] + direction_trackbar_center)
        if key == ord("w"):
            controls["speed"] += 1
            cv2.setTrackbarPos("Robot Speed", "Robot Control", controls["speed"])
        elif key == ord("s"):
            controls["speed"] -= 1
            cv2.setTrackbarPos("Robot Speed", "Robot Control", controls["speed"])
        robot_direction = controls["direction"]
        robot_speed = controls["speed"]
        for id in robot.steer_can_id_list:
            robot._set_position(id, robot_direction)
        for id in robot.wheel_can_id_list:
            robot._set_speed(id, robot_speed)
        # draw the rotion of the wheels
        left_wheel_pos = robot.steer_pos_list[116]
        right_wheel_pos = robot.steer_pos_list[76]
        rear_wheel_pos = robot.steer_pos_list[108]
        # left_wheel blue
        cv2.circle(canvas, (300, 260), 10, (255, 0, 0), 2)
        cv2.arrowedLine(canvas, (300, 260), (300 + int(100 * np.sin(np.radians(left_wheel_pos))), 260 - int(100 * np.cos(np.radians(left_wheel_pos)))), (255, 0, 0), 2)
        cv2.putText(
            canvas, 
            f"Left Wheel: {left_wheel_pos}", 
            (300, 160), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2, cv2.LINE_AA)
        # right_wheel green
        cv2.circle(canvas, (420, 260), 10, (0, 255, 0), 2)
        cv2.arrowedLine(canvas, (420, 260), (420 + int(100 * np.sin(np.radians(right_wheel_pos))), 260 - int(100 * np.cos(np.radians(right_wheel_pos)))), (0, 255, 0), 2)
        cv2.putText(
            canvas, 
            f"Right Wheel: {right_wheel_pos}", 
            (420, 260), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2, cv2.LINE_AA)
        # rear_wheel red    
        cv2.circle(canvas, (360, 360), 10, (0, 0, 255), 2)
        cv2.arrowedLine(canvas, (360, 360), (360 + int(100 * np.sin(np.radians(rear_wheel_pos))), 360 - int(100 * np.cos(np.radians(rear_wheel_pos)))), (0, 0, 255), 2)
        cv2.putText(
            canvas, 
            f"Rear Wheel: {rear_wheel_pos}", 
            (360, 360), 
            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (220, 220, 220), 2, cv2.LINE_AA)

        cv2.imshow("Robot Control", canvas)
        
        if key in (ord("q"), 27):
            break
    cv2.destroyAllWindows()
    