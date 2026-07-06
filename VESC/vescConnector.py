"""
VESCConnector class for communication with VESC motors as force interface.
"""
import threading
import time
import traceback

import cv2
import numpy as np
from pyvesc import VESC


class VESCConnector:
    """VESC connector with a Dynamixel-like force sensor API."""

    def __init__(self, serial_port="COM8", current_limit=2.0, fake_connection=False, show_graph=False):
        self.serial_port = serial_port
        self.current_limit = float(current_limit)
        self.fake_connection = fake_connection

        self.motor = None
        self.can_id_list = []
        self.zero_pos_list = []
        self.k_list = []
        self.force_scale = [1.0, 1.0, 1.0]

        self.measured_pos_list = [0.0, 0.0, 0.0]
        self.cmd_pos_list = [0.0, 0.0, 0.0]

        self.feedback_force = np.zeros(3, dtype=np.float64)
        self.force_list = {"Tx": 0.0, "Ty": 0.0, "Tz": 0.0, "Fx": 0.0, "Fy": 0.0, "Fz": 0.0}
        self.position_list = [0.0, 0.0, 0.0]
        self.current_list = [0.0, 0.0, 0.0]

        self.is_running = False
        self.control_thread = None
        self.lock = threading.Lock()

        self.show_graph = show_graph
        self.win_name = "VESC Force Graph"
        self.history_len = 600
        self.canvas = np.zeros((720, 1200, 3), dtype=np.uint8)
        self.tx_hist = []
        self.ty_hist = []
        self.fx_hist = []
        self.fy_hist = []

    def connect(self, base_can_id=None, can_id_list=None, zero_pos_list=None, k_list=None):
        """
        Connect to VESC.
        Returns True on success and False on failure.
        """
        self.base_can_id = base_can_id if base_can_id is not None else 16
        self.can_id_list = can_id_list if can_id_list is not None else [28, 16]
        self.zero_pos_list = zero_pos_list if zero_pos_list is not None else [250.0, 340.0]
        self.k_list = k_list if k_list is not None else [0.06, 0.06]
        self.tx_hist = []
        self.ty_hist = []
        self.fx_hist = []
        self.fy_hist = []

        if self.fake_connection:
            print("VESCConnector: fake connection enabled")
            return True

        try:
            self.motor = VESC(serial_port=self.serial_port)
            # Open context manually to keep the connection for the app lifetime.
            self.motor.__enter__()
            print("VESC connected:", self.serial_port)
            try:
                print("Firmware:", self.motor.get_firmware_version())
            except Exception:
                pass
            return True
        except Exception:
            traceback.print_exc()
            self.motor = None
            return False

    def calibrate(self):
        """
        Zero internal force estimate. If a live position is available, update
        zero positions from current rotor position.
        """
        self.feedback_force[:] = 0.0
        with self.lock:
            self.force_list["Tx"] = 0.0
            self.force_list["Ty"] = 0.0
            self.force_list["Tz"] = 0.0
            self.force_list["Fx"] = 0.0
            self.force_list["Fy"] = 0.0
            self.force_list["Fz"] = 0.0

        if self.fake_connection or self.motor is None:
            return

        try:
            for i, can_id in enumerate(self.can_id_list):
                measured_values = self._get_measurements_verified(can_id=can_id)
                self.measured_pos_list[i] = self._extract_position_deg(measured_values)
                self.zero_pos_list[i] = self.measured_pos_list[i]
        except Exception:
            traceback.print_exc()

    def run(self):
        """Start spring control loop."""
        if self.is_running:
            return
        print(f"Target position: {self.zero_pos_list}")
        while True:
            try:
                motor_0_pos = self._extract_position_deg(self._get_measurements_verified(can_id=self.can_id_list[0]))
                motor_1_pos = self._extract_position_deg(self._get_measurements_verified(can_id=self.can_id_list[1]))
                current_pos = [motor_0_pos, motor_1_pos]
                print(f"{self.can_id_list[0]}: {motor_0_pos}, {self.can_id_list[1]}: {motor_1_pos}")
                offset = [zero_pos - current_pos for zero_pos, current_pos in zip(self.zero_pos_list, current_pos)]
                print(f"Offset: {offset}")
                if abs(offset[0]) > 10 or abs(offset[1]) > 10:
                    print("Offset is too large, please adjust the target position and run again")
                    input("Press Enter to continue...")
                    continue
                else:
                    input("Press Enter to continue...")
                    break
            except Exception:
                traceback.print_exc()
                input("Press Enter to continue...")
        print("Starting...")
        self.is_running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()

    def _control_loop(self):
        while self.is_running:
            try:
                if self.fake_connection or self.motor is None:
                    time.sleep(0.02)
                    continue

                for i, can_id in enumerate(self.can_id_list):
                    try:
                        measured_values = self._get_measurements_verified(can_id=can_id)
                        self.measured_pos_list[i] = self._extract_position_deg(measured_values)
                    except AttributeError:
                        pass
                    
                    # Virtual spring offset from feedback force.
                    # cmd_pos = (self.zero_pos_list[i] - self.feedback_force[i] / max(self.k_list[i], 1e-6)) % 360
                    # self.cmd_pos_list[i] = cmd_pos

                    # self._set_position(cmd_pos, can_id)
                # print("cmd_pos_list", self.cmd_pos_list)
                self._update_measured_force_values()
                # print("update_force_values", self.is_running)
                # time.sleep(0.01)
            except KeyboardInterrupt:
                self.is_running = False
                print("KeyboardInterrupt")
                break
            except Exception:
                traceback.print_exc()
                time.sleep(0.05)
            time.sleep(0.02)
        print("control_loop ended")
        cv2.destroyAllWindows()

    def _update_measured_force_values(self):
        with self.lock:
            tx_2_zero = 0.0
            ty_2_zero = 0.0
            for i in range(len(self.can_id_list)):
                pos = self._safe_float(self.measured_pos_list[i], self.zero_pos_list[i])
                self.position_list[i] = pos

                relative_pos_2_zero = (self.measured_pos_list[i] + 180 - self.zero_pos_list[i]) % 360 - 180
                # Human-force estimate from spring displacement. which is the input to the system
                estimate_2_zero = -relative_pos_2_zero * self.k_list[i]
                if i == 0:
                    tx_2_zero = estimate_2_zero
                elif i == 1:
                    ty_2_zero = estimate_2_zero
                
                # Human-force estimate from command position. which is the measured human force
                relative_pos_2_cmd = (self.measured_pos_list[i] + 180 - self.cmd_pos_list[i]) % 360 - 180
                estimate_2_cmd = -relative_pos_2_cmd * self.k_list[i]
                if i == 0:
                    tx_2_cmd = estimate_2_cmd
                elif i == 1:
                    ty_2_cmd = estimate_2_cmd
            

            self.force_list["Tx"] = float(tx_2_zero * self.force_scale[0])
            self.force_list["Ty"] = float(ty_2_zero * self.force_scale[1])
            self.force_list["Fz"] = 0.0
            # self.force_list["Fx"] = self.force_list["Tx"]
            # self.force_list["Fy"] = self.force_list["Ty"]
            # self.force_list["Fz"] = self.force_list["Tz"]
            self.force_list["Fx"] = float(tx_2_cmd * self.force_scale[0])
            self.force_list["Fy"] = float(ty_2_cmd * self.force_scale[1])
            self.force_list["Fz"] = 0.0
            self._append_history()

    def _append_history(self):
        tx = -(self.cmd_pos_list[0]- self.zero_pos_list[0]) * self.k_list[0]
        ty = -(self.cmd_pos_list[1]- self.zero_pos_list[1]) * self.k_list[1]

        self.tx_hist.append(tx)
        self.ty_hist.append(ty)

        fx = self.force_list["Fx"]
        fy = self.force_list["Fy"]

        # # deadzone here
        # if abs(fx) < 0.1:
        #     fx = 0.0
        # if abs(fy) < 0.2:
        #     fy = 0.0
        # else:
        #     if fy > 0:
        #         fy = fy - 0.2
        #     else:
        #         fy = fy + 0.2

        self.fx_hist.append(fx)
        self.fy_hist.append(fy)

        if len(self.tx_hist) > self.history_len:
            self.tx_hist.pop(0)
        if len(self.ty_hist) > self.history_len:
            self.ty_hist.pop(0)
        if len(self.fx_hist) > self.history_len:
            self.fx_hist.pop(0)
        if len(self.fy_hist) > self.history_len:
            self.fy_hist.pop(0)

        if self.show_graph:
            self.draw_graph()

    def draw_graph(self):
        self.canvas[:] = 0
        cv2.putText(
            self.canvas,
            "Tx/Ty/Fx/Fy vs Time (press q to quit)",
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (220, 220, 220),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(self.canvas, "Command x: red", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)
        cv2.putText(self.canvas, "Command y: green", (200, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(self.canvas, "Measured x: blue", (350, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.putText(self.canvas, "Measured y: yellow", (500, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2, cv2.LINE_AA)

        self.draw_timeseries_panel(
            self.canvas,
            x0=20,
            y0=110,
            w=1160,
            h=580,
            title="Force/Torque Signals",
            series=[self.tx_hist, self.ty_hist, self.fx_hist, self.fy_hist],
            colors=[(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255)],
            y_range=None,
        )

        cv2.imshow(self.win_name, self.canvas)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            self.is_running = False

    def draw_timeseries_panel(self, img, x0, y0, w, h, title, series, colors, y_range=None):
        cv2.rectangle(img, (x0, y0), (x0 + w, y0 + h), (60, 60, 60), 1)
        cv2.putText(
            img,
            title,
            (x0 + 8, y0 + 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (220, 220, 220),
            2,
            cv2.LINE_AA,
        )

        if not series or not series[0]:
            return

        n = len(series[0])
        if n < 2:
            return

        if y_range is None:
            max_abs = 1e-6
            for s in series:
                if not s:
                    continue
                s_max = np.nanmax(np.abs(np.array(s, dtype=np.float32)))
                if np.isfinite(s_max):
                    max_abs = max(max_abs, float(s_max))
            ymax = max_abs
            ymin = -max_abs
        else:
            ymin, ymax = y_range
            if ymax == ymin:
                ymax = ymin + 1.0

        plot_x0 = x0 + 8
        plot_y0 = y0 + 32
        plot_w = w - 16
        plot_h = h - 44

        if ymin < 0 < ymax:
            zy = int(plot_y0 + (ymax / (ymax - ymin)) * plot_h)
            cv2.line(img, (plot_x0, zy), (plot_x0 + plot_w, zy), (80, 80, 80), 1, cv2.LINE_AA)

        cv2.putText(
            img,
            f"{ymax:.3f}",
            (plot_x0 + plot_w - 80, plot_y0 + 14),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (160, 160, 160),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            img,
            f"{ymin:.3f}",
            (plot_x0 + plot_w - 80, plot_y0 + plot_h),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (160, 160, 160),
            1,
            cv2.LINE_AA,
        )

        xs = np.linspace(plot_x0, plot_x0 + plot_w - 1, n, dtype=np.float32)
        for s, c in zip(series, colors):
            ys = np.array(s, dtype=np.float32)
            ys = np.clip(ys, ymin, ymax)
            ys_px = plot_y0 + (ymax - ys) / (ymax - ymin) * plot_h
            pts = np.stack([xs, ys_px], axis=1).astype(np.int32)
            cv2.polylines(img, [pts], False, c, 2, cv2.LINE_AA)

    def set_feedback_force(self, force):
        """Set feedback force [Tz, Ty, Tx] style vector from simulation pipeline."""
        arr = np.array(force, dtype=np.float64).flatten()
        if arr.shape[0] >= 3:
            self.feedback_force = [arr[2], arr[1], arr[0]]
        # self.feedback_force = [0.0, 0.0, 0.0]

    def get_measured_force(self, scale=1.0, filter_dead_zone=0.0):
        with self.lock:
            result = self.force_list.copy()
            for key in ("Tx", "Ty", "Tz", "Fx", "Fy", "Fz"):
                result[key] = result[key] * scale
                if abs(result[key]) < filter_dead_zone:
                    result[key] = 0.0
                else:
                    result[key] = result[key] - filter_dead_zone if result[key] > 0 else result[key] + filter_dead_zone
            return result


    def close(self):
        if self.control_thread is not None:
            while self.control_thread.is_alive():
                self.is_running = False
                self.control_thread.join(timeout=0.5)
        print("get force thread closed")

        if not self.fake_connection and self.motor is not None:
            try:
                for can_id in self.can_id_list:
                    self._set_current(0.0, can_id)
            except Exception:
                traceback.print_exc()
            try:
                self.motor.__exit__(None, None, None)
            except Exception:
                pass
            self.motor = None
        cv2.destroyAllWindows()
        print("motor connection closed")

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
                time.sleep(0.003)
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
    def _set_position(self, position, can_id):
        if can_id == self.base_can_id:
            self.motor.set_pos(float(position), can_id=None)
        else:
            self.motor.set_pos(float(position), can_id=can_id)

    def _set_current(self, current, can_id):
        current = float(np.clip(current, -self.current_limit, self.current_limit))
        if can_id == self.base_can_id:
            self.motor.set_current(current, can_id=None)
        else:
            self.motor.set_current(current, can_id=can_id)


if __name__ == "__main__":
    can_id_list = [108, 76, 116]
    connector = VESCConnector(serial_port="COM8", current_limit=2.0, fake_connection=False, show_graph=True)
    if connector.connect(base_can_id=108, can_id_list=can_id_list, zero_pos_list=[340.0, 250.0], k_list=[1, 1]):
        # connector.calibrate()
        connector.run()
        force = [0.0, 0.0, 0.0]
        try:
            while True:
                # print(connector.get_force())
                print(f"{can_id_list[0]}: {connector.measured_pos_list[0]}, {can_id_list[1]}: {connector.measured_pos_list[1]}")
                # connector.set_feedback_force(force)
                # force[1] -= 0.5
                time.sleep(0.05)
        except KeyboardInterrupt:
            pass
        finally:
            connector.close()
