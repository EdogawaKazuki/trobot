from pyvesc import VESC
import time
import numpy as np
import traceback

serial_port = "COM8"
current_limit = 2.0


def run_spring_controller():
    with VESC(serial_port=serial_port) as motor:
        print("Firmware:", motor.get_firmware_version())
        motor.set_duty_cycle(0.02)

        samp_freq = 100.0
        samp_interval = 1.0 / samp_freq
        cmd_in_every_n_frame = 1
        offset_list = [180 - 205 - 135, 180 - 115 - 135]
        zero_pos_list = [340, 250]
        can_id_list = [None, 28]

        while True:
            try:
                for i, can_id in enumerate(can_id_list):
                    motor.set_pos(zero_pos_list[i], can_id=can_id)
            except KeyboardInterrupt:
                break
            except Exception as e:
                print(traceback.format_exc())
                continue


if __name__ == "__main__":
    run_spring_controller()
