from pyPS4Controller.controller import Controller
import time
import threading

class Joystick(Controller):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.speed = 0
        self.direction = 0
        self.rotation_speed = 0

        self.v_x = 0
        self.v_y = 0
        self.v_theta = 0

        self.speed_deadzone = 0.1
        self.direction_deadzone = 0.1

        self.v_x_deadzone = 0.2
        self.v_y_deadzone = 0.2
        self.v_theta_deadzone = 0.2
        
        self.speed_offset = -65535/2
        self.direction_offset = 0
        self.rotation_offset = 0

        self.speed_scale = 1 / (65535/2)
        self.direction_scale = 1 / (65535/2)
        self.rotation_scale = 1 / (65535/2)

        self.v_x_scale = 1 / (65535/2)
        self.v_y_scale = -1 / (65535/2)
        self.v_theta_scale = -1 / (65535/2)

        self.v_x_offset = 0
        self.v_y_offset = 0
        self.v_theta_offset = 0

        self.is_rotation_mode = False
        self.is_driving_mode = False
        self.is_swerve_mode = False
        self.is_omni_mode = False
        

    def on_x_press(self):
        self.is_swerve_mode = True

    def on_x_release(self):
        self.is_swerve_mode = False

    def on_circle_press(self):
        self.is_omni_mode = True

    def on_circle_release(self):
        self.is_omni_mode = False

    def on_square_press(self):
        pass

    def on_square_release(self):
        pass

    def on_L1_press(self):
        self.is_rotation_mode = True

    def on_L1_release(self):
        self.is_rotation_mode = False

    def on_L2_press(self, value):
        self.speed = -(value - self.speed_offset) * self.speed_scale

    def on_L2_release(self):
        self.speed = 0

    def on_R1_press(self):
        self.is_driving_mode = True

    def on_R1_release(self):
        self.is_driving_mode = False

    def on_R2_press(self, value):
        self.speed = (value - self.speed_offset) * self.speed_scale

    def on_R2_release(self):
        self.speed = 0

    def on_up_arrow_press(self):
        pass

    def on_up_down_arrow_release(self):
        pass

    def on_down_arrow_press(self):
        pass

    def on_left_arrow_press(self):
        pass

    def on_left_right_arrow_release(self):
        pass

    def on_right_arrow_press(self):
        pass

    def on_L3_up(self, value):
        self.v_y = value * self.v_y_scale
        if abs(self.v_y) < self.v_y_deadzone:
            self.v_y = 0
        else:
            self.v_y -= self.v_y_deadzone

    def on_L3_down(self, value):
        self.v_y = value * self.v_y_scale
        if abs(self.v_y) < self.v_y_deadzone:
            self.v_y = 0
        else:
            self.v_y += self.v_y_deadzone

    def on_L3_left(self, value):
        self.v_x = value * self.v_x_scale
        if abs(self.v_x) < self.v_x_deadzone:
            self.v_x = 0
        else:
            self.v_x += self.v_x_deadzone

    def on_L3_right(self, value):
        self.v_x = value * self.v_x_scale
        if abs(self.v_x) < self.v_x_deadzone:
            self.v_x = 0
        else:
            self.v_x -= self.v_x_deadzone
    def on_L3_y_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        self.v_y = 0

    def on_L3_x_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        self.v_x = 0

    def on_L3_press(self):
        """L3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        pass

    def on_L3_release(self):
        """L3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        pass

    def on_R3_up(self, value):
        pass

    def on_R3_down(self, value):
        pass

    def on_R3_left(self, value):
        self.direction = value * self.direction_scale
        if self.is_swerve_mode:
            self.v_theta = value * self.v_theta_scale
        if abs(self.v_theta) < self.v_theta_deadzone:
            self.v_theta = 0
        else:
            self.v_theta -= self.v_theta_deadzone
    def on_R3_right(self, value):
        self.direction = value * self.direction_scale
        if self.is_swerve_mode:
            self.v_theta = value * self.v_theta_scale
        if abs(self.v_theta) < self.v_theta_deadzone:
            self.v_theta = 0
        else:
            self.v_theta += self.v_theta_deadzone
    def on_R3_y_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        pass

    def on_R3_x_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        self.direction = 0
        if self.is_swerve_mode:
            self.v_theta = 0

    def on_R3_press(self):
        """R3 joystick is clicked. This event is only detected when connecting without ds4drv"""
        pass

    def on_R3_release(self):
        """R3 joystick is released after the click. This event is only detected when connecting without ds4drv"""
        pass

    def on_options_press(self):
        pass

    def on_options_release(self):
        pass

    def on_share_press(self):
        """this event is only detected when connecting without ds4drv"""
        pass

    def on_share_release(self):
        """this event is only detected when connecting without ds4drv"""
        pass

    def on_playstation_button_press(self):
        """this event is only detected when connecting without ds4drv"""
        pass

    def on_playstation_button_release(self):
        """this event is only detected when connecting without ds4drv"""
        pass


if __name__ == "__main__":
    joystick = Joystick(interface="/dev/input/js0", connecting_using_ds4drv=False)
    threading.Thread(target=joystick.listen, daemon=True).start()
    controls = {"direction": 0, "speed": 0}
    while True:
        controls["direction"] = joystick.direction
        controls["speed"] = joystick.speed
        print(f"Speed: {controls['speed']}, Direction: {controls['direction']}")
        time.sleep(0.01)