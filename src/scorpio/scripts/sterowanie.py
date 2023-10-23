#!/usr/bin/env python3
import rospy

from std_msgs.msg import Int8
from pynput import keyboard

class MotorController:
    def __init__(self):
        rospy.init_node('motor_controller', anonymous=True)
        self.pub = rospy.Publisher('/virtual_dc_motor/set_cs', Int8, queue_size=10)

        
        self.current_speed = 0
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def update_speed(self, value):
        self.current_speed = value
        self.pub.publish(self.current_speed)
        print(f"Set Motor Speed to: {self.current_speed}")

    def on_key_press(self, key):
        try:
            if key == keyboard.Key.right:
                self.update_speed(min(self.current_speed + 1, 100))
            elif key == keyboard.Key.left:
                self.update_speed(max(self.current_speed - 1, -100))
        except AttributeError:
            pass

if __name__ == '__main__':
    controller = MotorController()
    rospy.spin()