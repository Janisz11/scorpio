#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16, Float32

class VelocityProcessor:
    def __init__(self):
        rospy.init_node('velocity_processor_node', anonymous=True)
        rospy.Subscriber("/virtual_dc_motor/get_position", UInt16, self.data_handler)
        self.speed_publisher = rospy.Publisher("/motor_data_driver/speed", Float32, queue_size=10)
        self.last_pos = None
        self.last_timestamp = None

    def data_handler(self, input_data):
        timestamp_now = rospy.get_time()

        if self.last_pos is not None and self.last_timestamp is not None:
            position_diff = input_data.data - self.last_pos
            if position_diff < 0:
                position_diff += 4096
            time_diff = timestamp_now - self.last_timestamp
            calculated_rpm = (position_diff / 4096.0) * 60.0 / time_diff
            self.speed_publisher.publish(calculated_rpm)

        self.last_pos = input_data.data
        self.last_timestamp = timestamp_now

if __name__ == '__main__':
    try:
        velocity_obj = VelocityProcessor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
