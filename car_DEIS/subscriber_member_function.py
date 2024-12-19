# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial
import time


class MinimalSubscriber(Node):

    def __init__(self, ser):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'action2',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        commands = msg.data.split(",")
        print(f"commands: {commands}")

        if commands[1] == 'g': #setSpeed command
            speeds = commands[-1].split(";")
            data_2_send = []
            for s in speeds:
                s = int(s)
                if s >= 0:
                    data_2_send.append(s)
                    data_2_send.append(0)
                elif s < 0:
                    data_2_send.append(abs(s))
                    data_2_send.append(1)
            
            speeds_bytes = bytearray(data_2_send)
            ser.write(speeds_bytes)
            line = ser.readline().decode('utf-8').rstrip()
            #time.sleep(0.1)
        

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.reset_input_buffer()
def main(args=None):
    
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber(ser)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
