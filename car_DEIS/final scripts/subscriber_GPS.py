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
import datetime
import math
import time

#from subscriber_member_function import *
import cv2 as cv
import numpy as np
import urllib3
from collections import deque

def download_image(url,save_as):
    http = urllib3.PoolManager()
    response = http.request('GET',url)
    with open(save_as, 'wb') as file:
        file.write(response.data)

def block_reduce(image,block_size):
    rows,cols = image.shape
    new_rows,new_cols = rows // block_size, cols // block_size
    
    reduced_image = np.zeros((new_rows,new_cols),dtype=np.uint8)
    
    for i in range(new_rows):
        for j in range(new_cols):
            block = image[i*block_size:(i+1)*block_size,j*block_size:(j+1)*block_size]
            if np.any(block==255):
                reduced_image[i,j] = 255
            else:
                reduced_image[i,j] = 0
    return reduced_image

def find_path(image,start,end, search_for):
    rows,cols = image.shape
    
    directions = [(-1,0), (1,0), (0,-1), (0,1), 
                  (-1,-1), (-1,1), (1,-1), (1,1)]
    if search_for == "white":
        path_to_white = []
        if image[start[0]][start[1]] == 0:
            path_to_white = find_path(image,start,end,"black")
            queue = deque([path_to_white[-1]])
            #print(path_to_white[-1])
            visited = set()
            #for point in path_to_white:
                #visited.add(point)
            visited.add(path_to_white[-1])
            parent = {path_to_white[-1]:None}
        else: 
            queue = deque([start])
            visited = set()
            visited.add(start)
            parent = {start:None}
        while queue:
            current = queue.popleft()
            if current == end:
                path = []
                while current is not None:
                    path.append(current)
                    current = parent[current]
                path.reverse()
                return path_to_white + path
            
            for dx,dy in directions:
                x,y = current[0] + dx, current[1] + dy
                if 0 <= x < rows and 0 <= y < cols and (x,y) not in visited:
                    if image[x,y] == 255:
                        queue.append((x,y))
                        visited.add((x,y))
                        parent[(x,y)] = current
    elif search_for == "black":
        print("black")
        queue = deque([start])
        visited = set()
        visited.add(start)
        parent = {start:None}
        while queue:
            current = queue.popleft()
            if image[current[0]][current[1]] == 255:
                path = []
                while current is not None:
                    #print(f"{image[current[0]][current[1]]}, {current}")
                    path.append(current)
                    current = parent[current]
                path.reverse()
                #print("epic find")
                return path
            
            for dx,dy in directions:
                x,y = current[0] + dx, current[1] + dy
                if 0 <= x < rows and 0 <= y < cols and (x,y) not in visited:
                    queue.append((x,y))
                    visited.add((x,y))
                    parent[(x,y)] = current
                    
    return []

def find_nearest_white(img, target):
    nonzero = cv.findNonZero(img)
    distances = np.sqrt((nonzero[:,:,0] - target[0]) ** 2 + (nonzero[:,:,1] - target[1]) ** 2)
    nearest_index = np.argmin(distances)
    return nonzero[nearest_index][0].tolist()
    
def path_finding(start,end):
    url = 'http://192.168.1.2/axis-cgi/jpg/image.cgi'
    save = '/home/deis-group2-rpi4/ros2_commands/src/py_pubsub/py_pubsub/image.jpg'

    erode_filter = np.ones((25,25),np.uint8)
    dilate_filter = np.ones((43,43),np.uint8)
    #while True:
    #download_image(url,save)
    img = cv.imread(save)
    frame_HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    #frame_threshold = cv.inRange(frame_HSV, (110, 80, 80), (130, 255, 255))
    frame_threshold = cv.inRange(frame_HSV, (0, 0, 0), (255, 255, 60))
    frame_threshold = cv.erode(frame_threshold,erode_filter)
    frame_threshold = cv.dilate(frame_threshold,dilate_filter)
    frame_threshold = cv.erode(frame_threshold,erode_filter)
    #extra_eroded = cv.erode(frame_threshold,erode_filter)
    
    #extra_eroded = cv.erode(frame_threshold,erode_filter)
    size = 20
    resized_frame = block_reduce(frame_threshold, size)
    #frame_threshold = cv.circle(frame_threshold,(1800,1100),radius=10,color=(100,255,255),thickness=-1)
    #frame_threshold = cv.circle(frame_threshold,(505,600),radius=10,color=(100,255,255),thickness=-1)
    #start_point = (round(1170*scale),round(800*scale))
    #end_point = (round(1177*scale),round(1000*scale))
    #path = find_path(resized_frame,start_point,end_point,"white")
    #cv.imshow("resized", frame_threshold)
    #cv.waitKey(0)
    #cv.destroyAllWindows()
    start_point = (start[0] // size,start[1] // size)
    valid_end = find_nearest_white(frame_threshold,end)
    print(f"AAAAAHHH: {valid_end}")
    print(frame_threshold[valid_end[1]][valid_end[0]])
    end_point = (valid_end[1] // size,valid_end[0] // size)
    print(resized_frame[end_point[0]][end_point[1]])
    path = find_path(resized_frame, start_point,end_point,"white")
    #frame_threshold = cv.rectangle(frame_threshold,(1000,1000),(1100,1100),color=(100,255,255),thickness=-1) 
    if path:
        scaled_path = [(y * size + size //2, x * size + size // 2) for y,x in path]
        #path_image = cv.cvtColor(frame_threshold,cv.COLOR_GRAY2BGR)
        #print(scaled_path)
        #for point in scaled_path:
            #print(point)
            #cv.circle(path_image, (point[1], point[0]),1,(0,0,255),-1)
        #cv.imshow("Path", path_image)
        #cv.waitKey(0)
        #cv.destroyAllWindows()
        return scaled_path
    else:
        print("No path")
        return []


enemy_position = [0,0]
OUR_ID = [2,3]
NUM_SPIRALS = 10
MOTOR_DELAY_SECS = 2

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.reset_input_buffer()

def limit_heading(heading):
    new_heading = heading
    if heading > 180:
        new_heading = heading - 360
    elif heading < -180:
        new_heading = heading + 360
    return new_heading

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription1 = self.create_subscription(
            String,
            'robotPositions',
            self.listener_callback,
            1)
        self.subscription1  # prevent unused variable warning

        self.subscription2 = self.create_subscription(
            String,
            'building_com',
            self.building_callback,
            1)
        self.subscription2

        self.old_timestamp = datetime.datetime.utcnow()
        self.old_coords = [(0, 0) for _ in range(NUM_SPIRALS)]
        self.old_velocity_vecs = [(0, 0) for _ in range(NUM_SPIRALS)]
        self.old_speeds = [0,0]
        self.old_heading = 0
        self.old_our_coords = [0,0]
        self.path = []
        self.at_point = False
        self.MOVE_TO = (0,0)
        self.go_home = False
        self.manual_car = True
        self.going_home = False
        self.at_home = False

    def building_callback(self, msg):
        commands = msg.data.split(",")
        if commands[1] == 'dc':
            coords = commands[-1].split(";")
            print(f"Enemy position: {coords}")
            global enemy_position
            enemy_position = [int(coords[0]),int(coords[1])]
            self.go_home = True

    def move_2_point(self,goal, coords, current_h):
        if current_h == 1337:
            speeds = [0,0]
        else:
            vec_to_goal = (goal[0] - coords[0], goal[1] - coords[1])
            #self.get_logger().info(f" cart:{[(int(v[0]), int(v[1])) for v in relative_distance_vecs_cart[3:4]]}")
            rel_to_goal = [math.sqrt(math.pow(vec_to_goal[0], 2) + math.pow(vec_to_goal[1], 2)),math.acos(vec_to_goal[0]/(0.001 + math.sqrt(math.pow(vec_to_goal[0],2) + math.pow(vec_to_goal[1],2)))) * (math.copysign(1,vec_to_goal[1])*180/math.pi) - current_h]
            rel_to_goal[1] = limit_heading(rel_to_goal[1])
            print(rel_to_goal[1])

            max_speed = 70
            min_speed = 65
            B = 160
            #if current_v[0] > 0 and current_v[0] < 150:
            if rel_to_goal[0] < 120 and len(self.path) == 1:
                speeds = [0,0]
                self.at_point = True
                self.at_home = True
                self.going_home = False
            else:
                if rel_to_goal[0] < 250 and len(self.path) > 1:
                    self.at_point = True
                speeds = [55,55]
                #print("Going forward")
                v = 4*rel_to_goal[0]
                w = 0.08*rel_to_goal[1]
                if rel_to_goal[1] < -30 or rel_to_goal[1] > 30:
                    speeds = [60,-60]
                else:
                    speed_ratios = ((2*v-B*w)/2000,(2*v+B*w)/2000)
                    speeds = [speed_ratios[0]*50,speed_ratios[1]*50]
                    if speeds[0] > max_speed or speeds[1] > max_speed:
                        highest_speed = max(speeds)
                        speeds[0] = speeds[0]/highest_speed*max_speed
                        speeds[1] = speeds[1]/highest_speed*max_speed
                    if (speeds[0] < min_speed and speeds[0] != 0) and (speeds[1] < min_speed and speeds[1] != 0):
                        ratio = min(speeds)/max(speeds)
                        speeds[speeds.index(max(speeds))] = min_speed
                        speeds[speeds.index(min(speeds))] = ratio*min_speed
        """elif rel_to_goal[0] > 100:
            speeds = [60,60]
        else:
            speeds = [0,0]"""
        data_2_send = []
        for s in speeds:
            s = int(s)
            if s >= 0:
                data_2_send.append(s)
                data_2_send.append(0)
            elif s < 0:
                data_2_send.append(abs(s))
                data_2_send.append(1)
        
        print(data_2_send)
        speeds_bytes = bytearray(data_2_send)
        print(speeds)
        ser.write(speeds_bytes)
        #line = ser.readline().decode('utf-8').rstrip()
        self.old_speeds = speeds
        
    
    
    def listener_callback(self, msg):
        global enemy_position
        current_timestamp = datetime.datetime.utcnow()
        #self.get_logger().info('I heard: "%s"' % msg.data)
        if self.going_home and not self.manual_car:
            print("MANUAL")
            switch_mode = bytearray([0,2,0,2])
            time.sleep(0.5)
            ser.write(switch_mode)
            self.manual_car = True
            self.go_home = True
        elif self.manual_car and not self.going_home and len(self.path) < 2 and not self.at_home:
            print("AUTOMATIC")
            #test = [60,0,60,0]
            #test_array = bytearray(test)
            switch_mode = bytearray([0,2,0,2])
            #print(switch_mode)
            #ser.write(switch_mode)
            time.sleep(1.5)
            ser.write(switch_mode)
            self.manual_car = False
        spirals = msg.data[1:-2].split(";")
        #self.get_logger().info("%s" % spirals[0])
        
        coords = [(int(float(s.split(" ")[0])), int(float(s.split(" ")[1]))) for s in spirals]
        
        #print(coords)
        our_coords = [coords.pop(OUR_ID[0]), coords.pop(OUR_ID[1]-1)]
        #print(our_coords)
        
        if self.go_home and our_coords[0][0] > 0 and not self.going_home:
            self.path = path_finding(our_coords[0], enemy_position)
            self.path = self.path[10:-1:4]
            self.go_home = False
            self.going_home = True
        delta_t = current_timestamp - self.old_timestamp
        self.old_timestamp = current_timestamp
        

        
        our_diff = (our_coords[0][0] - our_coords[1][0], our_coords[0][1] - our_coords[1][1])
        #print(our_diff[1]/our_diff[0])
        div = (math.sqrt(math.pow(our_diff[0],2) + math.pow(our_diff[1],2)))
        if div != 0:
            heading = math.acos(our_diff[0]/(math.sqrt(math.pow(our_diff[0],2) + math.pow(our_diff[1],2)))) * (180/math.pi)
        else:
            heading = 0
        if our_diff[1] < 0:
            heading = -heading
        #distances = [math.sqrt(math.pow(c[0] - our_coords[0],2)+math.pow(c[1] - our_coords[1],2)) for c in coords if (c[0] != -1 and c[1] != -1)]
        
        if self.at_home:
            vec_to_goal = (enemy_position[0] - our_coords[0][0], enemy_position[1] - our_coords[0][1])
            rel_heading = math.acos(vec_to_goal[0]/(0.001 + math.sqrt(math.pow(vec_to_goal[0],2) + math.pow(vec_to_goal[1],2)))) * (math.copysign(1,vec_to_goal[1])*180/math.pi) - heading
            rel_heading = limit_heading(rel_heading)
            heading_sign = math.copysign(1,rel_heading)
            #rel_heading = rel_heading + heading_sign*10
            rel_heading = limit_heading(rel_heading)
            print(f"rel heading: {rel_heading}")
            if  rel_heading > 20 or rel_heading < -20:
                #rel_sign = math.copysign(1,rel_heading)
                turn_speed = [55,0,55,1]
                turn_bytes = bytearray(turn_speed)
                time.sleep(0.2)
                ser.write(turn_bytes)
                print("TURNING")
            else:
                stop = [0,0,0,0]
                stop_bytes = bytearray(stop)
                ser.write(stop_bytes)
                time.sleep(0.2)
                shoot = [0,3,0,3]
                shoot_bytes = bytearray(shoot)
                ser.write(shoot_bytes)
                print("SHOOTING")



        print(f"Our position: {our_coords} \n")
        print(f"Our heading: {heading} \n")
        print(f"enemy_position {enemy_position} \n")
        #print(f"Distances: {distances}")
        
        #clear_to_drive = True
        """for d in distances:
            if d < 400:
                clear_to_drive = False
        
        if clear_to_drive:
            print("No danger: clear to drive")
        else:
            print("DANGER: Turn away")"""
        print(self.manual_car)
        relative_distance_vecs_cart = [(d_v[0] - o_d_v[0], d_v[1] - o_d_v[1]) for o_d_v, d_v in zip(self.old_coords, coords)]
        #self.get_logger().info(f" cart:{[(int(v[0]), int(v[1])) for v in relative_distance_vecs_cart[3:4]]}")
        relative_distance_vecs_polar = [(math.sqrt(math.pow(v[0], 2) + math.pow(v[1], 2)), math.acos(v[0]/(0.001 + math.sqrt(math.pow(v[0],2) + math.pow(v[1],2)))) * (math.copysign(1,v[1])*180/math.pi)) for v in relative_distance_vecs_cart]
        #self.get_logger().info(f"pol:{[(int(v[0]), int(v[1])) for v in relative_distance_vecs_polar[OUR_ID:OUR_ID+1]]}")
        velocity_vecs_polar = self.old_velocity_vecs
        for i, v in enumerate(relative_distance_vecs_polar):
            velocity = (v[0] / delta_t.total_seconds(), v[1])
            if velocity[0] < 500:
                velocity_vecs_polar[i] = velocity
        
        future_distance_vectors_polar = [(v[0] * MOTOR_DELAY_SECS, v[1]) for v in velocity_vecs_polar]
        future_distance_vectors_cart = [(v[0] * math.cos(v[1]), v[0] * math.sin(v[1])) for v in future_distance_vectors_polar] 
        
        #self.get_logger().info(f"speed:{[(int(v[0]), int(v[1])) for v in velocity_vecs_polar[3:4]]}")
                
        self.old_coords = coords
        
        #print(relative_distance_vecs_polar[OUR_ID:OUR_ID+1][0][1])
        #print(MOVE_TO)
        
        """speed_diff = old_speeds[0] - old_speeds[1]
        if abs(speed_diff) > 40:
            heading = relative_distance_vecs_polar[OUR_ID:OUR_ID+1][0][1] + math.copysign(1,speed_diff)*90
            heading = limit_heading(heading)
        else:
            heading = relative_distance_vecs_polar[OUR_ID:OUR_ID+1][0][1]
        """
        if self.going_home and not self.at_home:
            if our_coords[0][0] > 0 and our_coords[1][0] > 0 and len(self.path) > 0:
                #self.move_2_point(MOVE_TO, [relative_distance_vecs_polar[OUR_ID:OUR_ID+1][0][0], heading])
                print(f"Moving to: {self.path[0]} length: {len(self.path)}")
                if len(self.path) == 1:
                    self.MOVE_TO = (self.path[0])
                elif self.MOVE_TO == (0,0):
                    self.MOVE_TO = (self.path.pop(0))
                elif self.at_point:
                    self.MOVE_TO = (self.path.pop(0))
                    self.at_point = False
                self.move_2_point(self.MOVE_TO, our_coords[0], heading)
                self.old_heading = heading
                self.old_our_coords = our_coords[0]
            else:
                self.move_2_point([0,0],[1000,1000],1337)
        

#ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
#ser.reset_input_buffer()
def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    #building_subscriber = BuildingSubscriber()

    rclpy.spin(minimal_subscriber)
    #rclpy.spin(building_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    #building_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
