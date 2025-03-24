#!/usr/bin/env python3
# This file is part of rosgpt package.

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import copy
import math
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
import time
from rclpy.executors import SingleThreadedExecutor
import threading
from concurrent.futures import ThreadPoolExecutor


class TurtlesimController(Node):

    def __init__(self):
        super().__init__('turtlesim_controller')
        self.create_subscription(String,'/voice_cmd',self.voice_cmd_callback,10)
        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.x = 0.0
        self.y  = 0.0
        self.theta  = 0.0
        self.pose = Pose()
        self.clockwise = True

        self.thread_executor = ThreadPoolExecutor(max_workers=1)

        self.move_executor = SingleThreadedExecutor()
        move_thread = threading.Thread(target=self.move_executor.spin)
        move_thread.start()
        print('ROSGPT Turtlesim Controller Started. Waiting for input commands ...')

    def pose_callback(self, msg):
        self.x = msg.x
        self.y  = msg.y
        self.theta  = msg.theta
        self.pose = msg
        #print (self.x) #used for debegging
    
    #this callback represents the ROSGPTParser. It takes a JSON, parses it, and converts it to a ROS 2 command
    def voice_cmd_callback(self, msg):
        #print(msg.data)
        try:
            cmd = json.loads(msg.data)
            cmd = json.loads(cmd['json']) #we only consider the pure json message. cmd['text'] contains a mix of text and json
            print('JSON command received: \n',cmd,'\n')
            if cmd['action'] == 'go_to_goal':
                location = cmd['params']['location']['value']
                self.go_to_goal(location)
            elif cmd['action'] == 'move':
                linear_speed = cmd['params'].get('linear_speed', 0.2)
                distance = cmd['params'].get('distance', 1.0)
                is_forward = cmd['params'].get('is_forward', True)

                # Create a thread executor
                # we need to run the method on a different thread to avoid blocking rclpy.spin. 
                self.thread_executor.submit(self.move, linear_speed, distance, is_forward)

                # running move on the main thread will generate to error, as it will block rclpy.spin
                #self.move(linear_speed, distance, is_forward)
            elif cmd['action'] == 'rotate':
                angular_velocity = cmd['params'].get('angular_velocity', 1.0)
                angle = cmd['params'].get('angle', 90.0)
                is_clockwise = cmd['params'].get('is_clockwise', True)

                # Create a thread executor
                # we need to run the method on a different thread to avoid blocking rclpy.spin. 
                self.thread_executor.submit(self.rotate, angular_velocity, angle, is_clockwise)

                #self.rotate(angular_velocity, angle, is_clockwise)
        except json.JSONDecodeError:
            print('[json.JSONDecodeError] Invalid or empty JSON string received:', msg.data)
        except Exception as e:
            print('[Exception] An unexpected error occurred:', str(e))   

    def go_to_goal(self, location):
        # TODO: Implement go_to_goal method
        # wil be defined later
        pass

    def get_distance(self,start, destination):
        return math.sqrt(((destination.x-start.x)**2 + (destination.y-start.y)**2))
    
    def move(self, linear_speed, distance, is_forward): 
        if is_forward:
            direction = 'forward'
        else:
            direction = 'backward'
        print('Start moving the robot ', direction, ' at ', linear_speed, 'm/s and for a distance ', distance, 'meter')
        twist_msg = Twist()

        if (linear_speed > 1.0):
            print('[ERROR]: The speed must be lower than 1.0!')
            return -1

        twist_msg.linear.x = float(abs(linear_speed) if is_forward else -abs(linear_speed))
        twist_msg.linear.x = float(abs(linear_speed) * (1 if is_forward else -1))

        start_pose = copy.copy(self.pose)

        while self.get_distance(start_pose, self.pose) < distance:
            print(f'distance moved: {self.get_distance(start_pose, self.pose):.2f}')
            self.velocity_publisher.publish(twist_msg)
            self.move_executor.spin_once(timeout_sec=0.1)

        twist_msg.linear.x = 0.0
        self.velocity_publisher.publish(twist_msg)
        print(f'distance moved: {self.get_distance(start_pose, self.pose):.2f}')
        print('The Robot has stopped...')

    def rotate(self, angular_velocity, angle, is_clockwise):
        print('Start rotating the robot at ', angular_velocity, 'degrees/s for angle ', angle, 'degrees')
        twist_msg = Twist()
        
        # Update the clockwise direction from the parameter
        self.clockwise = is_clockwise
        
        angular_speed_degree = abs(angular_velocity)  # make sure it is a positive relative angle
        if (angular_speed_degree > 30):
            print(angular_speed_degree)
            print('[ERROR]: The rotation speed must be lower than 30 degrees/s!')
            return -1
            
        angular_speed_radians = math.radians(angular_speed_degree)
        twist_msg.angular.z = abs(angular_speed_radians) * (-1 if self.clockwise else 1)

        start_pose = copy.copy(self.pose)
        rotated_related_angle_degree = 0.0
        desired_relative_angle_degree = angle  # Use the provided angle parameter

        while rotated_related_angle_degree < desired_relative_angle_degree:
            self.velocity_publisher.publish(twist_msg)
            rotated_related_angle_degree = math.degrees(abs(start_pose.theta - self.pose.theta))
            print(f"start: {math.degrees(start_pose.theta):.2f}, self: {math.degrees(self.pose.theta):.2f}, rotated: {rotated_related_angle_degree:.2f}")
            time.sleep(0.1)
            
        twist_msg.angular.z = 0.0
        self.velocity_publisher.publish(twist_msg)
        print('The Robot has stopped rotating...')

    
def main(args=None):
    rclpy.init(args=args)
    node = TurtlesimController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
