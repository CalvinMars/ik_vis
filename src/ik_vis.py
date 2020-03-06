#!/usr/bin/env python

import pygame
import rospy
import threading
import time
import math
from sized_queue import SizedQueue
from sensor_msgs.msg import JointState
from multiprocessing import Queue

TURRET =        0
SHOULDER =      1
ELBOW =         2
WRIST =         3
WRIST_SPIN =    4
CLAW =          5

AFTARM_LENGTH =  10.25 * 8
FOREARM_LENGTH = 10.25 * 8
WRIST_LENGTH =   10.25 * 8

SHOULDER_OFFSET =   0
ELBOW_OFFSET =      0
WRIST_OFFSET =      90

WHITE =     (255, 255, 255)
BLACK =     (0, 0, 0)
GREEN =     (0, 255, 0)
BLUE =      (0, 0, 255)
RED =       (255, 0, 0)
YELLOW =    (255, 255, 0)

WIDTH =  600
HEIGHT = 600

class IkViz():
    def __init__(self):
        self.current_turret_pos = 0
        self.current_shoulder_pos = 0
        self.current_elbow_pos = 0
        self.current_wrist_pos = 0
        self.current_wrist_spin_pos = 0
        self.current_claw_pos = 0

        self.shoulder_queue = Queue()
        self.elbow_queue = Queue()
        self.wrist_queue = Queue()

        # Window setup
        pygame.init()
        pygame.font.init()

        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.width, self.height = self.screen.get_size()
        
        self.font = pygame.font.SysFont(None, 30)

        # ROS topic subscription
        ros_thread = threading.Thread(target = self.ros_subscribe)
        ros_thread.start()

        self.run_viz_loop()
        pygame.quit()
    
    def run_viz_loop(self):
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN or event.type == pygame.QUIT:
                    running = False

            shoulder_angle =    self.shoulder_queue.get()
            elbow_angle =       self.elbow_queue.get()
            wrist_angle =       self.wrist_queue.get()

            d = 90 - shoulder_angle
            e = elbow_angle - d
            f = 90 - e
            h = wrist_angle - 90 - f

            origin = (WIDTH / 2, HEIGHT / 2)

            shoulder_endpoint = (
                origin[0] + (self.sin(shoulder_angle) * AFTARM_LENGTH),
                origin[1] + (self.cos(shoulder_angle) * AFTARM_LENGTH))
            elbow_endpoint = (
                shoulder_endpoint[0] + (self.sin(e) * FOREARM_LENGTH),
                shoulder_endpoint[1] + (self.cos(e) * FOREARM_LENGTH))
            wrist_endpoint = (
                elbow_endpoint[0] + (self.sin(h) * WRIST_LENGTH),
                elbow_endpoint[1] + (self.cos(h) * WRIST_LENGTH))

            shoulder_origin = origin
            elbow_origin = shoulder_endpoint
            wrist_origin = elbow_endpoint

            self.screen.fill(WHITE)
            self.draw_arm_length(shoulder_origin, shoulder_endpoint)
            self.draw_arm_length(elbow_origin, elbow_endpoint)
            self.draw_arm_length(wrist_origin, wrist_endpoint)

            test_len = 50 * 8
            self.draw_line(shoulder_origin, test_len, 0, RED)
            self.draw_line(shoulder_origin, test_len, 90, BLUE)
            self.draw_line(shoulder_origin, test_len, 180, GREEN)
            self.draw_line(shoulder_origin, test_len, 270, YELLOW)
            
            pygame.display.update()

    def sin(self, angle_in_degrees):
        return math.sin(angle_in_degrees * math.pi / 180)

    def cos(self, angle_in_degrees):
        return math.cos(angle_in_degrees * math.pi / 180)

    def get_pretty_coords_str(self, coords):
        new_coords = (coords[0] - WIDTH / 2, coords[1] - HEIGHT / 2)
        return str(new_coords)
    
    def draw_arm_length(self, origin, endpoint):
        pygame.draw.line(self.screen, BLACK, origin, endpoint)
    
    # def get_endpoint(self, origin, length, angle):
    #     angle = angle * (math.pi / 180.0);
    #     end_x = (length * math.sin(angle)) + origin[0]
    #     end_y = (length * math.cos(angle)) + origin[1]
    #     return (end_x, end_y)

    def draw_line(self, origin, length, angle, color):
        endpoint = self.get_endpoint(origin, length, angle)
        pygame.draw.line(self.screen, color, origin, endpoint)

    def ros_subscribe(self):
        rospy.Subscriber('joint_states', JointState, self.pos_callback)
        rospy.spin()

    def pos_callback(self, data):
        #with self.lock:
        #    if self.exit:
        #        self.subscription.unregister()

        turret_pos = data.position[TURRET]
        shoulder_pos = data.position[SHOULDER]
        elbow_pos = data.position[ELBOW]
        wrist_pos = data.position[WRIST]
        wrist_spin_pos = data.position[WRIST_SPIN]
        claw_pos = data.position[CLAW]

        if turret_pos == self.current_turret_pos and shoulder_pos == self.current_shoulder_pos and elbow_pos == self.current_elbow_pos and wrist_pos == self.current_wrist_pos and wrist_spin_pos == self.current_wrist_spin_pos and claw_pos == self.current_claw_pos:
            return

        self.shoulder_queue.put(shoulder_pos)
        self.elbow_queue.put(elbow_pos)
        self.wrist_queue.put(wrist_pos)

        self.current_turret_pos = turret_pos
        self.current_shoulder_pos = shoulder_pos
        self.current_elbow_pos = elbow_pos
        self.current_wrist_pos = wrist_pos
        self.current_wrist_spin_pos = wrist_spin_pos
        self.current_claw_pos = claw_pos

if __name__ == '__main__':
    rospy.init_node('ikviz', log_level=rospy.DEBUG)
    ikViz = IkViz()
