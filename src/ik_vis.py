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

AFTARM_LENGTH =  10
FOREARM_LENGTH = 10
WRIST_LENGTH =   3.5

SHOULDER_OFFSET =   -0.39735519842 + math.pi
ELBOW_OFFSET =      -0.365948283725 + math.pi / 2
WRIST_OFFSET =      0.763303482146 + math.pi / 2

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE =  (0, 0, 255)
RED =   (255, 0, 0)

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

        # Once the main loop exits, let the ROS thread know to stop
        #self.exit = True
        #rospy.signal_shutdown("Goodbye!")
    
    def run_viz_loop(self):
        point_queue = SizedQueue(100)

        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN or event.type == pygame.QUIT:
                    running = False

            shoulder_angle = self.shoulder_queue.get() + SHOULDER_OFFSET
            shoulder_origin = (WIDTH / 2, HEIGHT / 2)
            shoulder_endpoint = self.get_endpoint(shoulder_origin, AFTARM_LENGTH, shoulder_angle)

            elbow_angle = self.elbow_queue.get() + ELBOW_OFFSET
            elbow_origin = shoulder_endpoint
            elbow_endpoint = self.get_endpoint(elbow_origin, FOREARM_LENGTH, elbow_angle)

            wrist_angle = self.wrist_queue.get() + WRIST_OFFSET
            wrist_origin = elbow_endpoint
            wrist_endpoint = self.get_endpoint(wrist_origin, WRIST_LENGTH, wrist_angle)

            point_queue.put(wrist_endpoint)

            #rospy.loginfo("shoulder:\t\t" + str(shoulder_angle))
            #rospy.loginfo("elbow:\t\t" + str(elbow_angle))
            #rospy.loginfo("wrist:\t\t" + str(wrist_angle))            
            #self.text = self.font.render(message, True, (10, 200, 10))
            #self.screen.blit(self.text, (WIDTH / 2, HEIGHT / 2))

            self.screen.fill(WHITE)
            self.draw_arm_length(shoulder_origin, shoulder_endpoint)
            self.draw_arm_length(elbow_origin, elbow_endpoint)
            self.draw_arm_length(wrist_origin, wrist_endpoint)

            # if point_queue.full():
            #    pygame.draw.line(self.screen, RED, wrist_endpoint, point_queue.get())

            pygame.display.update()
    
    def draw_arm_length(self, origin, endpoint):
        pygame.draw.line(self.screen, BLACK, origin, endpoint)
        pygame.draw.line(self.screen, BLUE, (endpoint[0], origin[1]), endpoint)
        pygame.draw.line(self.screen, GREEN, origin, (endpoint[0], origin[1]))
    
    def get_endpoint(self, origin, length, angle):
        length *= 15
        end_x = (length * math.sin(angle)) + origin[0]
        end_y = (length * math.cos(angle)) + origin[1]
        return (end_x, end_y)

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
