#!/usr/bin/env python

import pygame
import rospy
import threading
import time
from sensor_msgs.msg import JointState
from Queue import Queue

TURRET =        0
SHOULDER =      1
ELBOW =         2
WRIST =         3
WRIST_SPIN =    4
CLAW =          5

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

WIDTH = 600
HEIGHT = 600

class IkViz():
    
    def __init__(self):
        self.current_turret_pos = 0
        self.current_shoulder_pos = 0
        self.current_elbow_pos = 0
        self.current_wrist_pos = 0
        self.current_wrist_spin_pos = 0
        self.current_claw_pos = 0
        self.exit = False

        self.shoulder_queue = Queue()

        self.lock = threading.Lock()

        # ros_thread.join();

        # Window setup
        pygame.init()
        pygame.font.init()

        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        #self.screen.fill((255, 0, 0))
        width, height = self.screen.get_size()
        self.width = width
        self.height = height
        
        self.font = pygame.font.SysFont(None, 30)

        time.sleep(2)

        # pygame.draw.rect(screen, (0,128,255), pygame.Rect(30,30,60,60))
        # pygame.display.update()

        #self.run_viz_loop()
        
        # ROS topic subscription
        ros_thread = threading.Thread(target = self.ros_subscribe)
        ros_thread.start()

        self.run_viz_loop()

        # Once the main loop exits, let the ROS thread know to stop
        #self.exit = True
        #rospy.signal_shutdown("Goodbye!")
    
    def run_viz_loop(self):
        while True:
            # for event in pygame.event.get():
            #     if event.type == pygame.KEYDOWN:
            #         break

            message = str(self.shoulder_queue.get())
            rospy.loginfo("message:\t" + message)
            
            self.screen.fill(WHITE)
            self.text = self.font.render(message, True, (10, 200, 10))
            self.screen.blit(self.text, (WIDTH / 2, HEIGHT / 2))
            pygame.display.update()


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

        #rospy.loginfo(str(shoulder_pos) + "\t" + str(elbow_pos) + "\t" + str(wrist_pos))

        self.shoulder_queue.put(shoulder_pos)

        self.current_turret_pos = turret_pos
        self.current_shoulder_pos = shoulder_pos
        self.current_elbow_pos = elbow_pos
        self.current_wrist_pos = wrist_pos
        self.current_wrist_spin_pos = wrist_spin_pos
        self.current_claw_pos = claw_pos

if __name__ == '__main__':
    rospy.init_node('ikviz', log_level=rospy.DEBUG)
    ikViz = IkViz()
