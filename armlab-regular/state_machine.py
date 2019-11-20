import time
import numpy as np
from kinematics import *
import lcm
import os
import threading
os.sys.path.append('lcmtypes/')
from lcmtypes import ball_t
"""
TODO: Add states and state functions to this class
        to implement all of the required logic for the armlab
"""
class StateMachine():
    def __init__(self, rexarm, planner, kinect):
        self.rexarm = rexarm
        self.tp = planner
        self.kinect = kinect
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.test = 0
        self.togo = []
        self.togo_lock = threading.Lock()
        self.lc = lcm.LCM()
        lcmBallPoseSub = self.lc.subscribe("BALL_POSE",
                                           self.ball_pose_handler)

    def ball_pose_handler(self, channel, data):
        msg = ball_t.decode(data)
        self.togo_lock.acquire()
        self.togo.append(msg.position)
        if len(self.togo) == 1:
            self.next_state = "move"
        self.togo_lock.release()
        print(msg.position)

    def set_next_state(self, state):
        self.next_state = state

    """ This function is run continuously in a thread"""

    def run(self):
        if(self.current_state == "manual"):
            if (self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()                
            if(self.next_state == "estop"):
                self.estop()

        if(self.current_state == "idle"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
            if(self.next_state == "move"):
                self.move()

        if(self.current_state == "move"):
            if(self.next_state == "manual"):
                self.manual()
            if(self.next_state == "idle"):
                self.idle()
            if(self.next_state == "estop"):
                self.estop()
            if(self.next_state == "calibrate"):
                self.calibrate()
                
        if(self.current_state == "estop"):
            self.next_state = "estop"
            self.estop()  

        if(self.current_state == "calibrate"):
            if(self.next_state == "idle"):
                self.idle()
        
        self.listen_to_command()     

    """Functions run for each state"""

    def listen_to_command(self):
        self.lc.handle_timeout(10)

    def manual(self):
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"
        self.rexarm.send_commands()
        self.rexarm.get_feedback()

    def move(self):
        self.status_message = "State: Move - move to a target point"
        self.current_state = "move"
        self.togo_lock.acquire()
        target = self.togo.pop(0)
        angles = IK((target[0], target[1], 0))
        print(angles)
        self.rexarm.move_to_target_angles(angles)
        if not len(self.togo):
            self.next_state = "idle"
        self.togo_lock.release()
    def idle(self):
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"
        self.test += 1
        # if self.test == 20:
        #     angles = IK((0.08, 0.1, 0))
        #     print(angles)
        #     self.rexarm.move_to_target_angles(angles)
        # if self.test == 25:
        #     angles = IK((0.08, 0.12, 0))
        #     print(angles)
        #     self.rexarm.move_to_target_angles(angles)
        # if self.test == 26:
        #     angles = IK((0.08, 0.13, 0))
        #     print(angles)
        #     self.rexarm.move_to_target_angles(angles)
        # if self.test == 27:
        #     angles = IK((0.08, 0.14, 0))
        #     print(angles)
        #     self.rexarm.move_to_target_angles(angles)
        # if self.test == 28:
        #     angles = IK((0.08, 0.15, 0))
        #     print(angles)
        #     self.rexarm.move_to_target_angles(angles)
        # if self.test == 29:
        #     angles = IK((0.08, 0.16, 0))
        #     print(angles)
        #     self.rexarm.move_to_target_angles(angles)
        # if self.test == 30:
        #     angles = IK((0.08, 0.17, 0))
        #     print(angles)
        #     self.rexarm.move_to_target_angles(angles)
        self.rexarm.get_feedback()

    def estop(self):
        self.status_message = "EMERGENCY STOP - Check Rexarm and restart program"
        self.current_state = "estop"
        self.rexarm.disable_torque()
        self.rexarm.get_feedback()
        
    def calibrate(self):
        self.current_state = "calibrate"
        self.next_state = "idle"
        self.tp.go(max_speed=2.0)
        location_strings = ["lower left corner of board",
                            "upper left corner of board",
                            "upper right corner of board",
                            "lower right corner of board",
                            "center of shoulder motor"]
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in RGB image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.rgb_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False        
        
        i = 0
        for j in range(5):
            self.status_message = "Calibration - Click %s in depth image" % location_strings[j]
            while (i <= j):
                self.rexarm.get_feedback()
                if(self.kinect.new_click == True):
                    self.kinect.depth_click_points[i] = self.kinect.last_click.copy()
                    i = i + 1
                    self.kinect.new_click = False
   
        print(self.kinect.rgb_click_points)
        print(self.kinect.depth_click_points)

        """TODO Perform camera calibration here"""

        self.status_message = "Calibration - Completed Calibration"
        time.sleep(1)