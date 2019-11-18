#rexarm.py
import numpy as np
from kinematics import *
import time

""" 
TODO:

Implement the missing functions
add anything you see fit

"""

""" Radians to/from  Degrees conversions """
D2R = 3.141592/180.0
R2D = 180.0/3.141592

class Rexarm():
    def __init__(self, joints, gripper):
        self.joints = joints
        self.gripper = gripper
        self.gripper_open_pos = np.deg2rad(-60.0)
        self.gripper_closed_pos = np.deg2rad(30.0)
        self.gripper_state = True
        self.estop = False
        """
        467TODO: 
        Find the physical angle limits of the Rexarm. Remember to keep track of this if you include more motors
        """
        angle_max = 95.0
        angle_min = -95.0
        self.angle_limits = np.array(
            [[angle_min, angle_min, angle_min, angle_min, angle_min],
             [angle_max, angle_max, angle_max, angle_max, angle_max]],
            dtype=np.float) * D2R

        """ Commanded Values """
        self.num_joints = len(joints)
        self.position = [0.0] * self.num_joints     # degrees
        self.speed = [1.0] * self.num_joints        # 0 to 1
        self.max_torque = [1.0] * self.num_joints   # 0 to 1

        """ Feedback Values """
        self.joint_angles_fb = [0.0] * self.num_joints # degrees
        self.speed_fb = [0.0] * self.num_joints        # 0 to 1   
        self.load_fb = [0.0] * self.num_joints         # -1 to 1  
        self.temp_fb = [0.0] * self.num_joints         # Celsius
        self.move_fb = [0] *  self.num_joints

        self.num_tries = 51200

    def initialize(self):
        id = -1
        for joint in self.joints:
            id += 1
            joint.enable_torque(num_tries=self.num_tries)
            joint.set_torque_limit(1.0, num_tries=self.num_tries)
            joint.set_speed(1.0, num_tries=self.num_tries)
            if id == 0:
                joint.set_position(-90.0, num_tries=self.num_tries)
                self.position[id] = -90.0
            elif id == 1:
                joint.set_position(0.0, num_tries=self.num_tries)
                self.position[id] = 0.0
            elif id == 2:
                joint.set_position(90.0, num_tries=self.num_tries)
                self.position[id] = 90.0
            print(id, self.position[id])
            
        if(self.gripper != 0):
            self.gripper.set_torque_limit(1.0, num_tries=self.num_tries)
            self.gripper.set_speed(0.8, num_tries=self.num_tries)
            self.close_gripper()

    def open_gripper(self):
        """ TODO """
        self.gripper_state = False
        pass

    def close_gripper(self):
        """ TODO """
        self.gripper_state = True
        pass

    def toggle_gripper(self):
        """ TODO """
        pass

    def set_positions(self, joint_angles, update_now = True):
        self.clamp(joint_angles)
        for i,joint in enumerate(self.joints):
            self.position[i] = joint_angles[i]
            if(update_now):
                joint.set_position(joint_angles[i], num_tries=self.num_tries)
    
    def set_speeds_normalized_global(self, speed, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speed
            if(update_now):
                joint.set_speed(speed, num_tries=self.num_tries)

    def set_speeds_normalized(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            if(update_now):
                joint.set_speed(speeds[i], num_tries=self.num_tries)

    def set_speeds(self, speeds, update_now = True):
        for i,joint in enumerate(self.joints):
            self.speed[i] = speeds[i]
            speed_msg = abs(speeds[i]/joint.max_speed)
            if (speed_msg < 3.0/1023.0):
                speed_msg = 3.0/1023.0
            if(update_now):
                joint.set_speed(speed_msg, num_tries=self.num_tries)
    
    def set_torque_limits(self, torques, update_now = True):
        for i,joint in enumerate(self.joints):
            self.max_torque[i] = torques[i]
            if(update_now):
                joint.set_torque_limit(torques[i], num_tries=self.num_tries)

    def send_commands(self):
        self.set_positions(self.position)
        self.set_speeds_normalized(self.speed)
        self.set_torque_limits(self.max_torque)

    def enable_torque(self):
        for joint in self.joints:
            joint.enable_torque(num_tries=self.num_tries)

    def disable_torque(self):
        for joint in self.joints:
            joint.disable_torque(num_tries=self.num_tries)

    def get_positions(self):
        for i,joint in enumerate(self.joints):
            self.joint_angles_fb[i] = joint.get_position(num_tries=self.num_tries)
        return self.joint_angles_fb

    def get_speeds(self):
        for i,joint in enumerate(self.joints):
            self.speed_fb[i] = joint.get_speed(num_tries=self.num_tries)
        return self.speed_fb

    def get_loads(self):
        for i,joint in enumerate(self.joints):
            self.load_fb[i] = joint.get_load(num_tries=self.num_tries)
        return self.load_fb

    def get_temps(self):
        for i,joint in enumerate(self.joints):
            self.temp_fb[i] = joint.get_temp(num_tries=self.num_tries)
        return self.temp_fb

    def get_moving_status(self):
        for i,joint in enumerate(self.joints):
            self.move_fb[i] = joint.is_moving(num_tries=self.num_tries)
        return self.move_fb

    def get_feedback(self):
        self.get_positions()
        self.get_speeds()
        self.get_loads()
        self.get_temps()
        self.get_moving_status()

    def pause(self, secs):
        time_start = time.time()
        while((time.time()-time_start) < secs):
            self.get_feedback()
            time.sleep(0.05)
            if(self.estop == True):
                break

    def clamp(self, joint_angles):
        return np.clip(joint_angles, self.angle_limits[0][:len(joint_angles)],
                       self.angle_limits[1][:len(joint_angles)])

    def get_wrist_pose(self):
        """TODO"""
        pos = FK_pox(self.joint_angles_fb)
        return [pos[0],pos[1],0,0,0,pos[2]]
