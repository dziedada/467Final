import numpy as np 
import time

"""
TODO: build a trajectory generator and waypoint planner 
        so it allows your state machine to iterate through
        the plan at the desired command update rate
"""

class TrajectoryPlanner():
    def __init__(self, rexarm):
        self.idle = True
        self.rexarm = rexarm
        self.num_joints = rexarm.num_joints
        self.initial_wp = [0.0]*self.num_joints
        self.final_wp = [0.0]*self.num_joints 
        self.dt = 0.05 # command rate
    
    def set_initial_wp(self):
        pass

    def set_final_wp(self, waypoint):
        pass

    def go(self, max_speed = 2.5):
        pass

    def stop(self):
        pass

    def calc_time_from_waypoints(self, initial_wp, final_wp, max_speed):
        pass

    def generate_cubic_spline(self, initial_wp, final_wp, T):
        pass

    def execute_plan(self, plan, look_ahead=8):
        pass