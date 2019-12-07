import time
import numpy as np
import lcm
import os
os.sys.path.append('lcmtypes/')
from lcmtypes import ball_t
from lcmtypes import arm_path_t

def use_ball_t():
	msg = ball_t()
	msg.position = [0.1, 0.15]
	lc = lcm.LCM()
	nums = [6]
	for num in nums:
		inter = np.linspace(0.1,0.16, num)
		for i in inter:
			msg.position[1] = i
			lc.publish("BALL_POSE", msg.encode())
			#time.sleep(0.01)
		time.sleep(1)
		msg.position[1] = 0.1
		lc.publish("BALL_POSE", msg.encode())
		time.sleep(1)

def use_arm_path_t():
	num = 2
	inter = np.linspace(0.05, 0.16, num)
	msg = arm_path_t()
	msg.speed = 1
	msg.waypoints_num = 1
	#msg.waypoints = [[0.1, inter[0]]]
	msg.waypoints = [[0.1, 0.05, 2]]
	lc = lcm.LCM()
	lc.publish("ARM_PATH", msg.encode())
	time.sleep(4)
	msg.waypoints_num = num - 1
	msg.waypoints = []
	for i in range(1, num):
		msg.waypoints.append([0.1, inter[i]])
	msg.waypoints = [[-0.1, 0.05, 2]]
	lc.publish("ARM_PATH", msg.encode())

def test():
	msg = arm_path_t()
	msg.speed = 1
	msg.waypoints_num = 1
	#msg.waypoints = [[-0.1, 0.1]]
	#msg.waypoints = [[0.1, 0.08]]
	#msg.waypoints = [[0.19, 0.01]]
	msg.waypoints = [[-0.1, 0.1, 1]]
	lc = lcm.LCM()
	lc.publish("ARM_PATH", msg.encode())

def use_new_arm_path_t():
	msg = arm_path_t()
	msg.speed = 1
	msg.angles_num = 1
	#msg.waypoints = [[0.1, inter[0]]]
	msg.angles = [[- np.pi / 2, np.pi / 3, np.pi / 3, 0]]
	
	lc = lcm.LCM()
	lc.publish("ARM_PATH", msg.encode())
	time.sleep(3)
	msg.angles = [[- np.pi / 2, -np.pi / 3, -np.pi / 3, 0]]
	lc.publish("ARM_PATH", msg.encode())

def main():
	# use_ball_t()
	use_arm_path_t()
	#test()
	#use_new_arm_path_t()
	

if __name__ == '__main__':
    main()