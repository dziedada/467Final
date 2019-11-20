import time
import numpy as np
import lcm
import os
os.sys.path.append('lcmtypes/')
from lcmtypes import ball_t

def main():
	msg = ball_t()
	msg.position = [0.1, 0.15]
	lc = lcm.LCM()
	nums = [2, 3, 6]
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

if __name__ == '__main__':
    main()