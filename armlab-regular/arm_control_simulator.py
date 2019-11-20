import time
import numpy as np
import lcm
import os
os.sys.path.append('lcmtypes/')
from lcmtypes import ball_t

def main():
	msg = ball_t()
	msg.position = [0.08, 0.13]
	lc = lcm.LCM()
	lc.publish("BALL_POSE", msg.encode())
	time.sleep(0.5)

if __name__ == '__main__':
    main()