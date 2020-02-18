import sys
import urx
import time
from urx.robotiq_two_finger_gripper import Robotiq_Two_Finger_Gripper

if __name__ == '__main__':
	rob = urx.Robot("192.168.1.105")
        time.sleep(0.2)
	robotiqgrip = Robotiq_Two_Finger_Gripper(rob)

	if(len(sys.argv) != 2):
		print "false"
		sys.exit()

	if(sys.argv[1] == "close") :
		robotiqgrip.close_gripper()
	if(sys.argv[1] == "open") :
		robotiqgrip.open_gripper()
        print(rob.getl())
	
	rob.close()
	print "true"
	sys.exit()
