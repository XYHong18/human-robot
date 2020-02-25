import urx
import time
import sys


if __name__ == '__main__':
        v = 0.05
        a = 0.01
        rob = urx.Robot("192.168.1.105")
        time.sleep(0.2)
        p1 = rob.getl()
        print ("Original pose is: ",  p1)
        rob.movel((0.4, 0.4, 0.3, 3, -1, 0), a, v)
        p2 = rob.getl()
        print ("Current tool pose is: ",  p2)
        sys.exit(0)
