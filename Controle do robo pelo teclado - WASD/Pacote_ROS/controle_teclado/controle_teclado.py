#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
import sys, select, termios, tty

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def control_robot():
    pub = rospy.Publisher('controle_robo', String, queue_size=10)
    rospy.init_node('controle_teclado', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        key = getKey()
        if key == 'w':
            pub.publish("frente")
        elif key == 's':
            pub.publish("tras")
        elif key == 'a':
            pub.publish("esquerda")
        elif key == 'd':
            pub.publish("direita")
        elif key == ' ':
            pub.publish("parar")
        elif key == '\x03':
            break

if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    try:
        control_robot()
    except rospy.ROSInterruptException:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
