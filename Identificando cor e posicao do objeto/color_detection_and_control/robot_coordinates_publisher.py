#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def publish_coordinates():
    rospy.init_node('robot_coordinates_publisher', anonymous=True)
    pub = rospy.Publisher('/robot_coordinates', String, queue_size=10)
    rate = rospy.Rate(10)  # Frequência de publicação (10 Hz)

    while not rospy.is_shutdown():
        # Substitua as variáveis `x` e `y` com as coordenadas reais do robô
        x = 0.0
        y = 0.0
        coordinates = f'({x},{y})'
        pub.publish(coordinates)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_coordinates()
    except rospy.ROSInterruptException:
        pass
