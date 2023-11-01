#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import math

cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
cmd_msg = Twist()

def send_velocity_cmd(left_motor, right_motor):
    cmd_msg.linear.x = left_motor
    cmd_msg.angular.z = right_motor
    cmd_vel_pub.publish(cmd_msg)

def calculate_directions(cX_blue, cY_blue, cX_green, cY_green):
    vector_direction = (cX_blue - cX_green, cY_blue - cY_green)
    angle = math.atan2(vector_direction[1], vector_direction[0])
    angle = math.degrees(angle) % 360
    return angle, vector_direction

def update_marker_orientation(cv_image, cX_green, cY_green, width, length):
    frente_x = cX_green
    frente_y = cY_green - int(length/2)
    tras_x = cX_green
    tras_y = cY_green + int(length/2)
    esquerda_x = cX_green - int(width/2)
    esquerda_y = cY_green
    direita_x = cX_green + int(width/2)
    direita_y = cY_green

    cv2.line(cv_image, (frente_x, frente_y), (tras_x, tras_y), (0, 255, 0), 2)  # Frente para Trás (verde)
    cv2.line(cv_image, (esquerda_x, esquerda_y), (direita_x, direita_y), (0, 255, 0), 2)  # Esquerda para Direita (verde)

    cv2.circle(cv_image, (frente_x, frente_y), 7, (255, 255, 255), -1)
    cv2.circle(cv_image, (tras_x, tras_y), 7, (255, 255, 255), -1)
    cv2.circle(cv_image, (esquerda_x, esquerda_y), 7, (255, 255, 255), -1)
    cv2.circle(cv_image, (direita_x, direita_y), 7, (255, 255, 255), -1)

    cv2.putText(cv_image, 'Frente', (frente_x - 20, frente_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(cv_image, 'Tras', (tras_x - 20, tras_y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(cv_image, 'Esquerda', (esquerda_x - 40, esquerda_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    cv2.putText(cv_image, 'Direita', (direita_x, direita_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([140, 255, 255])

    lower_green = np.array([35, 100, 100])  # Ajuste os valores para a cor verde
    upper_green = np.array([85, 255, 255])  # Ajuste os valores para a cor verde

    mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
    mask_green = cv2.inRange(hsv_image, lower_green, upper_green)

    contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours_blue) > 0 and len(contours_green) > 0:
        blue_contour = max(contours_blue, key=cv2.contourArea)
        M_blue = cv2.moments(blue_contour)
        cX_blue = int(M_blue["m10"] / M_blue["m00"])
        cY_blue = int(M_blue["m01"] / M_blue["m00"])

        green_contour = max(contours_green, key=cv2.contourArea)
        M_green = cv2.moments(green_contour)
        cX_green = int(M_green["m10"] / M_green["m00"])
        cY_green = int(M_green["m01"] / M_green["m00"])

        width = 50
        length = 100

        update_marker_orientation(cv_image, cX_green, cY_green, width, length)

        angle, vector_direction = calculate_directions(cX_blue, cY_blue, cX_green, cY_green)
        print(f'Centro Azul (x, y): ({cX_blue}, {cY_blue}), Centro Verde (x, y): ({cX_green}, {cY_green}), Ângulo: {angle}, Vetor de Direção: {vector_direction}')

        left_motor = 0.2
        right_motor = 0.2
        send_velocity_cmd(left_motor, right_motor)

    cv2.imshow("Imagem da Câmera", cv_image)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('color_detection_node', anonymous=True)
    rospy.Subscriber('usb_cam/image_raw', Image, image_callback)
    rospy.spin()
