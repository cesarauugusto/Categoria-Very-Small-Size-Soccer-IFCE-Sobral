#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

# Importe o pacote 'rospy' e a mensagem 'Twist'
import rospy
from geometry_msgs.msg import Twist

# Crie um publisher para enviar comandos de velocidade
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# Crie um objeto Twist para armazenar os comandos de velocidade
cmd_msg = Twist()

# Função para enviar comandos de velocidade
def send_velocity_cmd(left_motor, right_motor):
    # Preencha os valores de velocidade
    cmd_msg.linear.x = left_motor  # Defina a velocidade linear conforme necessário
    cmd_msg.angular.z = right_motor  # Defina a velocidade angular conforme necessário

    # Envie os comandos de velocidade
    cmd_vel_pub.publish(cmd_msg)

def image_callback(msg):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower_blue = np.array([100, 100, 100])
    upper_blue = np.array([140, 255, 255])

    lower_green = np.array([0, 50, 100])
    upper_green = np.array([255, 255, 255])

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

        vector_direction = (cX_blue - cX_green, cY_blue - cY_green)

        print(f'Centro Azul (x, y): ({cX_blue}, {cY_blue}), Centro Verde (x, y): ({cX_green}, {cY_green}), Vetor de Direção: {vector_direction}')

        cv2.drawContours(cv_image, [blue_contour], -1, (255, 0, 0), 2)
        cv2.circle(cv_image, (cX_blue, cY_blue), 7, (255, 255, 255), -1)
        cv2.putText(cv_image,f' Azul ({cX_blue}, {cY_blue})', (cX_blue - 20, cY_blue - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.drawContours(cv_image, [green_contour], -1, (0, 255, 0), 2)
        cv2.circle(cv_image, (cX_green, cY_green), 7, (255, 255, 255), -1)
        cv2.putText(cv_image,f' Verde ({cX_green}, {cY_green})', (cX_green - 20, cY_green - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        cv2.arrowedLine(cv_image, (cX_green, cY_green), 
                        (cX_blue, cY_blue), (0, 0, 255), 2)

        # Adicione marcadores para frente, trás, esquerda e direita
        front_marker = (int(cX_green + 30*np.cos(np.arctan2(vector_direction[1], vector_direction[0]))),
                        int(cY_green + 30*np.sin(np.arctan2(vector_direction[1], vector_direction[0]))))
        back_marker = (int(cX_green - 30*np.cos(np.arctan2(vector_direction[1], vector_direction[0]))),
                       int(cY_green - 30*np.sin(np.arctan2(vector_direction[1], vector_direction[0]))))
        left_marker = (int(cX_green + 30*np.cos(np.arctan2(vector_direction[0], -vector_direction[1]))),
                       int(cY_green + 30*np.sin(np.arctan2(vector_direction[0], -vector_direction[1]))))
        right_marker = (int(cX_green - 30*np.cos(np.arctan2(vector_direction[0], -vector_direction[1]))),
                        int(cY_green - 30*np.sin(np.arctan2(vector_direction[0], -vector_direction[1]))))

        cv2.line(cv_image, front_marker, back_marker, (0, 255, 0), 2)
        cv2.line(cv_image, left_marker, right_marker, (0, 255, 0), 2)

        # Enviando comandos de velocidade para o ESP32
        left_motor = 0.2  # Defina a velocidade do motor esquerdo conforme necessário
        right_motor = 0.2  # Defina a velocidade do motor direito conforme necessário
        send_velocity_cmd(left_motor, right_motor)

    cv2.imshow("Imagem da Câmera", cv_image)
    cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('color_detection_node', anonymous=True)
    rospy.Subscriber('usb_cam/image_raw', Image, image_callback)
    rospy.spin()
