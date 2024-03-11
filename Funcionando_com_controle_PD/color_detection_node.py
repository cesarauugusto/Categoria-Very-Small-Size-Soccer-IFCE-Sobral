#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float64MultiArray
from math import atan2, sin, cos, degrees

class ColorDetectionNode:
    def __init__(self):
        rospy.init_node('color_detection_node', anonymous=True)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.marker_pub = rospy.Publisher('/robot_marker', MarkerArray, queue_size=1)
        self.vel_pub = rospy.Publisher('/velocidade', Twist, queue_size=1)
        self.wheel_vel_pub = rospy.Publisher('/velocidade_motores', Float64MultiArray, queue_size=1)

        # Define as faixas de cores
        self.lower_pink = np.array([145, 53, 184])
        self.upper_pink = np.array([161, 120, 255])

        self.lower_blue = np.array([0, 195, 131])
        self.upper_blue = np.array([176, 255, 255])

        self.lower_orange = np.array([0, 56, 221])
        self.upper_orange = np.array([24, 131, 255])

        # Configurações de texto
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.5
        self.font_color = (255, 255, 255)
        self.font_thickness = 1

        # Parâmetros de controle
        self.wheel_distance = 0.075  # Distância entre as rodas do robô (em metros)

        # Parâmetros de controle do movimento
        self.target_distance_threshold = 0.1  # Limiar de distância para considerar que o robô chegou ao alvo
        self.linear_speed = 0.1  # Velocidade linear do robô (metros por segundo)
        self.angular_speed = 0.5  # Velocidade angular do robô (radianos por segundo)

        # Variáveis para controle PID
        self.last_error = 0

        # Fator de conversão de pixels para metros
        self.pixels_per_meter = 100

    def adjust_contrast(self, image, alpha=1.5, beta=0):
        adjusted_image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        return adjusted_image

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Ajuste de contraste
        cv_image = self.adjust_contrast(cv_image, alpha=1, beta=0)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Mascara para as cores
        mask_pink = cv2.inRange(hsv, self.lower_pink, self.upper_pink)
        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask_orange = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        pink_contours, _ = cv2.findContours(mask_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Verifica a detecção das cores
        detected_pink = len(pink_contours) > 0
        detected_blue = len(blue_contours) > 0
        detected_orange = len(orange_contours) > 0

        distance = 0

        if detected_pink and detected_blue and detected_orange:
            pink_moments = cv2.moments(mask_pink)
            blue_moments = cv2.moments(mask_blue)
            orange_moments = cv2.moments(mask_orange)

            pink_center = (int(pink_moments['m10'] / pink_moments['m00']), int(pink_moments['m01'] / pink_moments['m00']))
            blue_center = (int(blue_moments['m10'] / blue_moments['m00']), int(blue_moments['m01'] / blue_moments['m00']))
            orange_center = (int(orange_moments['m10'] / orange_moments['m00']), int(orange_moments['m01'] / orange_moments['m00']))

            # Calcula a distância entre a bola e o robô
            distance = self.calculate_distance_cm(orange_center, blue_center)

            # Encontra o retângulo que envolve todas as áreas detectadas
            x, y, w, h = cv2.boundingRect(np.vstack(pink_contours + blue_contours))

            # Desenha o retângulo na imagem
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 255), 2)

            # Desenha linhas
            cv2.arrowedLine(cv_image, blue_center, pink_center, (0, 0, 255), 2)
            cv2.arrowedLine(cv_image, (x + w // 2, y + h // 2), orange_center, (255, 255, 255), 2)

            # Publica marcadores
            self.publish_robot_marker(Pose(Point(x, y, 0), (0, 0, 0, 1)), "square_start")
            self.publish_robot_marker(Pose(Point(x + w, y + h, 0), (0, 0, 0, 1)), "square_end")

            # Calcula velocidades lineares e angulares
            square_center = (x + w // 2, y + h // 2)
            linear_velocity, angular_velocity = self.calculate_velocities(orange_center, blue_center, square_center)

            # Calcula velocidades dos motores
            left_wheel_vel, right_wheel_vel = self.calculate_wheel_velocities(linear_velocity, angular_velocity, distance)

            # Publica velocidades dos motores
            wheel_vel_msg = Float64MultiArray(data=[left_wheel_vel, right_wheel_vel])
            self.wheel_vel_pub.publish(wheel_vel_msg)

            # Adiciona texto na imagem
            angle_text = f'Vel angular {angular_velocity:.2f} rad/s'
            cv2.putText(cv_image, angle_text, (10, 20), self.font, self.font_scale, self.font_color, self.font_thickness, cv2.LINE_AA)

            vel_text = f'Motor Left: {left_wheel_vel:.2f}, Motor Right: {right_wheel_vel:.2f}'
            cv2.putText(cv_image, vel_text, (10, 40), self.font, self.font_scale, self.font_color, self.font_thickness, cv2.LINE_AA)

            # Mostra coordenadas da bola, do robô e a distância entre eles
            cv2.putText(cv_image, f'Bola: ({orange_center[0]}, {orange_center[1]})', (10, 60), self.font, self.font_scale, self.font_color, self.font_thickness, cv2.LINE_AA)
            cv2.putText(cv_image, f'Robo: ({blue_center[0]}, {blue_center[1]})', (10, 80), self.font, self.font_scale, self.font_color, self.font_thickness, cv2.LINE_AA)
            cv2.putText(cv_image, f'Distancia: {distance:.2f} cm', (10, 100), self.font, self.font_scale, self.font_color, self.font_thickness, cv2.LINE_AA)

        cv2.imshow("Color Detection", cv_image)
        cv2.waitKey(1)

    def calculate_velocities(self, ball_center, robot_center, square_center):
        # Calculate angle between robot orientation and desired orientation
        desired_orientation = atan2(square_center[1] - robot_center[1], square_center[0] - robot_center[0])
        robot_orientation = atan2(ball_center[1] - robot_center[1], ball_center[0] - robot_center[0])
        angular_difference = atan2(sin(desired_orientation - robot_orientation), cos(desired_orientation - robot_orientation))

        # Calculate linear velocity
        linear_velocity = self.linear_speed

        # Calculate angular velocity
        angular_velocity = angular_difference * self.angular_speed

        return linear_velocity, angular_velocity

    def calculate_distance_cm(self, point1, point2):
        distance_pixels = np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
        distance_meters = distance_pixels / self.pixels_per_meter
        distance_cm = distance_meters * 100
        return distance_cm

    def calculate_wheel_velocities(self, linear_velocity, angular_velocity, distance):
        max_velocity = 60
        angular_velocity *= 10
        # Parâmetros de controle PID
        kp = 1.9
        kd = 0.5

        # Calcula o erro
        error = angular_velocity

        if error != 0 and distance>130:
            # Calcula o termo derivativo
            derivative = (error - self.last_error) * kd

            # Calcula a velocidade
            velocity = error * kp + derivative

            if(velocity > max_velocity):
                velocity = max_velocity

            if(velocity < -max_velocity):
                velocity = -max_velocity

            # Calcula as velocidades dos motores
            left_wheel_vel = max_velocity + velocity
            right_wheel_vel = max_velocity - velocity

            # Atualiza o erro anterior
            self.last_error = error
        
        elif distance < 130:
            # Define as velocidades dos motores como zero para parar o robô
            right_wheel_vel = 0
            left_wheel_vel = 0

        return left_wheel_vel, right_wheel_vel

    def publish_robot_marker(self, pose, id):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.02
        marker.scale.z = 0.02
        marker.color = ColorRGBA(1, 1, 1, 1)
        marker.pose = pose
        marker.id = id

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ColorDetectionNode()
    node.run()
