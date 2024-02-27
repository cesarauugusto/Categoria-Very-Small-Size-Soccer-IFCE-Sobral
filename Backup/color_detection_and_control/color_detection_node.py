#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, Float64MultiArray
from math import atan2, degrees

class ColorDetectionNode:
    def __init__(self):
        rospy.init_node('color_detection_node', anonymous=True)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.marker_pub = rospy.Publisher('/robot_marker', Marker, queue_size=1)
        self.vel_pub = rospy.Publisher('/velocidade', Float64MultiArray, queue_size=1)

        # Substituir os valores da cor amarela pelos da cor rosa
        self.lower_pink = np.array([145, 53, 184])
        self.upper_pink = np.array([161, 120, 255])

        self.lower_blue = np.array([0, 195, 131])
        self.upper_blue = np.array([176, 255, 255])

        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.5
        self.font_color = (255, 255, 255)
        self.font_thickness = 1

    def adjust_contrast(self, image, alpha=1.5, beta=0):
        adjusted_image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
        return adjusted_image

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        # Ajuste de contraste
        cv_image = self.adjust_contrast(cv_image, alpha=1, beta=0)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Substituir a cor amarela pela cor rosa
        mask_pink = cv2.inRange(hsv, self.lower_pink, self.upper_pink)
        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        pink_contours, _ = cv2.findContours(mask_pink, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(pink_contours) > 0:
            self.detected_pink = True
        else:
            self.detected_pink = False

        blue_contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(blue_contours) > 0:
            self.detected_blue = True
        else:
            self.detected_blue = False

        if self.detected_pink and self.detected_blue:
            pink_moments = cv2.moments(mask_pink)
            blue_moments = cv2.moments(mask_blue)

            pink_center = (int(pink_moments['m10'] / pink_moments['m00']), int(pink_moments['m01'] / pink_moments['m00']))
            blue_center = (int(blue_moments['m10'] / blue_moments['m00']), int(blue_moments['m01'] / blue_moments['m00']))

            combined_contours = pink_contours + blue_contours
            x, y, w, h = cv2.boundingRect(np.vstack(combined_contours))

            square_top_left = (x, y)
            square_bottom_right = (x + w, y + h)

            arrow_start = pink_center
            arrow_end = blue_center

            cv2.rectangle(cv_image, square_top_left, square_bottom_right, (0, 255, 255), 2)
            cv2.arrowedLine(cv_image, arrow_start, arrow_end, (0, 0, 255), 2)

            self.publish_robot_marker(Pose(Point(square_top_left[0], square_top_left[1], 0), (0, 0, 0, 1)), "square_start")
            self.publish_robot_marker(Pose(Point(square_bottom_right[0], square_bottom_right[1], 0), (0, 0, 0, 1)), "square_end")

            self.lower_orange = np.array([0, 56, 221])
            self.upper_orange = np.array([24, 131, 255])
            mask_orange = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
            orange_contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(orange_contours) > 0:
                orange_moments = cv2.moments(mask_orange)
                orange_center = (int(orange_moments['m10'] / orange_moments['m00']), int(orange_moments['m01'] / orange_moments['m00']))
                cv2.drawContours(cv_image, orange_contours, -1, (0, 165, 255), 2)

                square_center = (int((square_top_left[0] + square_bottom_right[0]) / 2), int((square_top_left[1] + square_bottom_right[1]) / 2))
                linear_velocity, angular_velocity, angle = self.calculate_velocities(orange_center, square_center)

                cv2.arrowedLine(cv_image, square_center, orange_center, (255, 255, 255), 2)

                angle_text = f'Angle: {angle:.2f} degrees'
                cv2.putText(cv_image, angle_text, (10, 20), self.font, self.font_scale, self.font_color, self.font_thickness, cv2.LINE_AA)

                # Publicar velocidades
                velocities = Float64MultiArray()
                velocities.data = [linear_velocity, angular_velocity]
                self.vel_pub.publish(velocities)

                # Adicionar texto na imagem
                vel_text = f'Linear: {linear_velocity:.2f}, Angular: {angular_velocity:.2f}'
                cv2.putText(cv_image, vel_text, (10, 40), self.font, self.font_scale, self.font_color, self.font_thickness, cv2.LINE_AA)

                # Mostrar as coordenadas da bola e do robô
                cv2.putText(cv_image, f'Bola: ({orange_center[0]}, {orange_center[1]})', (10, 60), self.font, self.font_scale, self.font_color, self.font_thickness, cv2.LINE_AA)
                cv2.putText(cv_image, f'Robo: ({blue_center[0]}, {blue_center[1]})', (10, 80), self.font, self.font_scale, self.font_color, self.font_thickness, cv2.LINE_AA)

        cv2.imshow("Color Detection", cv_image)
        cv2.waitKey(1)

    def calculate_velocities(self, ball_center, square_center):
        Kp_linear = 0.5
        Kp_angular = 0.5

        error_x = ball_center[0] - square_center[0]
        error_y = ball_center[1] - square_center[1]

        angle = atan2(error_y, error_x)
        angular_velocity = Kp_angular * angle
        linear_velocity = Kp_linear * np.sqrt(error_x**2 + error_y**2)

        return linear_velocity, angular_velocity, degrees(angle)

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

        self.marker_pub.publish(marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ColorDetectionNode()
    node.run()
