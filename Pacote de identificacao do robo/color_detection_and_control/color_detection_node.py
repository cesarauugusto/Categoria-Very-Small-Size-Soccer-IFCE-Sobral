#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from math import atan2, degrees

class ColorDetectionNode:
    def __init__(self):
        rospy.init_node('color_detection_node', anonymous=True)
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.marker_pub = rospy.Publisher('/robot_marker', Marker, queue_size=1)

        self.lower_yellow = np.array([159, 79, 144])
        self.upper_yellow = np.array([179, 255, 255])

        self.lower_blue = np.array([103, 105, 124])
        self.upper_blue = np.array([179, 255, 255])

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        # Converta a imagem para o espaço de cores HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Crie máscaras para as cores amarela e azul
        mask_yellow = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

        # Detecte as áreas de amarelo
        yellow_contours, _ = cv2.findContours(mask_yellow, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(yellow_contours) > 0:
            self.detected_yellow = True
        else:
            self.detected_yellow = False

        # Detecte as áreas de azul
        blue_contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(blue_contours) > 0:
            self.detected_blue = True
        else:
            self.detected_blue = False

        # Determine o lado esquerdo (amarelo) e o lado direito (azul)
        if self.detected_yellow and self.detected_blue:
            yellow_moments = cv2.moments(mask_yellow)
            blue_moments = cv2.moments(mask_blue)

            yellow_center = (int(yellow_moments['m10'] / yellow_moments['m00']), int(yellow_moments['m01'] / yellow_moments['m00']))
            blue_center = (int(blue_moments['m10'] / blue_moments['m00']), int(blue_moments['m01'] / blue_moments['m00']))
            
            # Calcule o retângulo envolvente para ambas as cores
            x, y, w, h = cv2.boundingRect(np.vstack(yellow_contours + blue_contours))
            
            # Calcule as coordenadas das extremidades do quadrado
            square_top_left = (x, y)
            square_bottom_right = (x + w, y + h)
            
            # Calcule a posição da seta
            arrow_start = yellow_center
            arrow_end = blue_center

            cv2.rectangle(cv_image, square_top_left, square_bottom_right, (0, 255, 255), 2)
            cv2.arrowedLine(cv_image, arrow_start, arrow_end, (0, 0, 255), 2)

            # Publicar marcadores para indicar as extremidades do quadrado
            self.publish_robot_marker(Pose(Point(square_top_left[0], square_top_left[1], 0), (0, 0, 0, 1)), "square_start")
            self.publish_robot_marker(Pose(Point(square_bottom_right[0], square_bottom_right[1], 0), (0, 0, 0, 1)), "square_end")

        # Exibir a imagem resultante
        cv2.imshow("Color Detection", cv_image)
        cv2.waitKey(1)

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
    while not rospy.is_shutdown():
        try:
            node.run()
        except KeyboardInterrupt:
            print("Shutting down...")
            cv2.destroyAllWindows()