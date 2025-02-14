import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray, ColorRGBA
from math import atan2, sin, cos, sqrt
import threading
import time
from collections import deque

class ColorDetectionNode:
    def __init__(self):
        rospy.init_node('color_detection_node', anonymous=True)
        
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()

        # Publicadores
        self.marker_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=1)
        self.trajectory_pub = rospy.Publisher('/robot_trajectory', Marker, queue_size=1)
        self.wheel_vel_pub = rospy.Publisher('/velocidade_motores', Float64MultiArray, queue_size=1)

        # ------------------ FAIXAS DE CORES ------------------
        self.lower_green  = np.array([61, 86, 135])
        self.upper_green  = np.array([95, 255, 255])
        self.lower_blue   = np.array([0, 165, 135])
        self.upper_blue   = np.array([140, 255, 255])
        self.lower_orange = np.array([0, 38, 244])
        self.upper_orange = np.array([95, 255, 255])

        # Faixa de cor para o branco (linhas do campo) [Original]
        self.lower_white = np.array([0, 0, 195])
        self.upper_white = np.array([147, 30, 255])

        # Faixa de cor para o preto (campo)
        self.lower_black = np.array([42, 23, 0])
        self.upper_black = np.array([179, 255, 255])

        # ------------------ CONFIGURAÇÕES DE TEXTO ------------------
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.9
        self.font_color = (0, 255, 0)
        self.font_thickness = 2

        # ------------------ PARÂMETROS DE MOVIMENTO ------------------
        self.wheel_distance = 0.075
        self.target_distance_threshold = 0.04
        self.linear_speed  = 0.1
        self.angular_speed = 0.5

        # ------------------ PID ------------------
        self.last_error = 0

        # ------------------ DISTÂNCIA/STATUS ------------------
        self.last_distance = None 
        self.robot_blue_center = None
        self.cont = 0   # contador de parada
        self.rotation = 1  # 1 => robô em movimento, 0 => parado

        # ------------------ TRAJETÓRIA DO ROBÔ ------------------
        self.trajectory_points = []  

        # ------------------ TRAJETÓRIA DA BOLA ------------------
        self.ball_traj_points = []   

        # ------------------ CONVERSÃO DE PIXELS->METROS ------------------
        self.pixels_per_meter = 250

        # ------------------ TAMANHO DA IMAGEM ------------------
        self.width  = 640
        self.height = 480

        # ------------------ FILTRO GAUSSIANO ------------------
        self.gaussian_kernel_size = (3, 3)
        self.gaussian_sigma = 1

        # ------------------ ÁREA MÍN./MÁX. ------------------
        self.min_area = 19000 
        self.max_area = 20000
        self.min_area_green = 19000
        self.max_area_green = 20500
        self.min_area_black = 16000
        self.max_area_black = 500000

        # ------------------ ESTADO INICIAL ------------------
        self.state = 'GO_TO_BALL'

        # ------------------ MEMÓRIA DOS RETÂNGULOS ------------------
        self.green_rect = None
        self.black_rect = None

        # ------------------ LISTA COM OS CONTORNOS APROX. DO CANNY (PRETO) -------------
        self.canny_approxs = []  

        # ------------------ THREAD DE MONITORAMENTO ------------------
        self.monitor_thread = threading.Thread(target=self.check_if_stopped)
        self.monitor_thread.start()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # ---------- Pré-processamento: Filtro Gaussiano ----------
        cv_image = cv2.GaussianBlur(cv_image, (3, 3), 1)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # ---------- Máscaras ----------
        mask_green  = cv2.inRange(hsv, self.lower_green,  self.upper_green)
        mask_blue   = cv2.inRange(hsv, self.lower_blue,   self.upper_blue)
        mask_orange = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        mask_black  = cv2.inRange(hsv, self.lower_black,  self.upper_black)

        # ---------- Contornos ----------
        green_contours, _  = cv2.findContours(mask_green,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _   = cv2.findContours(mask_blue,   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        black_contours, _  = cv2.findContours(mask_black,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_green  = (len(green_contours)  > 0)
        detected_blue   = (len(blue_contours)   > 0)
        detected_orange = (len(orange_contours) > 0)

        if detected_green and detected_blue and detected_orange:
            # Centros (verde, azul, laranja)
            green_moments  = cv2.moments(mask_green)
            blue_moments   = cv2.moments(mask_blue)
            orange_moments = cv2.moments(mask_orange)

            green_center = (
                int(green_moments['m10']/green_moments['m00']),
                int(green_moments['m01']/green_moments['m00'])
            )
            blue_center = (
                int(blue_moments['m10']/blue_moments['m00']),
                int(blue_moments['m01']/blue_moments['m00'])
            )
            orange_center= (
                int(orange_moments['m10']/orange_moments['m00']),
                int(orange_moments['m01']/orange_moments['m00'])
            )

            # Distância (bola-robô)
            distance = self.calculate_distance_cm(orange_center, blue_center)
            self.last_distance = int(distance)
            self.robot_blue_center = int(blue_center[1])

            # Detecta área verde
            self.green_rect = self.detect_green_area_with_memory(cv_image, green_contours)
            # Detecta área preta
            self.black_rect = self.detect_black_area_with_memory(cv_image, black_contours, orange_center)

            # ================ LÓGICA DE ESTADOS ================
            if self.state== 'GO_TO_BALL':
                target_center= orange_center
                if distance< self.target_distance_threshold* self.pixels_per_meter:
                    self.state= 'GO_TO_GOAL'
            else:
                if self.green_rect:
                    gx, gy, gw, gh= self.green_rect
                    target_center= (gx + gw//2, gy + gh//2)
                else:
                    target_center= green_center

                if distance> self.target_distance_threshold* self.pixels_per_meter:
                    self.state= 'GO_TO_BALL'

            # Giro rápido
            if self.state == 'GO_TO_GOAL':
                if self.green_rect:
                    gx, gy, gw, gh= self.green_rect
                    orientation_center= (gx + gw//2, gy + gh//2)
                else:
                    orientation_center= green_center

                orig_ang= self.angular_speed
                self.angular_speed= 2
                lin_v, ang_v= self.calculate_velocities(
                    target_center, blue_center, orientation_center
                )
                self.angular_speed= orig_ang
            else:
                lin_v, ang_v= self.calculate_velocities(
                    target_center, blue_center, green_center
                )

            # Cálculo das rodas
            lw, rw= self.calculate_wheel_velocities(lin_v, ang_v, distance)
            wheel_vel_msg= Float64MultiArray()
            wheel_vel_msg.data= [lw, rw, self.rotation]
            self.wheel_vel_pub.publish(wheel_vel_msg)

            # Rastreio bola
            self.ball_traj_points.append((orange_center[0], orange_center[1]))

            # Publica markers
            self.publish_markers(blue_center, orange_center, self.green_rect, self.black_rect)

            # Atualiza traj robô
            self.update_trajectory_marker(blue_center)

            # Desenha setas
            cv_image= self.draw_arrows(cv_image, green_center, blue_center, orange_center)

            # Texto
            cv_image= self.add_text(cv_image, f'Angulo {ang_v:.2f} rad/s',(10,20))
            cv_image= self.add_text(cv_image, f'Motor L: {lw:.2f}, R: {rw:.2f}',(10,50))
            cv_image= self.add_text(cv_image, f'Bola: ({orange_center[0]}, {orange_center[1]})',(10,80))
            cv_image= self.add_text(cv_image, f'Robo: ({blue_center[0]}, {blue_center[1]})',(10,110))
            cv_image= self.add_text(cv_image, f'Distancia: {distance:.2f} cm',(10,140))
            cv_image= self.add_text(cv_image, f'Status: {self.state}',(10,170))
            cv_image= self.add_text(cv_image, f'Rotacao: {self.rotation}',(10,200))

        # Imshow final (imagem toda)
        cropped_image = cv_image[0:self.height, 0:self.width]
        cv2.imshow("Color Detection", cropped_image)
        cv2.waitKey(1)

    # --------------------------------------------------------
    #               DETECÇÃO DE ÁREA VERDE (MEMÓRIA)
    # --------------------------------------------------------
    def detect_green_area_with_memory(self, cv_image, green_contours):
        best_rect= None
        max_area= 0
        for contour in green_contours:
            x, y, w, h= cv2.boundingRect(contour)
            area= w*h
            if self.min_area_green< area< self.max_area_green and area> max_area:
                max_area= area
                best_rect= (x,y,w,h)
        if best_rect:
            x, y, w, h= best_rect
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
            self.green_rect= best_rect
        else:
            if self.green_rect:
                gx,gy,gw,gh= self.green_rect
                cv2.rectangle(cv_image,(gx,gy),(gx+gw,gy+gh),(0,255,0),2)
        return self.green_rect

    # --------------------------------------------------------
    #   DETECÇÃO DE ÁREA PRETA (campo) + CONTORNO (CANNY) C/ MEMÓRIA
    #   + ARMAZENANDO TODOS OS CONTORNOS (LINHAS) EM self.canny_approxs
    # --------------------------------------------------------
    def detect_black_area_with_memory(self, cv_image, black_contours, orange_center):
        best_black_rect = None
        max_area_black  = 0

        self.canny_approxs = []

        for contour in black_contours:
            bx, by, bw, bh= cv2.boundingRect(contour)
            area= bw* bh
            if self.min_area_black< area< self.max_area_black and area> max_area_black:
                max_area_black= area
                best_black_rect= (bx, by, bw, bh)

        if best_black_rect:
            bx, by, bw, bh= best_black_rect
            cv2.rectangle(cv_image, (bx,by),(bx+bw,by+bh),(255,0,0),2)

            roi= cv_image[by:by+bh, bx:bx+bw]
            roi_gray= cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            edges= cv2.Canny(roi_gray,100,100, apertureSize=3)

            contours_canny, _= cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best_canny_rect= None
            max_area_canny=0
            for cc in contours_canny:
                perimeter= cv2.arcLength(cc, True)
                approx= cv2.approxPolyDP(cc, 0.005 * perimeter, True)

                self.canny_approxs.append((approx, bx, by))  

                if len(approx)==4:
                    x2,y2,w2,h2= cv2.boundingRect(approx)
                    area2= w2*h2
                    if self.min_area< area2< self.max_area and area2> max_area_canny:
                        max_area_canny= area2
                        best_canny_rect= (x2+ bx, y2+ by, w2, h2)

            if best_canny_rect:
                x2,y2,w2,h2= best_canny_rect
                cv2.rectangle(cv_image,(x2,y2),(x2+w2,y2+h2),(0,255,0),2)
                rect_center= (x2 + w2//2, y2 + h2//2)
                cv_image= cv2.arrowedLine(cv_image, orange_center, rect_center,(0,255,0),2)

            self.black_rect= best_black_rect
        else:
            if self.black_rect:
                bx,by,bw,bh= self.black_rect
                cv2.rectangle(cv_image,(bx,by),(bx+bw,by+bh),(255,0,0),2)

        return self.black_rect

    # --------------------------------------------------------
    #               PUBLICAÇÃO DE MARCADORES
    # --------------------------------------------------------
    def publish_markers(self, robot_center, ball_center, green_rect, black_rect):
        marker_array = MarkerArray()

        # Robô
        robot_marker = Marker()
        robot_marker.header.frame_id = "world"
        robot_marker.header.stamp = rospy.Time.now()
        robot_marker.id = 0
        robot_marker.type = Marker.CUBE
        robot_marker.action = Marker.ADD
        robot_marker.pose.position = Point(
            robot_center[0]/self.pixels_per_meter,
            robot_center[1]/self.pixels_per_meter,
            0
        )
        robot_marker.scale.x = 0.1
        robot_marker.scale.y = 0.1
        robot_marker.scale.z = 0.1
        robot_marker.color = ColorRGBA(0.0, 0.0, 1.0, 1.0)  
        marker_array.markers.append(robot_marker)

        # Bola
        ball_marker = Marker()
        ball_marker.header.frame_id = "world"
        ball_marker.header.stamp = rospy.Time.now()
        ball_marker.id = 1
        ball_marker.type = Marker.SPHERE
        ball_marker.action = Marker.ADD
        ball_marker.pose.position = Point(
            ball_center[0]/self.pixels_per_meter,
            ball_center[1]/self.pixels_per_meter,
            0
        )
        ball_marker.scale.x = 0.08
        ball_marker.scale.y = 0.08
        ball_marker.scale.z = 0.08
        ball_marker.color = ColorRGBA(1.0, 0.5, 0.0, 1.0)  
        marker_array.markers.append(ball_marker)

        # Área verde
        if self.green_rect:
            gx, gy, gw, gh= green_rect
            goleiro_marker= Marker()
            goleiro_marker.header.frame_id= "world"
            goleiro_marker.header.stamp= rospy.Time.now()
            goleiro_marker.id= 2
            goleiro_marker.type= Marker.CUBE
            goleiro_marker.action= Marker.ADD
            goleiro_marker.pose.position = Point(
                (gx + gw/2)/ self.pixels_per_meter,
                (gy + gh/2)/ self.pixels_per_meter,
                0
            )
            goleiro_marker.scale.x= gw/ self.pixels_per_meter
            goleiro_marker.scale.y= gh/ self.pixels_per_meter
            goleiro_marker.scale.z= 0.02
            goleiro_marker.color= ColorRGBA(0.0,1.0,0.0,0.5)
            marker_array.markers.append(goleiro_marker)

        # Área preta
        if black_rect:
            bx, by, bw, bh= black_rect
            black_marker= Marker()
            black_marker.header.frame_id = "world"
            black_marker.header.stamp = rospy.Time.now()
            black_marker.id = 3
            black_marker.type= Marker.LINE_STRIP
            black_marker.action = Marker.ADD
            black_marker.scale.x = 0.02
            black_marker.color= ColorRGBA(0.0,0.0,1.0,1.0)

            black_marker.points= [
                Point(bx/self.pixels_per_meter,      by/self.pixels_per_meter,      0),
                Point((bx+bw)/self.pixels_per_meter, by/self.pixels_per_meter,      0),
                Point((bx+bw)/self.pixels_per_meter, (by+bh)/self.pixels_per_meter, 0),
                Point(bx/self.pixels_per_meter,      (by+bh)/self.pixels_per_meter, 0),
                Point(bx/self.pixels_per_meter,      by/self.pixels_per_meter,      0)
            ]
            marker_array.markers.append(black_marker)

        # Trajetória da bola
        if len(self.ball_traj_points) > 1:
            ball_traj_marker = Marker()
            ball_traj_marker.header.frame_id = "world"
            ball_traj_marker.header.stamp = rospy.Time.now()
            ball_traj_marker.id = 5
            ball_traj_marker.type = Marker.LINE_STRIP
            ball_traj_marker.action = Marker.ADD
            ball_traj_marker.scale.x = 0.02
            ball_traj_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  

            points_list = []
            for (bx, by) in self.ball_traj_points:
                px = bx / self.pixels_per_meter
                py = by / self.pixels_per_meter
                points_list.append(Point(px, py, 0))
            ball_traj_marker.points = points_list
            marker_array.markers.append(ball_traj_marker)

        self.marker_pub.publish(marker_array)

    def update_trajectory_marker(self, robot_center):
        self.trajectory_points.append(Point(
            robot_center[0]/ self.pixels_per_meter,
            robot_center[1]/ self.pixels_per_meter,
            0
        ))
        trajectory_marker= Marker()
        trajectory_marker.header.frame_id= "world"
        trajectory_marker.header.stamp= rospy.Time.now()
        trajectory_marker.id= 4
        trajectory_marker.type= Marker.LINE_STRIP
        trajectory_marker.action= Marker.ADD
        trajectory_marker.scale.x= 0.02
        trajectory_marker.color= ColorRGBA(1.0,1.0,0.0,1.0)
        trajectory_marker.points= self.trajectory_points

        self.trajectory_pub.publish(trajectory_marker)

    def draw_arrows(self, image, green_center, blue_center, orange_center):
        image= cv2.arrowedLine(image, blue_center, green_center,(0,0,255),2)
        image= cv2.arrowedLine(image, blue_center, orange_center,(0,255,0),2)
        return image

    def add_text(self, image, text, position):
        return cv2.putText(
            image, text, position,
            self.font, self.font_scale,
            self.font_color, self.font_thickness,
            cv2.LINE_AA
        )

    def calculate_velocities(self, target_center, robot_center, orientation_center):
        desired_orientation= atan2(
            target_center[1]- robot_center[1],
            target_center[0]- robot_center[0]
        )
        robot_orientation= atan2(
            orientation_center[1]- robot_center[1],
            orientation_center[0]- robot_center[0]
        )
        angular_diff= atan2(
            sin(desired_orientation- robot_orientation),
            cos(desired_orientation- robot_orientation)
        )

        linear_velocity= self.linear_speed
        angular_velocity= angular_diff* self.angular_speed
        return linear_velocity, angular_velocity

    def calculate_distance_cm(self, point1, point2):
        dist_pixels= sqrt((point1[0]- point2[0])**2 + (point1[1]- point2[1])**2)
        dist_m= dist_pixels/ self.pixels_per_meter
        return dist_m* 45  # Ajuste se seu fator for 100

    def calculate_wheel_velocities(self, linear_velocity, angular_velocity, distance):
        max_velocity= 43
        angular_velocity*= 10

        kp= 0.7
        kd= 0.1
        ki= 0.05

        error= angular_velocity
        derivative= (error- self.last_error)* kd
        integral= abs((error+ self.last_error)* ki)
        velocity= error* kp + integral + derivative

        if velocity> max_velocity:
            velocity= max_velocity
        if velocity< -max_velocity:
            velocity= -max_velocity

        left_wheel_vel= ((max_velocity- velocity)+4)
        right_wheel_vel= max_velocity+ velocity

        self.last_error= error
        return left_wheel_vel, right_wheel_vel

    def check_if_stopped(self):
        past_distance= None
        past_robot_position= None
        while not rospy.is_shutdown():
            current_distance= self.last_distance
            robo_position   = self.robot_blue_center

            if current_distance== past_distance or robo_position== past_robot_position:
                self.cont+=1
                if self.cont>=5:
                    self.rotation=0
            else:
                self.rotation=1
                self.cont=0

            past_distance= current_distance
            past_robot_position= robo_position
            time.sleep(0.2)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ColorDetectionNode()
    node.run()
