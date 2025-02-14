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

class GoalieRobotNode:
    def __init__(self):
        rospy.init_node('goalie_robot_node', anonymous=True)

        # ---------- Subscrições/Publicações ----------
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.marker_pub = rospy.Publisher('/robot_marker', MarkerArray, queue_size=1)
        self.wheel_vel_pub = rospy.Publisher('/velocidade_motores', Float64MultiArray, queue_size=1)
        self.vel_pub = rospy.Publisher('/velocidade', Twist, queue_size=1)

        # ---------- Faixas de cores ----------
        self.lower_green  = np.array([61, 86, 135])
        self.upper_green  = np.array([95, 255, 255])
        self.lower_blue   = np.array([0, 165, 135])
        self.upper_blue   = np.array([140, 255, 255])
        self.lower_orange = np.array([0, 38, 244])
        self.upper_orange = np.array([95, 255, 255])

        # Campo preto
        self.lower_black = np.array([42, 23, 0])
        self.upper_black = np.array([179, 255, 255])

        # ---------- Configurações de texto ----------
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.6
        self.font_color = (255, 0, 200)
        self.font_thickness = 1

        # ---------- Parâmetros de controle ----------
        self.wheel_distance = 0.075
        self.target_distance_threshold = 0.006
        self.linear_speed  = 0.1
        self.angular_speed = 0.5

        # ---------- Variáveis de controle PID ----------
        self.last_error = 0

        # ---------- Conversão pixels->metros ----------
        self.pixels_per_meter = 250

        # ---------- Resolução da câmera ----------
        self.width  = 640
        self.height = 480

        # ---------- Área mínima e máxima do gol ----------
        self.min_area = 18450
        self.max_area = 19500

        # ---------- Estado e memória do gol / ret. preto ----------
        self.state      = 'RETURN_TO_GOAL'
        self.goal_rect  = None
        self.black_rect = None  # retângulo do campo

        # ---------- Parada do goleiro/jogador ----------
        self.last_distance = None
        self.rotation = 1
        self.cont = 0

        self.player_last_distance = None
        self.player_rotation = 1
        self.player_cont = 0

        # ---------- Threads de monitoramento ----------
        self.monitor_thread = threading.Thread(target=self.check_if_stopped_goalie)
        self.monitor_thread.start()
        self.monitor_thread_player = threading.Thread(target=self.check_if_stopped_player)
        self.monitor_thread_player.start()

        # ---------- Buffers p/ trajetória ----------
        # Agora SEM maxlen, armazenando TODA a trajetória
        self.ball_positions  = deque()  # armazena (x, y, time)
        self.robot_positions = deque()  # armazena (x, y)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Pré-processamento
        cv_image = cv2.GaussianBlur(cv_image, (3, 3), 1)
        hsv      = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Máscaras
        mask_green  = cv2.inRange(hsv, self.lower_green,  self.upper_green)
        mask_blue   = cv2.inRange(hsv, self.lower_blue,   self.upper_blue)
        mask_orange = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        mask_black  = cv2.inRange(hsv, self.lower_black,  self.upper_black)

        green_contours, _  = cv2.findContours(mask_green,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _   = cv2.findContours(mask_blue,   cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        black_contours, _  = cv2.findContours(mask_black,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_green  = len(green_contours)>0
        detected_blue   = len(blue_contours)>0
        detected_orange = len(orange_contours)>0

        max_area =0
        blue_center= (0,0)
        orange_center= (0,0)
        green_center= (0,0)

        if detected_green and detected_blue and detected_orange:
            # Centros
            green_moments  = cv2.moments(mask_green)
            blue_moments   = cv2.moments(mask_blue)
            orange_moments = cv2.moments(mask_orange)

            green_center= (
                int(green_moments['m10']/ green_moments['m00']),
                int(green_moments['m01']/ green_moments['m00'])
            )
            blue_center= (
                int(blue_moments['m10']/ blue_moments['m00']),
                int(blue_moments['m01']/ blue_moments['m00'])
            )
            orange_center= (
                int(orange_moments['m10']/ orange_moments['m00']),
                int(orange_moments['m01']/ orange_moments['m00'])
            )

            distance= self.calculate_distance_cm(orange_center, blue_center)
            self.last_distance        = distance
            self.player_last_distance = distance

            self.goal_rect, max_area  = self.detect_goal_area_with_area(cv_image)
            self.black_rect           = self.detect_black_area(cv_image, black_contours)

            inside_goal = self.is_inside_goal(green_center)
            if not inside_goal:
                self.state = 'RETURN_TO_GOAL'
            else:
                self.state = 'STAY_IN_GOAL'
                if distance<80:
                    self.state='GO_TO_BALL'

            current_time= rospy.get_time()
            # Armazena bola e robô
            self.ball_positions.append( (orange_center[0], orange_center[1], current_time) )
            self.robot_positions.append(blue_center)

            if self.state=='RETURN_TO_GOAL' and self.goal_rect:
                x,y,w,h= self.goal_rect
                gx= x+ w//2
                gy= y+ h//2
                target_center= (gx, gy)
            elif self.state=='GO_TO_BALL':
                pred= self.predict_ball_trajectory()
                target_center= pred if pred else orange_center
            else:
                # STAY_IN_GOAL
                target_center= green_center
                self.publish_zero_velocity(cv_image, "Ficando parado na area verde")

                cv_image= self.draw_arrows(cv_image, green_center, blue_center, orange_center)
                angular_velocity=0.0
                lw,rw= 0.0,0.0
                cv_image= self.display_info(cv_image, angular_velocity,lw,rw,
                                            orange_center,blue_center,
                                            distance, max_area)
                # Imshow
                cropped= cv_image[0:self.height, 0:self.width]
                cv2.imshow("Color Detection", cropped)
                cv2.waitKey(1)
                # Publica markers
                self.publish_markers(blue_center, orange_center, self.goal_rect, self.black_rect)
                return

            lin_v, ang_v= self.calculate_velocities(target_center, blue_center, green_center)
            lw, rw= self.calculate_wheel_velocities(lin_v, ang_v)
            self.publish_velocities(lin_v, ang_v, lw,rw)

            cv_image= self.draw_arrows(cv_image, green_center, blue_center, orange_center)
            cv_image= self.display_info(cv_image, ang_v, lw,rw,
                                        orange_center,blue_center,
                                        distance, max_area)

        # Publica markers
        self.publish_markers(blue_center, orange_center, self.goal_rect, self.black_rect)

        cropped_image= cv_image[0:self.height, 0:self.width]
        cv2.imshow("Color Detection", cropped_image)
        cv2.waitKey(1)

    # =========== Publicar markers: Robô, Bola, Gol, Campo, Trajetórias ===========
    def publish_markers(self, robot_center, ball_center, goal_rect, black_rect):
        marker_array= MarkerArray()

        # Robô
        robot_marker= Marker()
        robot_marker.header.frame_id="world"
        robot_marker.header.stamp= rospy.Time.now()
        robot_marker.id= 0
        robot_marker.type= Marker.CUBE
        robot_marker.action= Marker.ADD
        robot_marker.pose.position= Point(
            robot_center[0]/ self.pixels_per_meter,
            robot_center[1]/ self.pixels_per_meter, 0
        )
        robot_marker.scale.x= 0.1
        robot_marker.scale.y= 0.1
        robot_marker.scale.z= 0.1
        robot_marker.color= ColorRGBA(0.0,0.0,1.0,1.0)  # azul
        marker_array.markers.append(robot_marker)

        # Bola
        ball_marker= Marker()
        ball_marker.header.frame_id= "world"
        ball_marker.header.stamp= rospy.Time.now()
        ball_marker.id= 1
        ball_marker.type= Marker.SPHERE
        ball_marker.action= Marker.ADD
        ball_marker.pose.position= Point(
            ball_center[0]/ self.pixels_per_meter,
            ball_center[1]/ self.pixels_per_meter, 0
        )
        ball_marker.scale.x=0.08
        ball_marker.scale.y=0.08
        ball_marker.scale.z=0.08
        ball_marker.color= ColorRGBA(1.0,0.5,0.0,1.0)  # laranja
        marker_array.markers.append(ball_marker)

        # Gol
        if goal_rect:
            x,y,w,h= goal_rect
            goal_marker= Marker()
            goal_marker.header.frame_id= "world"
            goal_marker.header.stamp= rospy.Time.now()
            goal_marker.id= 2
            goal_marker.type= Marker.CUBE
            goal_marker.action= Marker.ADD
            goal_marker.pose.position= Point(
                (x + w/2)/ self.pixels_per_meter,
                (y + h/2)/ self.pixels_per_meter, 0
            )
            goal_marker.scale.x= w/self.pixels_per_meter
            goal_marker.scale.y= h/self.pixels_per_meter
            goal_marker.scale.z= 0.02
            goal_marker.color= ColorRGBA(0.0,1.0,0.0,0.5)  # verde semitransp
            marker_array.markers.append(goal_marker)

        # Campo preto
        if black_rect:
            bx, by, bw, bh= black_rect
            black_marker= Marker()
            black_marker.header.frame_id= "world"
            black_marker.header.stamp= rospy.Time.now()
            black_marker.id= 3
            black_marker.type= Marker.LINE_STRIP
            black_marker.action= Marker.ADD
            black_marker.scale.x=0.02
            black_marker.color= ColorRGBA(0.0,0.0,1.0,1.0) # azul

            black_marker.points= [
                Point(bx/self.pixels_per_meter,      by/self.pixels_per_meter,      0),
                Point((bx+bw)/self.pixels_per_meter, by/self.pixels_per_meter,      0),
                Point((bx+bw)/self.pixels_per_meter, (by+bh)/self.pixels_per_meter, 0),
                Point(bx/self.pixels_per_meter,      (by+bh)/self.pixels_per_meter, 0),
                Point(bx/self.pixels_per_meter,      by/self.pixels_per_meter,      0)
            ]
            marker_array.markers.append(black_marker)

        # Trajetória da bola (vermelha)
        if len(self.ball_positions)>1:
            ball_path_marker= Marker()
            ball_path_marker.header.frame_id= "world"
            ball_path_marker.header.stamp= rospy.Time.now()
            ball_path_marker.id= 4
            ball_path_marker.type= Marker.LINE_STRIP
            ball_path_marker.action= Marker.ADD
            ball_path_marker.scale.x= 0.02
            ball_path_marker.color= ColorRGBA(1.0,0.0,0.0,1.0) # vermelho

            pts_ball= []
            for (bx,by,_t) in self.ball_positions:
                px= bx/self.pixels_per_meter
                py= by/self.pixels_per_meter
                pts_ball.append(Point(px,py,0))
            ball_path_marker.points= pts_ball
            marker_array.markers.append(ball_path_marker)

        # Trajetória do robô (amarelo)
        if len(self.robot_positions)>1:
            robot_path_marker= Marker()
            robot_path_marker.header.frame_id= "world"
            robot_path_marker.header.stamp= rospy.Time.now()
            robot_path_marker.id= 5
            robot_path_marker.type= Marker.LINE_STRIP
            robot_path_marker.action= Marker.ADD
            robot_path_marker.scale.x= 0.02
            robot_path_marker.color= ColorRGBA(1.0,1.0,0.0,1.0) # amarelo

            pts_robot= []
            for (rx,ry) in self.robot_positions:
                px= rx/self.pixels_per_meter
                py= ry/self.pixels_per_meter
                pts_robot.append(Point(px,py,0))
            robot_path_marker.points= pts_robot
            marker_array.markers.append(robot_path_marker)

        # Publica
        self.marker_pub.publish(marker_array)

    # ============ Restante das funções =============

    def predict_ball_trajectory(self, future_time=0.5):
        if len(self.ball_positions)<2:
            return None
        x1,y1,t1= self.ball_positions[-1]
        x0,y0,t0= self.ball_positions[-2]
        dt= t1- t0
        if dt<=0:
            return None
        vx= (x1- x0)/dt
        vy= (y1- y0)/dt
        x_future= x1+ vx*future_time
        y_future= y1+ vy*future_time
        x_future= max(0, min(self.width, x_future))
        y_future= max(0, min(self.height,y_future))
        return (int(x_future), int(y_future))

    def detect_goal_area_with_area(self, cv_image):
        gray= cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges= cv2.Canny(gray,100,200)
        contours,_= cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        max_area=0
        best_rect=None
        for contour in contours:
            perimeter= cv2.arcLength(contour, True)
            approx= cv2.approxPolyDP(contour, 0.01* perimeter, True)
            if len(approx)==4:
                x,y,w,h= cv2.boundingRect(approx)
                area= w*h
                if self.min_area< area< self.max_area and area> max_area:
                    max_area= area
                    best_rect= (x,y,w,h)
        if best_rect:
            x,y,w,h= best_rect
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
            return best_rect,max_area
        else:
            if self.goal_rect:
                x,y,w,h= self.goal_rect
                cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)
                return self.goal_rect, (self.goal_rect[2]* self.goal_rect[3])
            return None,0.0

    def detect_black_area(self, cv_image, black_contours):
        best_black_rect= None
        max_area=0
        for cnt in black_contours:
            bx,by,bw,bh= cv2.boundingRect(cnt)
            area= bw*bh
            if area> max_area:
                max_area= area
                best_black_rect= (bx,by,bw,bh)
        if best_black_rect:
            x,y,w,h= best_black_rect
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(255,0,0),2)
            return best_black_rect
        else:
            return self.black_rect

    def is_inside_goal(self, green_center):
        if not self.goal_rect:
            return False
        x,y,w,h= self.goal_rect
        return (x<= green_center[0]<= (x+w) and
                y<= green_center[1]<= (y+h))

    def publish_zero_velocity(self, cv_image, msg="Parado"):
        wheel_vel_msg= Float64MultiArray()
        wheel_vel_msg.data= [0,0,self.rotation]
        self.wheel_vel_pub.publish(wheel_vel_msg)

        vel_msg= Twist()
        vel_msg.linear.x=0
        vel_msg.angular.z=0
        self.vel_pub.publish(vel_msg)

        cv2.putText(cv_image,msg,(10,100), self.font,
                    self.font_scale,self.font_color,
                    self.font_thickness, cv2.LINE_AA)

    def calculate_distance_cm(self, p1, p2):
        dist_pixels= sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
        dist_m= dist_pixels/ self.pixels_per_meter
        return dist_m*100

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
        if abs(angular_diff)<0.01:
            angular_diff= 0.01 if angular_diff>0 else -0.01

        linear_velocity= (self.linear_speed if abs(angular_diff)<1.0
                          else self.linear_speed* 0.5)
        angular_velocity= angular_diff* self.angular_speed
        return linear_velocity, angular_velocity

    def calculate_wheel_velocities(self, linear_velocity, angular_velocity):
        max_velocity= 40
        angular_velocity*=10

        kp= 0.5
        kd= 0.025
        ki= 0.1
        
        error= angular_velocity

        derivative= (error- self.last_error)* kd
        integral= (error+ self.last_error)* ki
        velocity= error* kp + integral + derivative

        velocity= max(min(velocity, max_velocity), -max_velocity)

        left_wheel_vel= ((max_velocity- velocity)+3)
        right_wheel_vel= max_velocity+ velocity

        self.last_error= error
        return left_wheel_vel, right_wheel_vel

    def publish_velocities(self, linear_velocity, angular_velocity,
                           left_wheel_vel,right_wheel_vel):
        wheel_vel_msg= Float64MultiArray()
        wheel_vel_msg.data= [left_wheel_vel, right_wheel_vel, self.rotation]
        self.wheel_vel_pub.publish(wheel_vel_msg)

        vel_msg= Twist()
        vel_msg.linear.x= linear_velocity
        vel_msg.angular.z= angular_velocity
        self.vel_pub.publish(vel_msg)

    def draw_arrows(self, image, green_center, blue_center, orange_center):
        image= cv2.arrowedLine(image, blue_center, green_center, (0,0,255),2)
        image= cv2.arrowedLine(image, blue_center, orange_center,(0,255,0),2)
        return image

    def display_info(self, image, angular_velocity,
                     left_wheel_vel,right_wheel_vel,
                     orange_center,blue_center,
                     distance, max_area):
        cv2.putText(image,f'Angulo {angular_velocity:.2f} rad/s',(10,20),
                    self.font,self.font_scale,
                    self.font_color,self.font_thickness,cv2.LINE_AA)
        cv2.putText(image,f'Motor L: {left_wheel_vel:.2f}, R: {right_wheel_vel:.2f}',
                    (10,40),self.font,self.font_scale,
                    self.font_color,self.font_thickness,cv2.LINE_AA)
        cv2.putText(image,f'Bola: ({orange_center[0]}, {orange_center[1]})',
                    (10,60),self.font,self.font_scale,
                    self.font_color,self.font_thickness,cv2.LINE_AA)
        cv2.putText(image,f'Robo: ({blue_center[0]}, {blue_center[1]})',(10,80),
                    self.font,self.font_scale,
                    self.font_color,self.font_thickness,cv2.LINE_AA)
        cv2.putText(image,f'Distancia: {distance:.2f} cm',(10,100),
                    self.font,self.font_scale,self.font_color,
                    self.font_thickness,cv2.LINE_AA)
        cv2.putText(image,f'AreaGol: {max_area:.2f} px',(10,140),
                    self.font,self.font_scale,self.font_color,
                    self.font_thickness,cv2.LINE_AA)
        cv2.putText(image,f'Status: {self.cont}',(10,160),
                    self.font,self.font_scale,self.font_color,
                    self.font_thickness,cv2.LINE_AA)
        cv2.putText(image,f'Rotacao: {self.rotation}',(10,180),
                    self.font,self.font_scale,self.font_color,
                    self.font_thickness,cv2.LINE_AA)
        return image

    def check_if_stopped_goalie(self):
        past_distance= None
        threshold=0.1
        while not rospy.is_shutdown():
            if past_distance is not None and self.last_distance is not None:
                diff= abs(self.last_distance- past_distance)
                if diff< threshold:
                    self.cont+=1
                    if self.cont>=4:
                        self.rotation=0
                else:
                    self.rotation=1
                    self.cont=0
            past_distance= self.last_distance
            time.sleep(0.2)

    def check_if_stopped_player(self):
        prev_distance= None
        threshold=1
        while not rospy.is_shutdown():
            if prev_distance is not None and self.player_last_distance is not None:
                diff= abs(self.player_last_distance- prev_distance)
                if diff< threshold:
                    self.player_cont+=1
                    if self.player_cont>=5:
                        self.player_rotation=0
                else:
                    self.player_rotation=1
                    self.player_cont=0
            prev_distance= self.player_last_distance
            time.sleep(0.2)

    def run(self):
        rospy.spin()

if __name__=='__main__':
    node=GoalieRobotNode()
    node.run()
