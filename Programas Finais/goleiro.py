import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Float64MultiArray
from math import atan2, sin, cos, sqrt
import threading
import time
from collections import deque

class GoalieRobotNode:
    def __init__(self):
        rospy.init_node('goalie_robot_node', anonymous=True)

        # Subscrições/Publicações
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.marker_pub = rospy.Publisher('/robot_marker', MarkerArray, queue_size=1)
        self.wheel_vel_pub = rospy.Publisher('/velocidade_motores', Float64MultiArray, queue_size=1)
        self.vel_pub = rospy.Publisher('/velocidade', Twist, queue_size=1)

        self.lower_green  = np.array([61, 86, 135])
        self.upper_green  = np.array([95, 255, 255])
        self.lower_blue   = np.array([0, 165, 135])
        self.upper_blue   = np.array([140, 255, 255])
        self.lower_orange = np.array([0, 38, 244])
        self.upper_orange = np.array([95, 255, 255])

        # Configurações de texto
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.6
        self.font_color = (255, 0, 200)
        self.font_thickness = 1

        # Parâmetros de controle
        self.wheel_distance = 0.075
        self.target_distance_threshold = 0.006
        self.linear_speed = 0.1
        self.angular_speed = 0.5

        # Variáveis de controle PID
        self.last_error = 0

        # Conversão pixels -> metros
        self.pixels_per_meter = 250

        # Resolução da câmera
        self.width = 640
        self.height = 480

        # Área mínima e máxima do gol
        self.min_area = 18450
        self.max_area = 19500

        # Estado inicial e memória do gol
        self.state = 'RETURN_TO_GOAL'
        self.goal_rect = None

        # ---------- Parada do goleiro ----------
        self.last_distance = None
        self.rotation = 1
        self.cont = 0

        # ---------- Parada do jogador ----------
        self.player_last_distance = None
        self.player_rotation = 1
        self.player_cont = 0

        # ---------- Threads de monitoramento ----------
        self.monitor_thread = threading.Thread(target=self.check_if_stopped_goalie)
        self.monitor_thread.start()
        self.monitor_thread_player = threading.Thread(target=self.check_if_stopped_player)
        self.monitor_thread_player.start()

        # ---------- Buffer para prever trajetória da bola ----------
        # Armazena (pos_x, pos_y, time_stamp) dos últimos frames
        self.ball_positions = deque(maxlen=10)  # Exemplo com até 10

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Pré-processamento
        cv_image = cv2.GaussianBlur(cv_image, (3, 3), 1)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Máscaras
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask_orange = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected_green = len(green_contours) > 0
        detected_blue = len(blue_contours) > 0
        detected_orange = len(orange_contours) > 0

        max_area = 0

        if detected_green and detected_blue and detected_orange:
            # Centros
            green_moments = cv2.moments(mask_green)
            blue_moments = cv2.moments(mask_blue)
            orange_moments = cv2.moments(mask_orange)

            green_center = (int(green_moments['m10'] / green_moments['m00']),
                            int(green_moments['m01'] / green_moments['m00']))
            blue_center = (int(blue_moments['m10'] / blue_moments['m00']),
                           int(blue_moments['m01'] / blue_moments['m00']))
            orange_center = (int(orange_moments['m10'] / orange_moments['m00']),
                             int(orange_moments['m01'] / orange_moments['m00']))

            distance = self.calculate_distance_cm(orange_center, blue_center)
            self.last_distance = distance
            self.player_last_distance = distance

            # Detecta gol + memória
            self.goal_rect, max_area = self.detect_goal_area_with_area(cv_image)

            inside_goal = self.is_inside_goal(green_center)
            if not inside_goal:
                self.state = 'RETURN_TO_GOAL'
            else:
                self.state = 'STAY_IN_GOAL'
                if distance < 80:
                    self.state = 'GO_TO_BALL'

            # ---------- ARMAZENA POSIÇÃO ATUAL DA BOLA + TEMPO ----------
            current_time = rospy.get_time()
            self.ball_positions.append((orange_center[0], orange_center[1], current_time))

            if self.state == 'RETURN_TO_GOAL' and self.goal_rect:
                x, y, w, h = self.goal_rect
                goal_center_x = x + w//2
                goal_center_y = y + h//2
                target_center = (goal_center_x, goal_center_y)
            elif self.state == 'GO_TO_BALL':
                # ---------- AQUI VEM A PREVISÃO DE TRAJETÓRIA ----------
                predicted_ball = self.predict_ball_trajectory()
                target_center = predicted_ball if predicted_ball else orange_center
            else:
                target_center = green_center
                self.publish_zero_velocity(cv_image, "Ficando parado na area verde")

                # Desenha setas
                cv_image = self.draw_arrows(cv_image, green_center, blue_center, orange_center)
                # Infos
                angular_velocity = 0.0
                left_wheel_vel, right_wheel_vel = 0.0, 0.0
                cv_image = self.display_info(cv_image, angular_velocity,
                                             left_wheel_vel, right_wheel_vel,
                                             orange_center, blue_center,
                                             distance, max_area)
                cropped_image = cv_image[0:self.height, 0:self.width]
                cv2.imshow("Color Detection", cropped_image)
                cv2.waitKey(1)
                return

            # Calcula velocidades
            linear_velocity, angular_velocity = self.calculate_velocities(target_center,
                                                                          blue_center,
                                                                          green_center)
            left_wheel_vel, right_wheel_vel = self.calculate_wheel_velocities(linear_velocity,
                                                                              angular_velocity)
            # Publica
            self.publish_velocities(linear_velocity, angular_velocity,
                                    left_wheel_vel, right_wheel_vel)

            # Desenha setas
            cv_image = self.draw_arrows(cv_image, green_center, blue_center, orange_center)

            # Exibe infos
            cv_image = self.display_info(cv_image, angular_velocity,
                                         left_wheel_vel, right_wheel_vel,
                                         orange_center, blue_center,
                                         distance, max_area)

        cropped_image = cv_image[0:self.height, 0:self.width]
        cv2.imshow("Color Detection", cropped_image)
        cv2.waitKey(1)

    def predict_ball_trajectory(self, future_time=0.5):
        """
        Prevê posição da bola no futuro_time (segundos) no modelo
        retilíneo uniforme simples.
        """
        if len(self.ball_positions) < 2:
            return None  # Não há dados suficientes

        # Pega posições mais recentes
        x1, y1, t1 = self.ball_positions[-1]
        x0, y0, t0 = self.ball_positions[-2]

        dt = t1 - t0
        if dt <= 0:
            return None

        # Velocidade em pixels/s
        vx = (x1 - x0) / dt
        vy = (y1 - y0) / dt

        # Estima posição futura
        x_future = x1 + vx * future_time
        y_future = y1 + vy * future_time

        # Garante que não ultrapasse limites da imagem (opcional)
        x_future = max(0, min(self.width, x_future))
        y_future = max(0, min(self.height, y_future))

        return (int(x_future), int(y_future))

    def detect_goal_area_with_area(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_area = 0
        best_canny_rect = None

        for contour in contours:
            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.01 * perimeter, True)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                area = w * h
                if self.min_area < area < self.max_area and area > max_area:
                    max_area = area
                    best_canny_rect = (x, y, w, h)

        if best_canny_rect:
            x, y, w, h = best_canny_rect
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            return best_canny_rect, max_area
        else:
            if self.goal_rect is not None:
                x, y, w, h = self.goal_rect
                cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                return self.goal_rect, (self.goal_rect[2]*self.goal_rect[3])
            return None, 0.0

    def is_inside_goal(self, green_center):
        if not self.goal_rect:
            return False
        x, y, w, h = self.goal_rect
        return (x <= green_center[0] <= x+w and
                y <= green_center[1] <= y+h)

    def display_info(self, image, angular_velocity,
                     left_wheel_vel, right_wheel_vel,
                     orange_center, blue_center,
                     distance, max_area):
        # Exibe as infos solicitadas
        image = self.add_text(image, f'Angulo {angular_velocity:.2f} rad/s', (10, 20))
        image = self.add_text(image, f'Motor Left: {left_wheel_vel:.2f}, Motor Right: {right_wheel_vel:.2f}', (10, 40))
        image = self.add_text(image, f'Bola: ({orange_center[0]}, {orange_center[1]})', (10, 60))
        image = self.add_text(image, f'Robo: ({blue_center[0]}, {blue_center[1]})', (10, 80))
        image = self.add_text(image, f'Distancia: {distance:.2f} cm', (10, 120))
        image = self.add_text(image, f'Area Gol: {max_area:.2f} px', (10, 140))
        image = self.add_text(image, f'Status: {self.cont}', (10, 160))
        image = self.add_text(image, f'Rotacao: {self.rotation}', (10, 180))
        return image

    def publish_zero_velocity(self, cv_image, msg="Parado"):
        wheel_vel_msg = Float64MultiArray()
        wheel_vel_msg.data = [0, 0, self.rotation]
        self.wheel_vel_pub.publish(wheel_vel_msg)

        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.vel_pub.publish(vel_msg)

        cv2.putText(cv_image, msg, (10, 100), self.font,
                    self.font_scale, self.font_color,
                    self.font_thickness, cv2.LINE_AA)

    def add_text(self, image, text, position):
        return cv2.putText(image, text, position, self.font,
                           self.font_scale, self.font_color,
                           self.font_thickness, cv2.LINE_AA)

    def calculate_distance_cm(self, point1, point2):
        distance_pixels = sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
        distance_meters = distance_pixels / self.pixels_per_meter
        return distance_meters * 100

    def calculate_velocities(self, target_center, robot_center, orientation_center):
        desired_orientation = atan2(target_center[1] - robot_center[1],
                                    target_center[0] - robot_center[0])
        robot_orientation = atan2(orientation_center[1] - robot_center[1],
                                  orientation_center[0] - robot_center[0])
        angular_diff = atan2(sin(desired_orientation - robot_orientation),
                             cos(desired_orientation - robot_orientation))

        if abs(angular_diff) < 0.01:
            angular_diff = 0.01 if angular_diff > 0 else -0.01

        linear_velocity = (self.linear_speed if abs(angular_diff) < 1.0
                           else self.linear_speed * 0.5)
        angular_velocity = angular_diff * self.angular_speed
        return linear_velocity, angular_velocity

    def calculate_wheel_velocities(self, linear_velocity, angular_velocity):
        max_velocity = 40
        angular_velocity *= 10

        kp = 0.5
        kd = 0.025
        ki = 0.1

        error = angular_velocity
        derivative = (error - self.last_error) * kd
        integral = (error + self.last_error) * ki
        velocity = error * kp + integral + derivative

        velocity = max(min(velocity, max_velocity), -max_velocity)

        left_wheel_vel= ((max_velocity- velocity)+3)
        right_wheel_vel = max_velocity + velocity

        self.last_error = error
        return left_wheel_vel, right_wheel_vel

    def publish_velocities(self, linear_velocity, angular_velocity,
                           left_wheel_vel, right_wheel_vel):
        wheel_vel_msg = Float64MultiArray()
        wheel_vel_msg.data = [left_wheel_vel, right_wheel_vel, self.rotation]
        self.wheel_vel_pub.publish(wheel_vel_msg)

        vel_msg = Twist()
        vel_msg.linear.x = linear_velocity
        vel_msg.angular.z = angular_velocity
        self.vel_pub.publish(vel_msg)

    def draw_arrows(self, image, green_center, blue_center, orange_center):
        image = cv2.arrowedLine(image, blue_center, green_center, (0, 0, 255), 2)
        image = cv2.arrowedLine(image, blue_center, orange_center, (0, 255, 0), 2)
        return image

    def check_if_stopped_goalie(self):
        """ Ajuste no verificador de parada: compara variação < limiar """
        past_distance = None
        threshold = 0.1
        while not rospy.is_shutdown():
            if past_distance is not None and self.last_distance is not None:
                diff = abs(self.last_distance - past_distance)
                if diff < threshold:
                    self.cont += 1
                    if self.cont >= 4:
                        self.rotation = 0
                else:
                    self.rotation = 1
                    self.cont = 0
            past_distance = self.last_distance
            time.sleep(0.2)

    def check_if_stopped_player(self):
        """ Ajuste no verificador de parada p/ jogador """
        prev_distance = None
        threshold = 2
        while not rospy.is_shutdown():
            if prev_distance is not None and self.player_last_distance is not None:
                diff = abs(self.player_last_distance - prev_distance)
                if diff < threshold:
                    self.player_cont += 1
                    if self.player_cont >= 5:
                        self.player_rotation = 0
                else:
                    self.player_rotation = 1
                    self.player_cont = 0
            prev_distance = self.player_last_distance
            time.sleep(0.2)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = GoalieRobotNode()
    node.run()
