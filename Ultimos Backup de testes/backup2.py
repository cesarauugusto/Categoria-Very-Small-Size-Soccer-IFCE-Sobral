import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point, Pose, Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Float64MultiArray
from math import atan2, sin, cos, sqrt

class ColorDetectionNode:
    def __init__(self):
        rospy.init_node('color_detection_node', anonymous=True)
        
        self.image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.image_callback)
        self.bridge = CvBridge()
        self.marker_pub = rospy.Publisher('/robot_marker', MarkerArray, queue_size=1)
        self.vel_pub = rospy.Publisher('/velocidade', Twist, queue_size=1)
        self.wheel_vel_pub = rospy.Publisher('/velocidade_motores', Float64MultiArray, queue_size=1)

        # Define as faixas de cores
        self.lower_green = np.array([76, 71, 184])
        self.upper_green = np.array([95, 229, 255])

        self.lower_blue = np.array([84, 135, 184])
        left_wheel_vel = 0
        self.upper_blue = np.array([121, 255, 255])

        self.lower_orange = np.array([3, 23, 255])
        self.upper_orange = np.array([55, 255, 255])

        # Faixa de cor para linhas brancas do campo
        self.lower_white = np.array([0, 0, 195])
        self.upper_white = np.array([147, 30, 255])

        # Faixa de cor para o preto
        self.lower_black = np.array([0, 38, 75])
        self.upper_black = np.array([134, 173, 158])

        # Configurações de texto
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.6
        self.font_color = (255, 0, 200)
        self.font_thickness = 1

        # Parâmetros de controle
        self.wheel_distance = 0.075  # Distância entre as rodas do robô (em metros)

        # Parâmetros de controle do movimento
        self.target_distance_threshold = 0.007  # Limiar de distância para considerar que o robô chegou ao alvo (em metros)
        self.linear_speed = 0.1  # Velocidade linear do robô (metros por segundo)
        self.angular_speed = 0.5  # Velocidade angular do robô (radianos por segundo)

        # Variáveis para controle PID
        self.last_error = 0

        # Fator de conversão de pixels para metros
        self.pixels_per_meter = 250

        # Tamanho da tela
        self.width = 640  # Defina isso para o tamanho correto da sua câmera
        self.height = 480  # Defina isso para o tamanho correto da sua câmera

        # Coordenadas de corte da imagem
        self.crop_x = 0
        self.crop_y = 0
        self.crop_width = 640  # Use a largura correta da sua câmera
        self.crop_height = 480  # Use a altura correta da sua câmera

        # Parâmetros do filtro Gaussiano
        self.gaussian_kernel_size = (3, 3)  # Tamanho do kernel (deve ser ímpar)
        self.gaussian_sigma = 1  # Desvio padrão

        # Área mínima e máxima para considerar um retângulo relevante (ajustável)
        self.min_area = 16000  # Valor inicial da área mínima, pode ser ajustado dinamicamente
        self.max_area = 18000  # Valor inicial da área máxima, pode ser ajustado dinamicamente

        self.state = 'GO_TO_BALL'

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Aplica filtro Gaussiano para reduzir ruídos
        cv_image = cv2.GaussianBlur(cv_image, self.gaussian_kernel_size, self.gaussian_sigma)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Máscaras para as cores
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask_orange = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        mask_white = cv2.inRange(hsv, self.lower_white, self.upper_white)
        mask_black = cv2.inRange(hsv, self.lower_black, self.upper_black)

        # Encontrar contornos das cores
        green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        white_contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        black_contours, _ = cv2.findContours(mask_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Verifica a detecção das cores
        detected_green = len(green_contours) > 0
        detected_blue = len(blue_contours) > 0
        detected_orange = len(orange_contours) > 0
        detected_white = len(white_contours) > 0
        detected_black = len(black_contours) > 0

        distance = 0
        rect_center = (0, 0)

        if detected_green and detected_blue and detected_orange:
            green_moments = cv2.moments(mask_green)
            blue_moments = cv2.moments(mask_blue)
            orange_moments = cv2.moments(mask_orange)

            green_center = (int(green_moments['m10'] / green_moments['m00']), int(green_moments['m01'] / green_moments['m00']))
            blue_center = (int(blue_moments['m10'] / blue_moments['m00']), int(blue_moments['m01'] / blue_moments['m00']))
            orange_center = (int(orange_moments['m10'] / orange_moments['m00']), int(orange_moments['m01'] / orange_moments['m00']))

            # Calcula a distância entre a bola e o robô
            distance = self.calculate_distance_cm(orange_center, blue_center)

            # Encontra todos os contornos
            all_contours = green_contours + blue_contours

            best_rect = None
            max_area = 0
            min_distance_to_center = float('inf')
            center_x = self.width // 2
            center_y = self.height // 2

            for contour in all_contours:
                x, y, w, h = cv2.boundingRect(contour)
                area = w * h

                if area < self.min_area or area > self.max_area:
                    continue  # Ignora retângulos fora do intervalo de área

                rect_center_x = x + w // 2
                rect_center_y = y + h // 2

                distance_to_center = sqrt((rect_center_x - center_x) ** 2 + (rect_center_y - center_y) ** 2)

                if distance_to_center < min_distance_to_center:
                    min_distance_to_center = distance_to_center
                    max_area = area
                    best_rect = (x, y, w, h)

            if best_rect:
                x, y, w, h = best_rect
                # Desenha o retângulo amarelo na imagem
                cv_image = self.draw_rectangle(cv_image, x, y, w, h, (0, 255, 255))

            # Desenha linhas
            cv_image = self.draw_arrows(cv_image, green_center, blue_center, (x + w // 2, y + h // 2), orange_center)

            # Detecta o maior contorno preto e define a ROI
            if detected_black:
                max_black_area = 0
                black_rect = None
                for contour in black_contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    area = w * h
                    if area > max_black_area:
                        max_black_area = area
                        black_rect = (x, y, w, h)
                
                if black_rect:
                    bx, by, bw, bh = black_rect
                    cv_image = cv2.rectangle(cv_image, (bx, by), (bx + bw, by + bh), (255, 0, 0), 2)  # Contorno azul

                    # Região de corte para aplicar o filtro Canny (definida pela área preta)
                    roi = cv_image[by:by + bh, bx:bx + bw]

                    # Aplicar Filtro de Canny para detectar bordas
                    roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
                    edges = cv2.Canny(roi_gray, 50, 700, apertureSize=3)

                    # Encontre os contornos das bordas
                    contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                    # Encontre o maior contorno de retângulo perfeito com bordas brancas dentro da área preta
                    max_area = 0
                    best_canny_rect = None
                    for contour in contours:
                        perimeter = cv2.arcLength(contour, True)
                        approx = cv2.approxPolyDP(contour, 0.05 * perimeter, True)

                        if len(approx) == 4:
                            x, y, w, h = cv2.boundingRect(approx)
                            area = w * h
                            if area > max_area and self.min_area < area < self.max_area:
                                max_area = area
                                rect_center_x = x + w // 2
                                rect_center_y = y + h // 2
                                best_canny_rect = (x + bx, y + by, w, h)

                    # Desenha o maior retângulo detectado pelo filtro Canny
                    if best_canny_rect:
                        x, y, w, h = best_canny_rect
                        cv_image = cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Contorno verde
                        # Desenha a seta do começo da bola até o centro do retângulo verde
                        rect_center = (x + w // 2, y + h // 2)
                        cv_image = cv2.arrowedLine(cv_image, orange_center, rect_center, (0, 255, 0), 2)

                    cv2.imshow("Filtro Canny", edges)
                    cv2.waitKey(1)

            # Define o destino dependendo do estado atual
            if self.state == 'GO_TO_BALL':
                target_center = orange_center
                if distance < self.target_distance_threshold * self.pixels_per_meter:
                    self.state = 'GO_TO_GOAL'
            else:
                target_center = rect_center
                # Adiciona a verificação para voltar ao estado 'GO_TO_BALL'
                if distance > self.target_distance_threshold * self.pixels_per_meter:
                    self.state = 'GO_TO_BALL'

            # Publica marcadores
            self.publish_robot_marker(Pose(Point(x, y, 0), (0, 0, 0, 1)), "square_start")
            self.publish_robot_marker(Pose(Point(orange_center[0], orange_center[1], 0), (0, 0, 0, 1)), "square_end")

            # Calcula as velocidades do robô
            linear_velocity, angular_velocity = self.calculate_velocities(target_center, blue_center, green_center)

            # Calcula as velocidades das rodas
            left_wheel_vel, right_wheel_vel = self.calculate_wheel_velocities(linear_velocity, angular_velocity, distance)

            # Publica velocidades das rodas
            wheel_vel_msg = Float64MultiArray()
            wheel_vel_msg.data = [left_wheel_vel, right_wheel_vel]
            self.wheel_vel_pub.publish(wheel_vel_msg)

            # Publica velocidades do robô
            vel_msg = Twist()
            vel_msg.linear.x = linear_velocity
            vel_msg.angular.z = angular_velocity
            self.vel_pub.publish(vel_msg)

            # Adiciona informações na imagem
            cv_image = self.add_text(cv_image, f'Vel angular {angular_velocity:.2f} rad/s', (10, 20))
            cv_image = self.add_text(cv_image, f'Motor Left: {left_wheel_vel:.2f}, Motor Right: {right_wheel_vel:.2f}', (10, 40))
            cv_image = self.add_text(cv_image, f'Bola: ({orange_center[0]}, {orange_center[1]})', (10, 60))
            cv_image = self.add_text(cv_image, f'Robo: ({blue_center[0]}, {blue_center[1]})', (10, 80))
            cv_image = self.add_text(cv_image, f'Distancia: {distance:.2f} cm', (10, 100))
            cv_image = self.add_text(cv_image, f'Area Gol: {max_area:.2f} px', (10, 120))
            cv_image = self.add_text(cv_image, f'Status: {self.state}', (10, 140))

        cropped_image = cv_image[self.crop_y:self.crop_y+self.crop_height, self.crop_x:self.crop_x+self.crop_width]

        cv2.imshow("Color Detection", cropped_image)
        cv2.waitKey(1)

    def draw_rectangle(self, image, x, y, w, h, color):
        return cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)

    def draw_arrows(self, image, green_center, blue_center, square_center, orange_center):
        image = cv2.arrowedLine(image, blue_center, green_center, (0, 0, 255), 2)
        image = cv2.arrowedLine(image, square_center, orange_center, (255, 255, 255), 2)
        return image

    def add_text(self, image, text, position):
        return cv2.putText(image, text, position, self.font, self.font_scale, self.font_color, self.font_thickness, cv2.LINE_AA)

    def calculate_velocities(self, target_center, robot_center, orientation_center):
        # Calculate angle between robot orientation and desired orientation
        desired_orientation = atan2(target_center[1] - robot_center[1], target_center[0] - robot_center[0])
        robot_orientation = atan2(orientation_center[1] - robot_center[1], orientation_center[0] - robot_center[0])
        angular_difference = atan2(sin(desired_orientation - robot_orientation), cos(desired_orientation - robot_orientation))

        # Calculate linear velocity
        linear_velocity = self.linear_speed

        # Calculate angular velocity
        angular_velocity = angular_difference * self.angular_speed

        return linear_velocity, angular_velocity

    def calculate_distance_cm(self, point1, point2):
        distance_pixels = sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)
        distance_meters = distance_pixels / self.pixels_per_meter
        distance_cm = distance_meters * 10 #Transformar para CM
        return distance_cm

    def calculate_wheel_velocities(self, linear_velocity, angular_velocity, distance):
        max_velocity = 40
        angular_velocity *= 10

        kp = 1.1
        kd = 0.1
        ki = 0

        left_wheel_vel = 0
        right_wheel_vel = 0

        error = angular_velocity

        derivative = (error - self.last_error) * kd
        integral = (error + self.last_error) * ki
        velocity = error * kp + integral + derivative

        if velocity > max_velocity:
            velocity = max_velocity

        if velocity < -max_velocity:
            velocity = -max_velocity

        left_wheel_vel = max_velocity + velocity
        right_wheel_vel = max_velocity - velocity

        self.last_error = error


        return left_wheel_vel, right_wheel_vel

    def publish_robot_marker(self, pose, marker_id):
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
        marker.id = marker_id

        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = ColorDetectionNode()
    node.run()
