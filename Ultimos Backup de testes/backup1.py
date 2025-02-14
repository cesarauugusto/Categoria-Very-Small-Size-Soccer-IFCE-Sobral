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
        self.lower_green = np.array([76, 71, 184])
        self.upper_green = np.array([95, 229, 255])

        self.lower_blue = np.array([84, 135, 184])
        self.upper_blue = np.array([121, 255, 255])

        self.lower_orange = np.array([3, 23, 255])
        self.upper_orange = np.array([55, 255, 255])

        # Faixa de cor para linhas brancas do campo
        self.lower_white = np.array([0, 0, 195])
        self.upper_white = np.array([147,30, 255])

        # Configurações de texto
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.font_scale = 0.6
        self.font_color = (255, 0, 200)
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
        self.pixels_per_meter = 250

        # Tamanho da tela
        self.width = 680 # Nova largura desejada
        self.height = 480  # Nova altura desejada

        # Coordenadas de corte da imagem
        self.crop_x = 0
        self.crop_y = 0
        self.crop_width = 680
        self.crop_height = 390

    def image_callback(self, data):
        #try:
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        #except CvBridgeError as e:
        #    rospy.logerr(e)
        #    return

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Mascara para as cores
        mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
        mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        mask_orange = cv2.inRange(hsv, self.lower_orange, self.upper_orange)

        # Nova máscara para linhas brancas
        mask_white = cv2.inRange(hsv, self.lower_white, self.upper_white)

        green_contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        orange_contours, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Detecta contornos das linhas brancas
        white_contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Verifica a detecção das cores
        detected_green = len(green_contours) > 0
        detected_blue = len(blue_contours) > 0
        detected_orange = len(orange_contours) > 0
        detected_white = len(white_contours) > 0

        distance = 0

        if detected_green and detected_blue and detected_orange:
            green_moments = cv2.moments(mask_green)
            blue_moments = cv2.moments(mask_blue)
            orange_moments = cv2.moments(mask_orange)

            green_center = (int(green_moments['m10'] / green_moments['m00']), int(green_moments['m01'] / green_moments['m00']))
            blue_center = (int(blue_moments['m10'] / blue_moments['m00']), int(blue_moments['m01'] / blue_moments['m00']))
            
            # Simulação 1: Define manualmente a posição onde o robo deve ir
            #orange_center = (300, 200)
            
            # Simulação 2: Descomente a linha acima e comente a linha abaixo para usar a simulação 1
            orange_center = (int(orange_moments['m10'] / orange_moments['m00']), int(orange_moments['m01'] / orange_moments['m00']))

            # Calcula a distância entre a bola e o robô
            distance = self.calculate_distance_cm(orange_center, blue_center)

            # Encontra o retângulo que envolve todas as áreas detectadas
            x, y, w, h = cv2.boundingRect(np.vstack(green_contours + blue_contours))

            # Desenha o retângulo na imagem
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 255), 2)

            # Desenha linhas
            cv2.arrowedLine(cv_image, blue_center, green_center, (0, 0, 255), 2)
            cv2.arrowedLine(cv_image, (x + w // 2, y + h // 2), orange_center, (255, 255, 255), 2)

            # Região de corte para aplicar o filtro Canny
            corte = mask_white[:, self.crop_width // 2:]

            # Aplicar Filtro de Canny para detectar bordas
            edges = cv2.Canny(corte, 50, 50, apertureSize=5, L2gradient=True)

            # Encontre os contornos das bordas
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Para cada contorno, verifique se é um retângulo
            for contour in contours:
                # Aproxime o contorno por um polígono
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.03 * perimeter, True)
                
                # Se o polígono tiver quatro vértices, é um retângulo
                if len(approx) == 4:
                    # Desenhe o retângulo na imagem original
                    cv2.drawContours(cv_image[:, self.crop_width // 2 :], [approx], -1, (0, 255, 0), 2)

            cv2.imshow("Filtro Canny", edges)
            cv2.waitKey(1)

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

        cropped_image = cv_image[self.crop_y:self.crop_y+self.crop_height, self.crop_x:self.crop_x+self.crop_width]

        cv2.imshow("Color Detection", cropped_image)
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
        distance_cm = distance_meters * 620
        return distance_cm

    def calculate_wheel_velocities(self, linear_velocity, angular_velocity, distance):
        max_velocity = 41
        angular_velocity *= 10
        # Parâmetros de controle PID
        kp = 1.1
        kd = 0.09
        ki = 0

        left_wheel_vel = 0
        right_wheel_vel = 0


        # Calcula o erro
        error = angular_velocity

        if error != 0 and distance > 20:
            # Calcula o termo derivativo
            derivative = (error - self.last_error) * kd
            integral = ((error + self.last_error) * ki)
            # Calcula a velocidade
            velocity = error * kp + integral + derivative

            if velocity > max_velocity:
                velocity = max_velocity

            if velocity < -max_velocity:
                velocity = -max_velocity

            # Calcula as velocidades dos motores
            left_wheel_vel = max_velocity + velocity
            right_wheel_vel = max_velocity - velocity

            # Atualiza o erro anterior
            self.last_error = error

        elif distance<100:
            left_wheel_vel = 0
            right_wheel_vel = 0

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
