# Testes-Robo-VSSS

Testes de configurações do robô de comepetição very small size soccer

*Utiliza ESPDUINO para processamento de dados e L298 para controle do motor.  

*Utiliza o ambiente ROS para conseguir a integração do Robô com a câmera.

*O arquivo calibration contém um arquivo python para que calibre as cores do robo e da bola atráves do HSV.

*Os valores de HSV que foi calibrado é colocado no arquivo de detectção de cor mais precisamente no color_detection_node.py

o Arquivo color_detection_node é o arquivo principal pois é nele que é por onde tudo começa onde se utiliza a visão computacional para achar as cores do robô, depois que ele identifica ele realiza os calculos de velocidade angular e linear com base nas cores detectado do robô com a referencia da bola, após calculado a velocidade angular, ela é mandada para a função para transformar em velocidade angular em um controle para trasnformar em PWM para enviar essas informações com mais precisão para os motores para que o robo chegue até a bola o mais rapido possivel e com a máxima precisão.
