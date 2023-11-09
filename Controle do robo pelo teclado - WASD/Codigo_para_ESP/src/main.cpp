#define ROSSERIAL_ARDUINO_TCP
#define ESP32

#define PWMA 5  // PWMA
#define ENA 19  // ENA
#define PWMB 23 // PWMB
#define ENB 18  // ENB

#include <WiFi.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3.h>

const char *ssid = "IFCE_Pesquisa01";
const char *password = "sobral@pesquisa";

IPAddress server(192, 168, 70, 133);
const uint16_t serverPort = 11411;

//rosrun rosserial_python serial_node.py tcp 11411

ros::NodeHandle nh;

std_msgs::String controle_msg;

void callback_controle_robo(const std_msgs::String &msg)
{
  String controle = msg.data;
  Serial.print("Mensagem de controle recebida: ");
  Serial.println(controle);

  if (controle == "frente")
  {
    // Lógica para mover o robô para frente
    digitalWrite(ENA, LOW);
    ledcWrite(1, 200);

    digitalWrite(ENB, LOW);
    ledcWrite(2, 200);
  }
  else if (controle == "tras")
  {
  }
  else if (controle == "esquerda")
  {
    digitalWrite(ENA, LOW);
    ledcWrite(1, 300);

    digitalWrite(ENB, LOW);
    ledcWrite(2, 200);
    // Lógica para mover o robô para a esquerda
  }
  else if (controle == "direita")
  {
    // Lógica para mover o robô para a direita
    digitalWrite(ENA, LOW);
    ledcWrite(1, 200);

    digitalWrite(ENB, LOW);
    ledcWrite(2, 300);
  }
  else if (controle == "parar")
  {
    // Lógica para o robo parar
     digitalWrite(ENA, LOW);
    ledcWrite(1, 0);

    digitalWrite(ENB, LOW);
    ledcWrite(2, 0);
  }
  else
  {
    // Lógica para parar o robô
  }
}

ros::Subscriber<std_msgs::String> sub_controle("controle_robo", &callback_controle_robo);

void setup()
{
  pinMode(23, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(18, OUTPUT);

  ledcAttachPin(PWMA, 1);
  ledcAttachPin(PWMB, 2);

  ledcSetup(1, 5000, 10);
  ledcSetup(2, 5000, 10);

  digitalWrite(ENA, HIGH);
  ledcWrite(1, 0);

  digitalWrite(ENB, HIGH);
  ledcWrite(2, 0);

  Serial.begin(9600);
  Serial.println();
  Serial.print("Conectando em ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP Adress: ");
  Serial.println(WiFi.localIP());
  Serial.println("");

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  while (nh.getHardware()->connected() != 1)
  {
    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    delay(50);
    Serial.print(".");
  }
  Serial.print("\nOK TCP-IP = ");
  Serial.println(nh.getHardware()->getLocalIP());
  nh.subscribe(sub_controle);
}

void loop()
{
  if (nh.getHardware()->connected())
  {

    nh.spinOnce();
    delay(1);
  }
  else
  {
    Serial.println("disconnected\n");
    while (nh.getHardware()->connected() != 1)
    {
      nh.getHardware()->setConnection(server, serverPort);
      nh.initNode();
      delay(3);
      Serial.print(".");
    }
  }
}
