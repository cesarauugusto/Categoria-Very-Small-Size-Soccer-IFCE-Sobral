#include <Arduino.h>

#define PWMA 25
#define PWMB 5
#define ENA 19
#define ENB 18

void setup()
{

  // IO5 -- PWMA -- D10
  // IO23 -- PWMB -- D11

  // IO19 -- ENA --D12
  // IO18 -- ENB -- D13

  pinMode(19, OUTPUT);
  pinMode(18, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(25, OUTPUT);

  ledcAttachPin(23, 0);//Atribuimos o pino 23 ao canal 0.
  ledcSetup(0, 1000, 10);//Atribuimos ao canal 0 a frequencia de 1000Hz com resolucao de 10bits.
  ledcWrite(0, 256);//Escrevemos um duty cycle de 0 no canal 0.

  ledcAttachPin(5, 1);//Atribuimos o pino 5 ao canal 1.
  ledcSetup(1, 1000, 10);//Atribuimos ao canal 1 a frequencia de 1000Hz com resolucao de 10bits.
  ledcWrite(1, 0);//Escrevemos um duty cycle de 0 no canal 1.
}

void loop()
{
 // ledcWrite(ledCanal, dutyCycle);
  ledcWrite(1, 512);
  ledcWrite(0, 1023);

  digitalWrite(ENA, 1);
  digitalWrite(ENB, 1);
  delay(2000);
  digitalWrite(ENA, 0);
  digitalWrite(ENB, 0);
  delay(2000);

}