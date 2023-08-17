#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include "GY521.h"
#include <SoftwareSerial.h>
#include <TinyGPS.h>

Adafruit_BMP280 bmp;    ///< Definição I2C do BMP

GY521 sensor(0x68);     ///< Definição I2c do GY

/* Declaração das variáveis do BMP */
float temperature;  
float pressure;
int32_t altitude;

/* Declaração das variáveis do GY */
float x;
float y;
float z;

/* Definir as portas que serão utilizadas como RX e TX no GPS */
SoftwareSerial serial1(34, 35); // RX, TX
TinyGPS gps1;

/* Declaração das variáveis do GPS */
long latitude, longitude;
unsigned long idadeInfo;

void setup() {
  
  Serial.begin(9600);
  serial1.begin(9600);

  /* Inicialização do BMP280 */
  Serial.println(F("Inicializando BMP280"));

  if (!bmp.begin(0x76)) {
    Serial.println(F("Não foi possível achar o sensor, cheque a conexão "
                      "ou tente outro endereço!"));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_FORCED,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  /*-------------------------------------------------------------------------------------------*/

  /* Inicialização do GY521 */
  Wire.begin();

  delay(100);
  if (sensor.wakeup() == false)
  {
    Serial.print(millis());
    Serial.println("\tNão foi possível conectar o GY521");
  }
  sensor.setAccelSensitivity(2);  //  8g
  sensor.setGyroSensitivity(1);   //  500 degrees/s

  sensor.setThrottle();
  Serial.println("start...");

  /* Calibração do sensor */
  sensor.axe = 0.574;
  sensor.aye = -0.002;
  sensor.aze = -1.043;
  sensor.gxe = 10.702;
  sensor.gye = -6.436;
  sensor.gze = -0.676;
  /*-------------------------------------------------------------------------------------------*/


}

void loop() {
  
  /* Medidas do BMP */
  if (bmp.takeForcedMeasurement()) {
    
    temperature = bmp.readTemperature();   ///< Variável que armazena temperatura

    pressure = bmp.readPressure();         ///< Variável que armazena pressão

    altitude = bmp.readAltitude(1013.25);  ///< Variável que armazena altitude          

  } else {
    Serial.println("BMP falhou");
  }
  /*-------------------------------------------------------------------------------------------*/

  /* Medidas do GY 521 */
  sensor.read();
  float x = sensor.getAngleX();
  float y = sensor.getAngleY();
  float z = sensor.getAngleZ();
  /*-------------------------------------------------------------------------------------------*/

  /* Medidas do GPS */
  bool recebido = false;          ///< Variável que indica se o sinal de GPS foi recebido

  while (serial1.available()) {
    char cIn = serial1.read();
    recebido = gps1.encode(cIn); ///< Se receber o sinal a variável recebe o valor true
  }

  if (recebido) {
    gps1.get_position(&latitude, &longitude, &idadeInfo);
  }
  /*-------------------------------------------------------------------------------------------*/
  
  /* Criando uma string para armazenar os dados e enviar através do Lora */
  char stemp [120];
  sprintf(stemp, "* %10.6f ; %10.6f ; %10.2f ; %10.2f ; %10.2f ; %10.2f ; %10.2f ; %10.2f*", float(latitude) / 1000000, float(longitude) / 1000000, temperature, pressure, altitude, x, y, z);
  Serial.println(stemp);

}