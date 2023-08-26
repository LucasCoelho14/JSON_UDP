#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <SPI.h>

WiFiUDP udp;

IPAddress raspberryPiIP(255, 255, 255, 255);  // Replace with the Raspberry Pi's IP address (192, 168, 4, 255)
const int udpPort = 1234;

//Inicializando o objeto do sensor BMP280 e MPU6050
Adafruit_BMP280 bmp;
Adafruit_MPU6050 mpu;

//Definindo as informações da rede Wi-Fi
const char* ssid = "Supernova Rocketry";
const char* password = "foguetaos2";

void setup() {
  Serial.begin(9600);
  
  Serial.println();
  Serial.print("Conectando-se a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);
 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to the WiFi network");
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
  udp.begin(udpPort); // Choose a port number

  //Inicializando o sensor BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("Erro ao iniciar o sensor BMP280");
    while (1);
  }
}

void loop() {
  //Leitura dos valores dos sensores
  float temperatura = bmp.readTemperature();
  float pressao = bmp.readPressure() / 100.0; // Conversão para hPa
  float altitude = bmp.readAltitude(1013.25); // Ajuste de pressão ao nível do mar

  //Leitura do MPU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  //Sensores que não fazem parte deste teste
  int experimento0 = 250;
  int experimento1 = 2;
  int bateria = 86;

  StaticJsonDocument<240> jsonBuffer; //Cada par de valores utiliza aproximadamente 16 bytes
                                      //Cada par nome-vetor utiliza aproximadamente 16*(1+N) bytes, em que N é o comprimento do vetor 
  //Criando um objeto JsonObject para armazenar os valores dos sensores
  JsonObject sensores = jsonBuffer.to<JsonObject>();

  //Adicionando os valores dos sensores ao JsonObject
  sensores["equipe"] = 5242;
  sensores["bateria"] = bateria;
  sensores["temperatura"] = temperatura;
  sensores["pressao"] = pressao;
  //sensores["altitude"] = altitude;
  sensores["giroscopio"][0] = g.gyro.x;
  sensores["giroscopio"][1] = g.gyro.y;
  sensores["giroscopio"][2] = g.gyro.z;
  sensores["acelerometro"][0] = a.acceleration.x;
  sensores["acelerometro"][1] = a.acceleration.y;
  sensores["acelerometro"][2] = a.acceleration.z;
  sensores["payload"][0] = experimento0;
  sensores["payload"][1] = experimento1;

  //Convertendo o JsonDocument em uma string JSON
  String jsonString;
  serializeJson(jsonBuffer, jsonString);
  udp.beginPacket(raspberryPiIP, udpPort);
  udp.print(jsonString);
  udp.endPacket();
  // Imprimindo a string JSON no monitor serial
  Serial.println(jsonString);
  
  delay(5000);
}