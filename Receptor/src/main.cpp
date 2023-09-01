#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>

// Pinout do Lora
#define LORA_MISO 19
#define LORA_CS 18
#define LORA_MOSI 27
#define LORA_SCK 5
#define LORA_RST 14
#define LORA_IRQ 26 

// Frequência do LORA -> 433hz ou 915hz
#define LORA_BAND 872E6

String data;
void onReceive(int packetSize) {
  Serial.println("Pacote recebido");

  
  if (packetSize) {
    while (LoRa.available()) {
      data = LoRa.readString();
      Serial.println("Data:" + data);
    }
  }
  Serial.println(data);
}
void initLoRa() {
  Serial.println("Inicializando Lora ...");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  // Start LoRa using the frequency
  int result = LoRa.begin(LORA_BAND);
  if (result != 1) {
    Serial.println("Falha na inicialização do Lora");
    for (;;);
  }
 // LoRa.onReceive(onReceive);
 // LoRa.receive();
  Serial.println("LoRa inicializado");
  delay(2000);
}
void setup() {
  Serial.begin(9600);
  Serial.println("Configurando LoRa Reciever...");
  initLoRa();
}
void loop() {
  
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    //Recebimento do pacote
    Serial.print("Pacote recebido ");
    //Leitura do Pacote
    while (LoRa.available()) {
      data = LoRa.readString();
      Serial.println(data);
    }
  }
}