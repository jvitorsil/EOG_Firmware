#include <Arduino.h>
#include <stdio.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <cstring>
#include <sstream>

#define CHANNEL_1 32
#define CHANNEL_2 33

#define SSID "iPhone"
#define PASSWORD "senhasenha"

#define MQTT_TOPIC "TOPIC_Channel1"

#define MQTT_BROKER "172.20.10.5"
#define MQTT_PORT 1883
#define MQTT_Client "EOG_Hardware" 

#define FS 120

struct DataPacket{
  uint16_t channel1[20];
  uint16_t channel2[20];
};

DataPacket dataPacket;

WiFiClient espClient;
PubSubClient client(espClient);

void adcTask(void *pvParameters);

void setup() {
  Serial.begin(115200);
  
  Serial.println("Connect WiFi");
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED){
    Serial.println("Connectando...");
    delay(1000);
  }
  
  Serial.println("Connect MQTT");
  client.setServer(MQTT_BROKER, MQTT_PORT);
  while(!client.connected()){
    Serial.println("Connectando MQTT...");
    if(client.connect("ESP32Client"))
    delay(500);
  }
  analogReadResolution(12);

  xTaskCreate(adcTask, "ADCTask", 4096, NULL, 1, NULL);
}

void loop() {
}


void adcTask(void *pvParameters) {

  TickType_t xLastWakeTime = xTaskGetTickCount();

  uint8_t index = 0;
    
  std::stringstream s1, s2;

  while (1) {
    dataPacket.channel1[index] = analogRead(CHANNEL_1);
    dataPacket.channel2[index] = analogRead(CHANNEL_2);
    
    s1 << std::to_string(dataPacket.channel1[index]) << ";";
    s2 << std::to_string(dataPacket.channel2[index]) << ";";

    index++; 

    if(index >= sizeof(dataPacket.channel1)/sizeof(dataPacket.channel1[0])){

      std::string payload = s1.str() + " " + s2.str();

      client.publish(MQTT_TOPIC, payload.c_str());

      s1.str(""); s2.str("");
      s1.clear(); s2.clear();
      index = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(1000/FS));  // Delay de 1000 milissegundos
  }
}
