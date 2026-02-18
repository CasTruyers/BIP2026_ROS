// ========== LIBRARIES ==========
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <HCSR04.h>

#define SONAR_TRIGGER_PIN 12
#define SONAR_ECHO_PIN 13

// ========== FUNCTION PROTOTYPES ==========
void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len);
esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo);

void OnDataSent(const wifi_tx_info_t* info, esp_now_send_status_t status) { 
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

void OnDataRecv(const esp_now_recv_info* info, const unsigned char* incomingData, int len) {
  // Remote node can also receive feedback if needed
}

esp_err_t addPeer(uint8_t* mac, esp_now_peer_info_t* peerInfo){
  if(peerInfo == NULL){
    peerInfo = (esp_now_peer_info_t*)malloc(sizeof(esp_now_peer_info_t));
    memset(peerInfo, 0, sizeof(esp_now_peer_info_t));
  }
  memcpy(peerInfo->peer_addr, mac, 6);
  peerInfo->channel = 0;
  peerInfo->encrypt = false;
  return esp_now_add_peer(peerInfo);
}


// Variables - Update with gateway MAC address
uint8_t gatewayMAC[6] = {0x28, 0x05, 0xA5, 0x26, 0x80, 0xBC};

void setup() {
  Serial.begin(115200);
  
  HCSR04.begin(SONAR_TRIGGER_PIN, SONAR_ECHO_PIN);
  Serial.println("Sensor initialized");
  
  WiFi.begin();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  addPeer(gatewayMAC, NULL);
  
  Serial.println("Remote sensor ready");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
}

void loop() {
  double* distances = HCSR04.measureDistanceCm();
  // double distances[20] = {20};

  esp_now_send(gatewayMAC, (uint8_t*)distances, sizeof(double));
  
  delay(50);
}