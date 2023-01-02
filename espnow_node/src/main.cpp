#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "mbedtls/aes.h"

// Change the node ID
#define BOARD_ID 1

char * key = "3450258945085207";

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

unsigned long previousMillis = 0;
const long interval = 120000;

typedef struct struct_message {
  int destinyId;
  char data[16];
  bool connected;
  uint8_t routes[10];
} struct_message;

// Create a struct_message called myData
struct_message message;
struct_message keepALive;

esp_now_peer_info_t peerInfo;

void encrypt(char * plainText, char * key, unsigned char * outputBuffer){
  mbedtls_aes_context aes;

  mbedtls_aes_init( &aes );
  mbedtls_aes_setkey_enc( &aes, (const unsigned char*) key, strlen(key) * 8 );
  mbedtls_aes_crypt_ecb( &aes, MBEDTLS_AES_ENCRYPT, (const unsigned char*)plainText, outputBuffer);
  mbedtls_aes_free( &aes );
}

void decrypt(unsigned char * chipherText, char * key, unsigned char * outputBuffer){
  mbedtls_aes_context aes;

  mbedtls_aes_init( &aes );
  mbedtls_aes_setkey_dec( &aes, (const unsigned char*) key, strlen(key) * 8 );
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_DECRYPT, (const unsigned char*)chipherText, outputBuffer);
  mbedtls_aes_free( &aes );
}

void clearRoute() {
  for (size_t i = 0; i < sizeof(message.routes); i++)
  {
    if (message.routes[i] != 0)
      message.routes[i] = 0;
    else
      i = sizeof(message.routes);
  }
}

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&message, incomingData, sizeof(message));
  // Verify that the message is this node
  if(message.destinyId == BOARD_ID) {
    char messageIncoming[16];
    decrypt((unsigned char*)message.data, key, (unsigned char*)messageIncoming);

    Serial.print("message received: ");
    Serial.print(messageIncoming);
    clearRoute();

  } else {
    bool retransmission = true;

    for (size_t i = 0; i < sizeof(message.routes); i++)
    {
      if (message.routes[i] == BOARD_ID)
      {
        retransmission = false;
        i = sizeof(message.routes);
      } else if(message.routes[i] == 0){
        message.routes[i] = BOARD_ID;
        i = sizeof(message.routes);
      }
    }
    
    if (retransmission)
      esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
  }
}

void dataToKeepALive() {
  char text[16];
  unsigned char messageOut[16];
  keepALive.destinyId = 20;
  keepALive.connected = true;
  itoa(BOARD_ID, text, 10);
  strncat(text, ",ON", 3);
  keepALive.routes[0] = BOARD_ID;
  encrypt(text, key, messageOut);
  
  memcpy(&keepALive.data, messageOut, sizeof(keepALive.data));
  esp_now_send(broadcastAddress, (uint8_t *) &keepALive, sizeof(keepALive));
  Serial.println("Send keep a live");
}

void setup() {

  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Once ESP-NOW is successfully started, the callbacks for sending and receiving messages are registered.
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  dataToKeepALive();
}

void loop() {
  // Put your code here
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
  previousMillis = currentMillis;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &keepALive, sizeof(keepALive));
    if(result == ESP_OK)
      Serial.print("Sending keep a live, ");
    else
      Serial.print("An error has occurred while sending keep to alive, ");
  }
}
