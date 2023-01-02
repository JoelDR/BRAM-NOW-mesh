## MESH ESP-NOW WITH AES ENCRYPTION

**Características:**
- Lista de ruta. Si el mensaje ya ha pasado por un nodo, este no vuelve a retransmitir.
- Los datos del mensaje están cifrados con AES.
- Placas ESP32- Lopy4
- Cada nodo puede comunicarse con otros.
- Funciona con ESP-NOW broadcast
- Envío y recepción de mensajes.
- Keep A Live de los nodos
- Cnnfiguración del ID de cada nodo.
- Inicio instantáneo después del encendido.

### Arquitectura de la red MESH-NOW
Esta red en malla esta conformado por un Smart Home Server, en este caso Home Assitant (Hassio) para administrar la red mediante un dashboard, segundo un gateway encargado de comunicar Hassio con la malla ESP-NOW. Este último utiliza el protocolo ESP-NOW para que los nodos puedan comunicarse entre sí y con el gateway.

<img src="./resources/images/Architecture.jpg" width="500" height="510" alt="Architecture">

### CREAR UN NODO

```c++
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

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
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

```

### CREAR EL GATEWAY

#### Código para el ESP32

```c++
#include <Arduino.h>
#include <BLEDevice.h>
#include <ArduinoJson.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include "mbedtls/aes.h"

//BLE server name
#define bleServerName "BME280_ESP32"

#define BOARD_ID 10

typedef struct struct_message {
  int destinyId;
  char data[16];
  bool connected;
  uint8_t routes[10];
} struct_message;

struct_message message;
esp_now_peer_info_t peerInfo;
BLECharacteristic *pCharacteristic;
BLEServer* pServer = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;
float txtValue = 0;

char * key = "3450258945085207";


// Defining the service and characteristics
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define CHARACTERISTIC_UUID_TX "ea12ce72-e2f6-44c5-ac3f-51fb791a7c99"
#define CHARACTERISTIC_UUID_RX "89844d3b-e63d-479e-af2c-1f97e6fc5651"

// Broadcast Address
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

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

//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    StaticJsonDocument<256> messageJSON;
    deserializeJson(messageJSON, rxValue);

    // Encrypt data
    unsigned char messageOut[16];
    char messageInput[16];
    String light = messageJSON["light"];
    light.toCharArray(messageInput, 16);
    if (message.routes[0] != 0)
    {
      clearRoute();
    }
    
    message.routes[0] = BOARD_ID;
    encrypt(messageInput, key, messageOut);
    memcpy(&message.data, messageOut, sizeof(message.data));

    // Set values to send
    message.destinyId = messageJSON["id"];
    //strcpy(message.data, messageJSON["light"]);

    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &message, sizeof(message));
  }
};

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&message, incomingData, sizeof(message));
  char macStr[18]="";
  char messageIncoming[16];
  
  // Checks if the message is for the node
  if(message.destinyId == BOARD_ID) {
    decrypt((unsigned char*)message.data, key, (unsigned char*)messageIncoming);
    pCharacteristic->setValue(messageIncoming);
    pCharacteristic->notify();

  } else {
    bool retransmission = true;

    // Check the list to see if the node id is already added.
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

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Start serial communication 
  Serial.begin(115200);

  // Create the BLE Device
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE characteristics

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  
  // BLE2902 needed to notify
  pCharacteristic->addDescriptor(new BLE2902());

  // Characteristic for receiving end
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    BLECharacteristic::PROPERTY_WRITE
  );

  pCharacteristic->setCallbacks(new MyCallbacks());
  // Strat the service
  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

   WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // register broadcast address     
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}
```

### 
