### Explicación del código

Primero, se incluyen las librerías necesarias para el funcionamiento de este nodo.


```c++
#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "mbedtls/aes.h"
```

En la siguiente línea define el ID del nodo que se usará para identificarlo.
```c++
#define BOARD_ID 1
```

Defina la llave que se utilizará para cifrar los datos con AES. Esta llave debe ser una longuitus de 16 bytes.

```c++
char * key = "3450258945085207";
```

Esta línea de código define la dirección MAC del dispositivo al cual se enviaran los datos. Debe ser una dirección broadcast para que los nodos envien mensajes entre ellos.

```c++
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
```

Luego, se crea una estructura de mensaje que contendrá 4 variables esenciales para el funcionamiento de la red en malla. La primera **destinyId** contendrá el ID del nodo al que vaya destinado el mensaje. La variable **data** contendrá la información que desea enviar a cada nodo y al gateway, como datos de sensores. esta variable es esencial ya que contendrá la información cifrada. La siguiente es **route** que es un arreglo que contendrá el ID de los nodos por los cuales el mensaje es transmitido. 
```c++
typedef struct struct_message {
  int destinyId;
  char data[16];
  bool connected;
  uint8_t routes[10];
} struct_message;
```

Luego creamos dos nuevas variables de tipo **struct_message** llamados **message** y **keepALive**. La primera variable **message** es el principal y es que se utilizará para mandar información a los otros nodos y al gateway. La variable **keepALive** será utilizada únicamente para enviar el estado (activo) del nodo al gateway, esto para no crear conflicto con los mensajes que lleguem y no almacenar toda la información en una misma variable.

```c++
struct_message message;
struct_message keepALive;
```

Para la seguridad se hace uso de los métodos de cifrar y descrifrar, estos deben ser llamados cuando quiera enviar o recibir datos. Reciben como parámetros el texto original sin cifrar, la llave definida previamente, y una variable tipo char que guardará el dato cifrado. Para más información sobre el funcionamiento de la librería AES puede visitar el siguiente enlace:

https://everythingesp.com/esp32-arduino-tutorial-encryption-aes128-in-ecb-mode/

```c++
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
```

Verificamos que el mensaje sea para el nodo, para ello se llama a la variable **destinyID** y se compara con el ID, si es correcto se procede a descrifrar el dato. Para este ejemplo, el dato descrifrado se imprime por consola. Por último, se debe llamar a la función **clearRoute** para limpiar el array de rutas del mensaje.

```c++
if(message.destinyId == BOARD_ID) {
  char messageIncoming[16];
  decrypt((unsigned char*)message.data, key, (unsigned char*)messageIncoming);

  Serial.print("message received: ");
  Serial.print(messageIncoming);
  clearRoute();
}
```


El siguiente algoritmo verifica si el ID del nodo esta en el array de rutas del mensaje, si es así el nodo no retransmite, caso contrario agrega el ID nodo al array y retransmite el mensaje.

```c++
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
```

Para limpiar el array de rutas se llama método **clearRoute** después de haber recibido o enviamos algún mensaje. Este método es importante ya que los mensajes se quedan guardados en memoria, si no se borra el contenido de la variable **routes** definida en la estructura, puede provocar que algunos no reciban mensajes ya que su ID habrá quedado guardado en memoria.
```c++
void clearRoute() {
  for (size_t i = 0; i < sizeof(message.routes); i++)
  {
    if (message.routes[i] != 0)
      message.routes[i] = 0;
    else
      i = sizeof(message.routes);
  }
}
```

El método **datatoKeepAlive** configura los datos necesarios en la variable **keepALive** para enviar el estado del nodos. Debe ser llamada en el **setup()** luego de haber registardo los callback de ESP-NOW. La variable **destiny** debe ser modificada dependiendo el ID que tenga el gateway o el nodo destino al que va ser enviado.

```c++
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
```

Es importante mencionar que cuando se envie un mensaje siempre debe registrarse el ID del nodo como se muestra en la siguiente línea:

```c++
keepALive.routes[0] = BOARD_ID;
```

Por último en el método **loop()** agregue el código que desea ejecutar. Para este ejemplo se coloco el envio de mensajes Keep A Live como se muestra en el siguiente código.

```c++
esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &keepALive, sizeof(keepALive));
if(result == ESP_OK)
  Serial.print("Sending keep a live, ");
else
  Serial.print("An error has occurred while sending keep to alive, ");
```

**es_now_send** recibe como parámetro la dirección MAC, la variable con la información que se va a enviar y el tamaño del mensaje.

```c++
esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &keepALive, sizeof(keepALive));
```
Verifica si el mensaje fue enviado con éxito.

```c++
if(result == ESP_OK)
  Serial.print("Sending keep a live, ");
else
  Serial.print("An error has occurred while sending keep to alive, ");
```
