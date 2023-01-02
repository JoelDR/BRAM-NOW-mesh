### Explicación del código

Defina el nombre de su servidor BLE, puede ser el que usted desee.

```c++
#define bleServerName "BME280_ESP32"
```

Aquí define el ID que tendrá el gateway. Es importante que este ID sea diferente de cero ya que la variable de rutas se inicializa por defecto con ceros. Recomendamos que el valor del ID sea mayor al número de nodo que agregará la red.
```c++
#define BOARD_ID 10
```

Establezca la clave con la que trabajará el cifrado AES. Esta llave debe ser la misma que este definida en los nodos.
```c++
char * key = "3450258945085207";
```

Debe establecer el UUID (Universally Unique Identifier) para el servidor BLE y las características. Puede generar los UUID en el siguiente enlace:

https://www.uuidgenerator.net/


```c++
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define CHARACTERISTIC_UUID_TX "ea12ce72-e2f6-44c5-ac3f-51fb791a7c99"
#define CHARACTERISTIC_UUID_RX "89844d3b-e63d-479e-af2c-1f97e6fc5651"
```

La función callback llamado **MyCallbacks** recibe los mensajes del cliente BLE, aquí se encapsulan los datos para enviarlos a los nodos por ESP-NOW

```c++
class MyCallbacks: public BLECharacteristicCallbacks
```