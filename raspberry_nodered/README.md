## Explicación de los flujos
## Instalación
Para configurar el Raspberry debe instalar los siguientes nodos:
* [node-red-contrib-generic-ble en node-red](https://flows.nodered.org/node/node-red-contrib-generic-ble)
* [node-red-dashboard](https://flows.nodered.org/node/node-red-dashboard)
* [node-red-node-ui-list](https://flows.nodered.org/node/node-red-node-ui-list/in/590bc13ff3a5f005c7d2189bbb563976)

## Leer datos BLE
Para leer los datos del ESP32 encargado de recibir los mensajes de la malla ESP-NOW, importe el flujo **read_BLE**. Inyecte los nodos **connect** para establecer la conexión con el servidor BLE y **notify** leer los datos. Para este flujo los mensajes que se reciben se encapsulan para enviarlos a los respectivos **topics** del broker MQTT, usted puede modificar estos nodos de acuerdo a sus necesidades.

![Read BLE](/resources/images/read_BLE.jpg)

## Escribir datos BLE
Para escribir o enviar datos al servidor BLE debe enviar un buffer al nodo de BLE. En este flujo se receptan datos del broker MQTT, luego pasan por un nodo **function** encargado de realizar la conversión a buffer, luego este es enviado al servidor de BLE.

![Write BLE](/resources/images/write_BLE.jpg)

## Lista de nodos
El siguiente flujo recibe datos de los nodos que se conectan a la red del broker MQTT. Luego se formatean estos datos para enviarlos a un nodo lista del dashboard en node-red. Antes de conectar algún nodo a la red, le segerimos que primero active el nodo **Init list of nodes** para crear la lista que almacenara el nombre y el estado de los nodos que se conecten a la red.

![Lst Nodes](/resources/images/list%20nodes_node-red.jpg)