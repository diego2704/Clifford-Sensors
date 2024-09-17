#include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>

MCP2515 mcp2515(10); // Pin CS del MCP2515

void setup() {
  Serial.begin(115200);

  // Inicialización del MCP2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); // Configurar la velocidad de comunicación y la frecuencia del cristal
  mcp2515.setNormalMode(); // Modo normal de operación

  Serial.println("MCP2515 Initialized and ready to receive");
}

void loop() {
  struct can_frame canMsg;

  // Chequear si hay un mensaje CAN disponible
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    // Si hay un mensaje, procesarlo
    receiveMessageInFrames(canMsg);
  }
}

String receivedMessage = ""; // Almacena el mensaje completo
uint16_t expectedCanId = 0x121; // ID base esperado para reconstruir el mensaje

void receiveMessageInFrames(struct can_frame &canMsg) {
  // Verificar si el CAN ID es el esperado
  if (canMsg.can_id == expectedCanId) {
    // Agregar los datos de la trama recibida al mensaje completo
    for (int i = 0; i < canMsg.can_dlc; i++) {
      if (canMsg.data[i] != 0) { // Ignorar los ceros de relleno
        receivedMessage += (char)canMsg.data[i];
      }
    }

    // Actualizar el ID esperado para la próxima trama
    expectedCanId++;

    Serial.print("Trama CAN con ID ");
    Serial.print(canMsg.can_id, HEX);
    Serial.println(" recibida.");
  } else {
    // Si recibimos un CAN ID diferente, significa que el mensaje ha terminado
    if (receivedMessage.length() > 0) {
      Serial.print("Mensaje completo recibido: ");
      Serial.println(receivedMessage);

      // Reiniciar para el próximo mensaje
      receivedMessage = "";
      expectedCanId = 0x121;
    }
  }
}