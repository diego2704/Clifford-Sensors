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

  Serial.println("MCP2515 Initialized");
}

void loop() {
  // Chequear si hay datos disponibles en el puerto serial
  if (Serial.available()) {
    // Leer el mensaje del puerto serial
    String message = Serial.readStringUntil('\n');
    // Enviar el mensaje, dividiéndolo en tramas
    sendMessageInFrames(message.c_str());
  }
}

void sendMessageInFrames(const char* message) {
  struct can_frame canMsg; // Estructura para una trama CAN
  uint16_t can_id = 0x121; // ID base para las tramas CAN
  size_t len = strlen(message); // Longitud del mensaje
  size_t frames = (len + 7) / 8; // Número de tramas necesarias

  for (size_t i = 0; i < frames; i++) {
    canMsg.can_id = can_id + i; // Asignar un ID único a cada trama
    canMsg.can_dlc = 8; // Cada trama tiene hasta 8 bytes

    // Copiar los bytes correspondientes del mensaje en la trama
    for (size_t j = 0; j < 8; j++) {
      size_t index = i * 8 + j;
      if (index < len) {
        canMsg.data[j] = message[index]; // Asignar carácter
      } else {
        canMsg.data[j] = 0; // Rellenar con ceros si no hay más datos
      }
    }

    // Enviar la trama CAN
    mcp2515.sendMessage(&canMsg);

    // Imprimir confirmación de la trama enviada
    Serial.print("Trama CAN ");
    Serial.print(i + 1);
    Serial.println(" enviada.");
  }
}
