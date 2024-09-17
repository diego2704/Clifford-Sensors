#include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>

// MCP2515
MCP2515 mcp2515(10); // Pin CS del MCP2515

// Variables para el sensor MQ-4
int lectura_sensor;
int valor_limite_min = 300;  // Valor mínimo para enviar
int valor_limite_max = 1000; // Valor máximo para enviar

void setup() {
  Serial.begin(115200);

  // Inicialización del MCP2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); // Configurar la velocidad del bus CAN (1 Mbps) y cristal de 8 MHz
  mcp2515.setNormalMode(); // Modo normal de operación
  Serial.println("MCP2515 Initialized");
  
}

void loop() {

  struct can_frame canMsg;

  // Leer el valor del sensor MQ-4
  lectura_sensor = analogRead(A0);
  
  // Solo enviar el valor si está dentro del rango especificado
  if (lectura_sensor >= valor_limite_min && lectura_sensor <= valor_limite_max) {
    // Convertir el valor del sensor a una cadena de texto
    char textBuffer[10];
    snprintf(textBuffer, sizeof(textBuffer), "%d", lectura_sensor); // Convertir el entero a cadena

    // Enviar el valor del sensor MQ-4 como cadena de caracteres
    canMsg.can_id = 0x104; // ID para lectura_sensor_texto
    canMsg.can_dlc = 8;   // Tamaño máximo de los datos del mensaje en bytes

    // Asegúrate de que la cadena de texto cabe en 8 bytes
    memset(canMsg.data, 0, sizeof(canMsg.data));
    strncpy((char*)canMsg.data, textBuffer, 8);

    mcp2515.sendMessage(&canMsg);
    Serial.println("Enviado lectura_sensor como texto");

    // Imprimir el valor enviado
    Serial.print("Valor del sensor enviado: ");
    Serial.println(lectura_sensor);
  }

  delay(1000); // Esperar 1 segundo antes de leer nuevamente
}
