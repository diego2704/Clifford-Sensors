#include <Wire.h> 
#include <SPI.h>
#include <mcp2515.h>

// MCP2515
MCP2515 mcp2515(10); // Pin CS del MCP2515

// Definir pin del sensor de voltaje
const int sensorPin = A7;

// Definir el factor de conversión para el sensor FZ0430 (divisor de voltaje 5:1)
const float factorConversion = 5.0;

void setup() {
  // Iniciar comunicación serie
  Serial.begin(115200);

  // Inicialización del MCP2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); // Configurar la velocidad del bus CAN (1 Mbps) y cristal de 8 MHz
  mcp2515.setNormalMode(); // Modo normal de operación
  Serial.println("MCP2515 Initialized");
}

void loop() {
  // Estructura de datos para el mensaje CAN
  struct can_frame canMsg;

  // Leer valor analógico del sensor
  int valorAnalogico = analogRead(sensorPin);

  // Convertir el valor analógico a voltaje (Arduino trabaja a 5V y 10 bits, es decir, 1024 niveles)
  float voltajeMedido = (valorAnalogico * 5.0) / 1024.0;

  // Multiplicar por el factor de conversión (5:1 en el caso del FZ0430)
  float voltajeReal = voltajeMedido * factorConversion;

  // Preparar el mensaje CAN para enviar el valor del voltaje
  canMsg.can_id = 0x105; // ID para el mensaje CAN
  canMsg.can_dlc = 4;    // Tamaño de los datos (4 bytes para el valor de voltaje)

  // Convertir el voltaje a formato de 4 bytes (float) para enviar por CAN
  memcpy(canMsg.data, &voltajeReal, sizeof(voltajeReal));

  // Enviar el mensaje CAN
  mcp2515.sendMessage(&canMsg);
  Serial.println("Enviado lectura_sensor");

  // Imprimir el voltaje real en el monitor serie
  Serial.print("Voltaje medido: ");
  Serial.print(voltajeReal);
  Serial.println(" V");

  // Esperar un segundo antes de realizar la siguiente lectura
  delay(1000);
}
