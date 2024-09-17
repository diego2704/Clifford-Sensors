#include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>

// MCP2515
MCP2515 mcp2515(10); // Pin CS del MCP2515

// Definir pines del sensor de voltaje y corriente
const int sensorVoltajePin = A7;
const int sensorCorrientePin = A1;

// Definir el factor de conversión para el sensor FZ0430 (divisor de voltaje 5:1)
const float factorConversionVoltaje = 5.0;

// Definir la sensibilidad del sensor ACS712 (30A) para la corriente
const float sensibilidadCorriente = 0.066; // 66 mV por amperio para el modelo de 30A
const float offsetCorriente = 2.5; // Offset del sensor (2.5V en reposo)

// Definir cuántas muestras tomar para promediar la corriente
const int numMuestras = 100;

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
  // Estructura de datos para los mensajes CAN
  struct can_frame canMsgVoltaje;
  struct can_frame canMsgCorriente;

  // ===== Medición de Voltaje =====
  // Leer valor analógico del sensor de voltaje
  int valorAnalogicoVoltaje = analogRead(sensorVoltajePin);

  // Convertir el valor analógico a voltaje (Arduino trabaja a 5V y 10 bits, es decir, 1024 niveles)
  float voltajeMedido = (valorAnalogicoVoltaje * 5.0) / 1024.0;

  // Multiplicar por el factor de conversión (5:1 en el caso del FZ0430)
  float voltajeReal = voltajeMedido * factorConversionVoltaje;

  // Preparar el mensaje CAN para enviar el valor del voltaje
  canMsgVoltaje.can_id = 0x105; // ID para el mensaje CAN del voltaje
  canMsgVoltaje.can_dlc = 4;    // Tamaño de los datos (4 bytes para el valor de voltaje)

  // Convertir el voltaje a formato de 4 bytes (float) para enviar por CAN
  memcpy(canMsgVoltaje.data, &voltajeReal, sizeof(voltajeReal));

  // Enviar el mensaje CAN del voltaje
  mcp2515.sendMessage(&canMsgVoltaje);
  Serial.println("Enviado lectura de voltaje por CAN");

  // ===== Medición de Corriente =====
  // Tomar muestras de corriente y calcular el promedio
  float sumaCorriente = 0;
  for (int i = 0; i < numMuestras; i++) {
    int valorAnalogicoCorriente = analogRead(sensorCorrientePin);
    
    // Convertir el valor analógico a voltaje
    float voltajeCorriente = valorAnalogicoCorriente * 5.0 / 1023.0;
    
    // Calcular la corriente (en amperios) para esta muestra
    float corrienteMuestra = (voltajeCorriente - offsetCorriente) / sensibilidadCorriente;

    // Sumar esta corriente a la suma total
    sumaCorriente += corrienteMuestra;

    // Pequeña pausa para evitar lecturas demasiado rápidas
    delay(1);
  }

  // Calcular el promedio de las corrientes medidas
  float corrientePromedio = sumaCorriente / numMuestras;

  // Preparar el mensaje CAN para enviar el valor de la corriente
  canMsgCorriente.can_id = 0x106; // ID para el mensaje CAN de la corriente
  canMsgCorriente.can_dlc = 4;    // Tamaño de los datos (4 bytes para el valor de corriente)

  // Convertir la corriente a formato de 4 bytes (float) para enviar por CAN
  memcpy(canMsgCorriente.data, &corrientePromedio, sizeof(corrientePromedio));

  // Enviar el mensaje CAN de la corriente
  mcp2515.sendMessage(&canMsgCorriente);
  Serial.println("Enviado lectura de corriente por CAN");

  // ===== Imprimir en el monitor serie =====
  Serial.print("Voltaje medido: ");
  Serial.print(voltajeReal);
  Serial.println(" V");

  Serial.print("Corriente promedio medida: ");
  Serial.print(corrientePromedio);
  Serial.println(" A");

  // Esperar un segundo antes de realizar la siguiente lectura
  delay(1000);
}
