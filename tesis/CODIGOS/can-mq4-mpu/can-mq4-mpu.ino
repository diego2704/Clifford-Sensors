#include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>

// Variables para el giroscopio MPU-6050
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

// Variables para el sensor MQ-4
int lectura_sensor;
int valor_limite_min = 300;  // Valor mínimo para enviar
int valor_limite_max = 1000; // Valor máximo para enviar

// MCP2515
MCP2515 mcp2515(10); // Pin CS del MCP2515

void setup() {
  Serial.begin(115200);

  // Inicialización del MCP2515
  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ); // Configurar la velocidad del bus CAN (1 Mbps) y cristal de 8 MHz
  mcp2515.setNormalMode(); // Modo normal de operación
  Serial.println("MCP2515 Initialized");

  // Configuración del MPU-6050
  Wire.setClock(400000); // Configurar I2C a 400kHz
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Calibración del giroscopio
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }
  RateCalibrationRoll /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw /= 2000;
}

void loop() {
  // Leer datos del giroscopio
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Integración para obtener la orientación (Roll, Pitch, Yaw)
  Roll += RateRoll * 0.05;  // Multiplicado por el intervalo de tiempo (50ms)
  Pitch += RatePitch * 0.05;
  Yaw += RateYaw * 0.05;

  // Enviar los datos del MPU-6050 a través del bus CAN
  sendMPUDataOverCAN(Roll, Pitch, Yaw, RateRoll, RatePitch, RateYaw);

  // Leer el valor del sensor MQ-4
  lectura_sensor = analogRead(A0);
  
  // Solo enviar el valor si está dentro del rango especificado
  if (lectura_sensor >= valor_limite_min && lectura_sensor <= valor_limite_max) {
    sendMQ4DataOverCAN(lectura_sensor);
  }

  delay(50); // Esperar antes de la siguiente lectura
}

// Leer señales del giroscopio
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Empezar a leer desde el registro del giroscopio
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}

// Función para enviar los datos del MPU-6050 por CAN
void sendMPUDataOverCAN(float roll, float pitch, float yaw, float rateRoll, float ratePitch, float rateYaw) {
  struct can_frame canMsg;
  canMsg.can_dlc = 8; // Máximo de 8 bytes por trama

  // Enviar Roll, Pitch y Yaw
  canMsg.can_id = 0x100; // ID para la primera trama
  memcpy(&canMsg.data[0], &roll, sizeof(float));
  memcpy(&canMsg.data[4], &pitch, sizeof(float));
  mcp2515.sendMessage(&canMsg);
  delay(500);

  canMsg.can_id = 0x101; // ID para la segunda trama
  memcpy(&canMsg.data[0], &yaw, sizeof(float));
  mcp2515.sendMessage(&canMsg);
  delay(500);

  // Enviar RateRoll, RatePitch y RateYaw
  canMsg.can_id = 0x102; // ID para la tercera trama
  memcpy(&canMsg.data[0], &rateRoll, sizeof(float));
  memcpy(&canMsg.data[4], &ratePitch, sizeof(float));
  mcp2515.sendMessage(&canMsg);
  delay(500);

  canMsg.can_id = 0x103; // ID para la cuarta trama
  memcpy(&canMsg.data[0], &rateYaw, sizeof(float));
  mcp2515.sendMessage(&canMsg);
  delay(500);

  // Confirmar en el monitor serial que las tramas han sido enviadas
  Serial.println("Datos MPU enviados por CAN:");
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(" Pitch: ");
  Serial.print(pitch);
  Serial.print(" Yaw: ");
  Serial.println(yaw);
  delay(500);

  Serial.print("RateRoll: ");
  Serial.print(rateRoll);
  Serial.print(" RatePitch: ");
  Serial.print(ratePitch);
  Serial.print(" RateYaw: ");
  Serial.println(rateYaw);
  delay(500);
}

// Función para enviar los datos del sensor MQ-4 por CAN
void sendMQ4DataOverCAN(int lectura_sensor) {
  struct can_frame canMsg;

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
  delay(500);
}
