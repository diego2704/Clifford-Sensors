#include <Wire.h>

#define MPU9250_ADDRESS 0x68

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

void gyro_signals(void) {
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission(); 
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission(); 
  Wire.requestFrom(MPU9250_ADDRESS, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
}

void calibrateGyro() {
  Serial.println("Iniciando calibración del giroscopio. Por favor, mantén el sensor en reposo.");

  const int numReadings = 1000; // Número de lecturas para la calibración
  int32_t gyroX = 0, gyroY = 0, gyroZ = 0;

  for (int i = 0; i < numReadings; ++i) {
    gyro_signals();
    gyroX += RateRoll;
    gyroY += RatePitch;
    gyroZ += RateYaw;
    delay(5); // Intervalo de muestreo
  }

  RateCalibrationRoll = gyroX / numReadings;
  RateCalibrationPitch = gyroY / numReadings;
  RateCalibrationYaw = gyroZ / numReadings;

  Serial.println("Calibración del giroscopio completada.");
  Serial.print("Offsets: Roll = "); Serial.print(RateCalibrationRoll);
  Serial.print(", Pitch = "); Serial.print(RateCalibrationPitch);
  Serial.print(", Yaw = "); Serial.println(RateCalibrationYaw);
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Configurar el giroscopio
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // Despierta el MPU-9250
  Wire.endTransmission();

  delay(100);

  // Configurar el acelerómetro y el giroscopio
  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1A); // Configurar el registro de configuración del giroscopio
  Wire.write(0x05); // Set Gyro Full Scale Range a +/- 250 DPS
  Wire.endTransmission();

  Wire.beginTransmission(MPU9250_ADDRESS);
  Wire.write(0x1B); // Configurar el registro de configuración del acelerómetro
  Wire.write(0x08); // Set Accel Full Scale Range a +/- 4g
  Wire.endTransmission();

  // Calibrar el giroscopio
  calibrateGyro();
}

void loop() {
  gyro_signals();
  RateRoll -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw -= RateCalibrationYaw;

  // Integración para obtener la orientación
  Roll += RateRoll * 0.05;  // Multiplicado por el intervalo de tiempo (50 ms)
  Pitch += RatePitch * 0.05;
  Yaw += RateYaw * 0.05;

  Serial.print("Roll [°]= ");
  Serial.print(Roll);
  Serial.print(" Pitch [°]= ");
  Serial.print(Pitch);
  Serial.print(" Yaw [°]= ");
  Serial.println(Yaw);

  Serial.print("RateRoll [°]= ");
  Serial.print(RateRoll);
  Serial.print(" RatePitch [°]= ");
  Serial.print(RatePitch);
  Serial.print(" RateYaw [°]= ");
  Serial.println(RateYaw);

  delay(39);
}
