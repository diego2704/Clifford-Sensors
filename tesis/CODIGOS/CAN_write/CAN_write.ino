#include <SPI.h>
#include <mcp2515.h>
struct can_frame canMsg1;
MCP2515 mcp2515(10);
int pul1 = 8;
int pul2 = 9;
int vpul1 = 0; //valor inicial pulsadores
int vpul2 = 0;

void setup() {
  pinMode(pul1, INPUT);
  pinMode(pul2, INPUT);
  canMsg1.can_id  = 0x0F6; // Identificador del nodo transmisor
  canMsg1.can_dlc = 8; //Data Length Code - código de longitud de datos, 8 Bytes
  canMsg1.data[0] = 0x00;
  canMsg1.data[1] = 0x00;
  canMsg1.data[2] = 0x00;
  canMsg1.data[3] = 0x00;
  canMsg1.data[4] = 0x00;
  canMsg1.data[5] = 0x00;
  canMsg1.data[6] = 0x00;
  canMsg1.data[7] = 0x00;
  while (!Serial);
  Serial.begin(115200);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
}

void loop() {
  vpul1 = digitalRead(pul1);
  delay(50);
  vpul2 = digitalRead(pul2);
  delay(50);
  canMsg1.data[0] = vpul1;
  canMsg1.data[1] = vpul2;
  mcp2515.sendMessage(&canMsg1);
  Serial.println("Mensaje enviado");
  Serial.println("Mensaje enviado");
  Serial.print(canMsg1.data[0]);
  Serial.print(",");
  Serial.print(canMsg1.data[1]);
  Serial.print(",");
  Serial.print(canMsg1.data[2]);
  Serial.print(",");
  Serial.print(canMsg1.data[3]);
  Serial.print(",");
  Serial.print(canMsg1.data[4]);
  Serial.print(",");
  Serial.print(canMsg1.data[5]);
  Serial.print(",");
  Serial.print(canMsg1.data[6]);
  Serial.print(",");
  Serial.println(canMsg1.data[7]);
  
  delay(100);
}
