#include <SPI.h>
#include <mcp2515.h>
struct can_frame canMsg;
MCP2515 mcp2515(10);

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();
}

void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    for (int i = 0; i < canMsg.can_dlc; i++) {
      Serial.print(canMsg.data[i]);
      if (i < canMsg.can_dlc - 1) {
        Serial.print(",");
      }
    }
    Serial.println();
  }
  delay(100);
}
