#include <Arduino.h>
#include <SPI.h>
#include "mcp_can.h"

// ---- Pins (your wiring) ----
static const int PIN_SCK  = 19;
static const int PIN_MOSI = 18;
static const int PIN_MISO = 21;
static const int PIN_CS   = 33;
static const int PIN_INT  = 32;

// ---- CAN (adjust as needed) ----
static const uint8_t  MCP_CLK   = MCP_16MHZ;     // 8MHzなら MCP_8MHZ に
static const uint32_t CAN_BAUD  = CAN_500KBPS;   // 250k/500k/1M など

// ---- Protocol (temporary example) ----
static const uint32_t ID_CMD  = 0x200; // ESP32 -> ATmega
static const uint32_t ID_RESP = 0x201; // ATmega -> ESP32

volatile bool canIntFlag = false;
MCP_CAN CAN(PIN_CS);

void IRAM_ATTR onCanInt() { canIntFlag = true; }

bool canBeginOnce() {
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  byte rc = CAN.begin(MCP_ANY, CAN_BAUD, MCP_CLK);
  if (rc != CAN_OK) { Serial.printf("CAN.begin fail rc=%d\n", rc); return false; }

  // ★まずはループバックで自己診断
  CAN.setMode(MCP_LOOPBACK);

  pinMode(PIN_INT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_INT), onCanInt, FALLING);

  // 受信フィルタを緩める（自分が投げる 0x200 も拾えるように）
  CAN.init_Mask(0, 0, 0x000);  // マスク0＝全通過
  CAN.init_Mask(1, 0, 0x000);
  CAN.init_Filt(0, 0, 0x000);
  CAN.init_Filt(1, 0, 0x000);
  CAN.init_Filt(2, 0, 0x000);
  CAN.init_Filt(3, 0, 0x000);
  CAN.init_Filt(4, 0, 0x000);
  CAN.init_Filt(5, 0, 0x000);

  return true;
}


bool sendMeasureTrigger() {
  byte buf[8] = {0};
  buf[0] = 0x01; // one-shot measure
  byte rc = CAN.sendMsgBuf(ID_CMD, 0 /*std id*/, 8, buf);
  if (rc != CAN_OK) {
    Serial.print("sendMsgBuf fail rc="); Serial.println(rc);
    return false;
  }
  return true;
}

bool readResponseOnce(uint16_t &distance_mm, uint8_t &status, uint8_t &strength) {
  if (!canIntFlag) return false;

  noInterrupts(); canIntFlag = false; interrupts();

  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned char len = 0;
    unsigned char rxBuf[8] = {0};
    unsigned long  rxId  = 0;
    byte ext = 0;  // 0=standard, 1=extended

    // ★ v1.5.1 互換API（4引数版）
    if (CAN.readMsgBuf(&rxId, &ext, &len, rxBuf) != CAN_OK) continue;

    // 標準IDのみ扱う（拡張IDはスキップする場合）
    if (ext) continue;

    if (rxId == ID_RESP && len >= 3) {
      status      = rxBuf[0];
      distance_mm = (uint16_t(rxBuf[1]) << 8) | uint16_t(rxBuf[2]); // BE想定
      strength    = (len >= 4) ? rxBuf[3] : 0;
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nESP32 -> MCP25625 (mcp_can v1.5.1) init...");
  if (!canBeginOnce()) {
    Serial.println("CAN init failed. Check wiring/clock/baud/terminations.");
    while (1) delay(1000);
  }
  Serial.println("CAN init OK.");
}

void loop() {
  // trigger
  if (!sendMeasureTrigger()) {
    Serial.println("Trigger send failed");
  }

  // wait response (<= 50ms)
  uint32_t t0 = millis();
  uint16_t dist = 0;
  uint8_t status = 0, strength = 0;
  bool got = false;

  while (millis() - t0 < 50) {
    if (readResponseOnce(dist, status, strength)) { got = true; break; }
    delay(1);
  }

  if (got) {
    if (status == 0) {
      Serial.print("URM37 distance = ");
      Serial.print(dist);
      Serial.print(" mm  strength=");
      Serial.println(strength);
    } else {
      Serial.print("URM37 error, status="); Serial.println(status);
    }
  } else {
    Serial.println("No response (timeout).");
  }

  delay(200);
}
