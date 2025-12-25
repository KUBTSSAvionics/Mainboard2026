/*
 * ESP32 + TEL0157 (Gravity GNSS)  UART接続
 *  - TEL0157 TX -> ESP32 IO16
 *  - TEL0157 RX -> ESP32 IO27
 *  - ボーレート 9600/8N1
 * ライブラリ: DFRobot_GNSS v1.0.0
 *
 * v1.0.0要点:
 *  - DFRobot_GNSS_UART のコンストラクタに(&Serial2, baud, rxPin, txPin)が必須
 *  - begin() は引数なし
 *  - 位置取得APIは getLat()/getLon()/getAlt()/getUTC()/getDate()/getNumSatUsed() 等（構造体返し）
 */

#include <DFRobot_GNSS.h>

static const uint8_t GNSS_RX_PIN = 16;     // ESP32側のRX（受ける） ← TEL0157 TX
static const uint8_t GNSS_TX_PIN = 27;     // ESP32側のTX（送る） → TEL0157 RX
static const uint16_t GNSS_BAUD  = 9600;

// v1.0.0 形式のコンストラクタ
DFRobot_GNSS_UART gnss(&Serial2, GNSS_BAUD, GNSS_RX_PIN, GNSS_TX_PIN);

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("TEL0157 + DFRobot_GNSS v1.0.0 (UART 9600/8N1) start");

  // GNSS初期化（引数なし）
  while (!gnss.begin()) {
    Serial.println("NO Devices ! (GNSS not responding)");
    delay(1000);
  }
  Serial.println("GNSS begin OK");
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last = millis();

    // ---- “翻訳済み” API（v1.0.0想定） ----
    sTim_t utc  = gnss.getUTC();     // 時:分:秒 (UTC)
    sTim_t date = gnss.getDate();    // 年/月/日

    sLonLat_t lat = gnss.getLat();   // lat.latitudeDegree, lat.latDirection('N'/'S')
    sLonLat_t lon = gnss.getLon();   // lon.lonitudeDegree, lon.lonDirection('E'/'W')

    double alt    = gnss.getAlt();         // 高度[m]
    uint8_t used  = gnss.getNumSatUsed();  // 使用衛星数
    double sog    = gnss.getSog();         // 対地速度（単位は実装依存）
    double cog    = gnss.getCog();         // 針路
    char latHEM = (lat.latitudeDegree  >= 0) ? 'N' : 'S';   // 数値の符号で決める
    char lonHEM = (lon.lonitudeDegree >= 0) ? 'E' : 'W';

    Serial.printf("%04d/%02d/%02d %02d:%02d:%02d  sats=%u  alt=%.1f  sog=%.2f  cog=%.2f\n",
                  date.year, date.month, date.date,
                  utc.hour, utc.minute, utc.second,
                  used, alt, sog, cog);

    Serial.printf("lat=%.6f %c  lon=%.6f %c\n",
                  lat.latitudeDegree, latHEM,
                  lon.lonitudeDegree, lonHEM);
  }
}
