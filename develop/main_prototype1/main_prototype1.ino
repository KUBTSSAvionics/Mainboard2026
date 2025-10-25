#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

// ======== Pin Config (あなたの実配線) ========
static const int PIN_I2C_SDA = 15;
static const int PIN_I2C_SCL = 14;

static const int PIN_SD_CS   = 17;
static const int PIN_SD_SCK  = 19;
static const int PIN_SD_MISO = 21;
static const int PIN_SD_MOSI = 18;
static const int PIN_SD_DET  = 22;
// =============================================

// ======== Logging Config ========
const float   LOG_HZ         = 50.0f;        // ログ周波数
const uint32_t LOG_MAX_BYTES = 1 * 1024 * 1024; // 1MBでローテ
const uint32_t FLUSH_EVERY_N = 25;           // 25行ごとにflush
// =================================

// ==== SD (SdFs: FAT/EXFAT両対応) ====
SPIClass spi(VSPI);
SdFs sd;
FsFile logFile;
// ===================================

// ==== BNO055 ====
Adafruit_BNO055 bno(55, 0x28); // 0x28(既定)。ADRをHIGHにしているなら0x29に変更
// =================

// ==== RX8900 (シンプルI2C) ====
static const uint8_t RX8900_ADDR = 0x32; // 7-bit
uint8_t bcd2i(uint8_t b){ return (b>>4)*10 + (b&0x0F); }
uint8_t i2bcd(uint8_t i){ return ((i/10)<<4) | (i%10); }

uint8_t rx8900_read(uint8_t reg){
  Wire.beginTransmission(RX8900_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((int)RX8900_ADDR, 1);
  return Wire.read();
}
void rx8900_write(uint8_t reg, uint8_t val){
  Wire.beginTransmission(RX8900_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// FLAG(0x0E)のVLF/VDETを0クリア。必要なら時刻セット（ビルド時刻）
void rx8900_init_if_needed(){
  uint8_t flg = rx8900_read(0x0E);
  bool vlf  = flg & 0b10;
  bool vdet = flg & 0b01;

  // クリア
  rx8900_write(0x0E, flg & ~0b11);

  if (vlf || vdet) {
    // 初回/電圧低下 → ビルド時刻で暫定設定（本番はNTP等に差し替えてOK）
    const char* months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char mmm[4]; int dd, yyyy, hh, mm, ss;
    sscanf(__DATE__, "%3s %d %d", mmm, &dd, &yyyy);
    sscanf(__TIME__, "%d:%d:%d", &hh, &mm, &ss);
    int M = (strstr(months, mmm) - months)/3 + 1;
    int Y = yyyy - 2000;

    rx8900_write(0x00, i2bcd(ss));
    rx8900_write(0x01, i2bcd(mm));
    rx8900_write(0x02, i2bcd(hh));   // 24h
    rx8900_write(0x03, i2bcd(0));    // weekday未使用
    rx8900_write(0x04, i2bcd(dd));
    rx8900_write(0x05, i2bcd(M));
    rx8900_write(0x06, i2bcd(Y));
  }
}

// "YYYY-MM-DD HH:MM:SS" + ".mmm" を返す（ミリ秒はESP32のmillisで付加）
String now_string_ms(){
  // 秒～年: 0x00..0x06
  uint8_t s = bcd2i(rx8900_read(0x00));
  uint8_t m = bcd2i(rx8900_read(0x01));
  uint8_t h = bcd2i(rx8900_read(0x02));
  uint8_t D = bcd2i(rx8900_read(0x04));
  uint8_t M = bcd2i(rx8900_read(0x05));
  uint8_t Y = bcd2i(rx8900_read(0x06)); // 2000+Y

  uint16_t year = 2000 + Y;
  char buf[32];
  snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u",
           year, M, D, h, m, s);

  // ミリ秒（絶対時刻同期が要るならRX8900の1Hz割り込みで同期をとる運用に）
  static uint32_t t0 = millis();
  uint32_t ms = (millis() - t0) % 1000;

  char out[40];
  snprintf(out, sizeof(out), "%s.%03lu", buf, (unsigned long)ms);
  return String(out);
}

String date_folder(){
  // 先頭10文字 "YYYY-MM-DD"
  return now_string_ms().substring(0, 10);
}

bool ensureDir(const String& path){
  if (sd.exists(path.c_str())) return true;
  return sd.mkdir(path.c_str());
}

bool open_log_csv(){
  String dir = "/" + date_folder();
  if (!ensureDir(dir)) return false;

  // 連番 log_001.csv, log_002.csv...
  for (int i=1; i<=999; ++i){
    char name[64];
    snprintf(name, sizeof(name), "%s/log_%03d.csv", dir.c_str(), i);
    if (!sd.exists(name)) {
      if (!logFile.open(name, O_WRONLY | O_CREAT | O_TRUNC)) return false;
      // ヘッダ（例：時刻, センサ値）
      logFile.println("time,"
                      "ax,ay,az,"
                      "gx,gy,gz,"
                      "mx,my,mz,"
                      "roll,pitch,yaw,"
                      "qw,qx,qy,qz,"
                      "tempC");
      return true;
    } else {
      // 既存ファイルが上限未満なら追記
      FsFile f;
      if (f.open(name, O_RDWR)) {
        uint64_t sz = f.fileSize();
        f.close();
        if (sz < LOG_MAX_BYTES) {
          if (!logFile.open(name, O_WRONLY | O_APPEND)) return false;
          return true;
        }
      }
    }
  }
  return false;
}

bool sd_mount_with_speed(uint8_t mhz){
  SdSpiConfig cfg(PIN_SD_CS, SHARED_SPI, SD_SCK_MHZ(mhz), &spi);
  return sd.begin(cfg);
}

void safe_flush_periodic(uint32_t &lineCounter){
  if (++lineCounter % FLUSH_EVERY_N == 0) logFile.flush();
}

// ====== SETUP ======
void setup(){
  Serial.begin(115200);
  delay(200);

  // I2C
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000); // BNO055は400kHz対応

  // SD Detect
  if (PIN_SD_DET >= 0) {
    pinMode(PIN_SD_DET, INPUT_PULLUP); // 多くは「挿入=LOW」
    if (digitalRead(PIN_SD_DET) != LOW) {
      Serial.println("No SD card inserted");
      while(1) delay(1000);
    }
  }

  // SPI for SD
  spi.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);

  // まず 1MHzで確実にマウント
  if (!sd_mount_with_speed(1)) {
    Serial.println("SD mount failed @1MHz");
    while(1) delay(1000);
  }
  Serial.printf("SD: %s\n",
    sd.fatType()==FAT_TYPE_EXFAT ? "exFAT" :
    sd.fatType()==32 ? "FAT32" : "FAT(other)");

  // 可能なら速度アップ（失敗しても低速で継続）
  if (sd_mount_with_speed(20)) Serial.println("SD re-mounted @20MHz");
  else                         Serial.println("SD stay @1MHz");

  // RX8900 初期化（必要時のみ）
  rx8900_init_if_needed();

  // BNO055 初期化
  if (!bno.begin(OPERATION_MODE_NDOF)) { // NDOFで姿勢解算まで有効
    Serial.println("BNO055 not found");
    while(1) delay(1000);
  }
  // 外付け水晶が無いならfalseのまま。あるならtrueに変更。
  bno.setExtCrystalUse(false);

  // ログファイルを開く
  if (!open_log_csv()){
    Serial.println("open log csv failed");
    while(1) delay(1000);
  }
  Serial.println("Logging start");
}

// ====== LOOP ======
void loop(){
  static uint32_t prevMicros = 0;
  static uint32_t lineCounter = 0;
  const uint32_t interval_us = (uint32_t)(1e6f / LOG_HZ);

  uint32_t now = micros();
  if (now - prevMicros < interval_us) return;
  prevMicros += interval_us;

  // ---- 取得 ----
  sensors_event_t ori, acc, gyro, mag, temp;
  bno.getEvent(&ori, Adafruit_BNO055::VECTOR_EULER); // deg
  bno.getEvent(&acc, Adafruit_BNO055::VECTOR_ACCELEROMETER); // m/s^2
  bno.getEvent(&gyro, Adafruit_BNO055::VECTOR_GYROSCOPE);     // rad/s
  bno.getEvent(&mag,  Adafruit_BNO055::VECTOR_MAGNETOMETER);  // uT
  bno.getEvent(&temp); // 温度
  float tempC = bno.getTemp();

  imu::Quaternion q = bno.getQuat(); // 単位なし（正規化）

  // ---- ローテ条件：サイズ or 日付変化 ----
  if (!logFile || logFile.fileSize() >= LOG_MAX_BYTES) {
    if (logFile) logFile.close();
    open_log_csv();
  } else {
    // 日付フォルダが変わったらローテ（00:00跨ぎ）
    static String curFolder = date_folder();
    String df = date_folder();
    if (df != curFolder) {
      curFolder = df;
      if (logFile) logFile.close();
      open_log_csv();
    }
  }

  // ---- 書き込み ----
  String tstr = now_string_ms();
  logFile.printf("%s,"                                   // time
                 "%.6f,%.6f,%.6f,"                       // ax ay az (m/s^2)
                 "%.6f,%.6f,%.6f,"                       // gx gy gz (rad/s)
                 "%.6f,%.6f,%.6f,"                       // mx my mz (uT)
                 "%.6f,%.6f,%.6f,"                       // roll pitch yaw (deg)
                 "%.6f,%.6f,%.6f,%.6f,"                  // qw qx qy qz
                 "%.2f\n",                                // tempC
                 tstr.c_str(),
                 acc.acceleration.x, acc.acceleration.y, acc.acceleration.z,
                 gyro.gyro.x,        gyro.gyro.y,        gyro.gyro.z,
                 mag.magnetic.x,     mag.magnetic.y,     mag.magnetic.z,
                 ori.orientation.x,  ori.orientation.y,  ori.orientation.z,
                 q.w(), q.x(), q.y(), q.z(),
                 tempC);

  safe_flush_periodic(lineCounter);
}
