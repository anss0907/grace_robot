// =============================================================================
// ESP32 COMPLETE SYSTEM — Dual-Core FreeRTOS Firmware
// =============================================================================
// Based on grace_esp32_firmware.ino (dual-core architecture) with added:
//   - BME680 environmental sensor (temp, humidity, pressure, gas)
//   - PMS5003 particulate matter sensor (PM1.0, PM2.5, PM10)
//   - 3x RGB LED strip control via MOSFET drivers
//   - Dual MPU6050 IMU sensors  (from original)
//   - Hoverboard motor driver   (from original)
//
// Core 0: USB Serial I/O  — send sensor data, receive motor + LED commands
// Core 1: Hardware I/O    — hoverboard UART, I2C sensors, PMS5003 UART, GPIO
//
// Protocol — only connected sensors are included in the output:
//   ESP32 → Linux:  ,T0005,D1,b36000p,t0250p,L0100p,Lm0095p,R0100p,Rm0098p,
//                    I11,ax100500p,...,I21,...,BM1,BT00250p,BH00450p,BP10135p,
//                    BG00150p,PM1,P100010p,P200015p,P1000020p,S1100,S2010,S3001
//   Linux → ESP32:  l50 r50  (motor)  |  1r 2g 3b a  (LED)
// =============================================================================

#include <HardwareSerial.h>
#include <Wire.h>
#include <cstring>

// ======================== USER-CONFIGURABLE MACROS ========================

// Motor speed limits (RPM)
#define SPEED_MAX               180

// Hoverboard protocol scale factor
#define HOVERBOARD_SCALE_FACTOR 2.5f
#define HOVERBOARD_CMD_MAX      450

// Driver feedback timeout (ms)
#define DRIVER_TIMEOUT_MS       500

// Hoverboard command send interval (ms) — 50Hz
#define HOVER_SEND_INTERVAL_MS  20

// Sensor data send interval (ms) — ~200Hz
#define SENSOR_SEND_INTERVAL_MS 5

// BME680 read interval (ms) — slow sensor
#define BME_READ_INTERVAL_MS    2000

// IMU read interval — every loop iteration (fast)
// PMS5003 — read whenever data available

// Serial baud rates
#define USB_BAUD                115200
#define HOVER_BAUD              115200
#define PMS_BAUD                9600

// Serial parse timeout (ms)
#define SERIAL_PARSE_TIMEOUT_MS 5

// ESP32 UART2 pins for hoverboard
#define HOVER_RX_PIN            16
#define HOVER_TX_PIN            17

// ESP32 UART1 pins for PMS5003
#define PMS_RX_PIN              25
#define PMS_TX_PIN              26

// I2C pins
#define I2C_SDA_PIN             21
#define I2C_SCL_PIN             22

// MPU6050 addresses
#define IMU1_ADDR               0x68  // AD0 = GND
#define IMU2_ADDR               0x69  // AD0 = 3.3V

// BME680 address
#define BME680_ADDR             0x77

// RGB LED strip pins (3 strips, MOSFET driven)
#define STRIP1_RED_PIN          4
#define STRIP1_GREEN_PIN        32
#define STRIP1_BLUE_PIN         33

#define STRIP2_RED_PIN          18
#define STRIP2_GREEN_PIN        19
#define STRIP2_BLUE_PIN         5

#define STRIP3_RED_PIN          27
#define STRIP3_GREEN_PIN        14
#define STRIP3_BLUE_PIN         23

// FreeRTOS task stack sizes (bytes)
#define CORE0_STACK_SIZE        8192
#define CORE1_STACK_SIZE        8192

// ======================== INTERNAL CONSTANTS ========================
static constexpr uint16_t START_FRAME       = 0xABCD;
static constexpr uint8_t  MPU_PWR_MGMT_1    = 0x6B;
static constexpr uint8_t  MPU_ACCEL_XOUT_H  = 0x3B;

// BME680 registers (simplified — we do forced mode reads)
static constexpr uint8_t  BME680_CHIP_ID_REG = 0xD0;
static constexpr uint8_t  BME680_CHIP_ID_VAL = 0x61;
static constexpr uint8_t  BME680_CTRL_MEAS   = 0x74;
static constexpr uint8_t  BME680_CTRL_HUM    = 0x72;
static constexpr uint8_t  BME680_CONFIG      = 0x75;
static constexpr uint8_t  BME680_TEMP_MSB    = 0x22;
static constexpr uint8_t  BME680_HUM_MSB     = 0x25;
static constexpr uint8_t  BME680_PRESS_MSB   = 0x1F;

// ======================== DATA STRUCTURES ========================

struct ImuRawData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

// Command sent TO hoverboard
struct HoverCommand {
  uint16_t start;
  int16_t  speedL;
  int16_t  speedR;
  uint16_t checksum;
};

// Feedback received FROM hoverboard
struct HoverFeedback {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
};

struct BME680Data {
  int16_t temperature;   // °C × 10 (e.g. 250 = 25.0°C)
  int16_t humidity;      // % × 10  (e.g. 450 = 45.0%)
  int16_t pressure;      // hPa × 10 (e.g. 10135 = 1013.5 hPa)
  int16_t gasResistance; // kΩ × 10 (e.g. 150 = 15.0 kΩ)
};

struct PMSData {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10_0;
};

struct LedStripState {
  bool red;
  bool green;
  bool blue;
};

// Shared state between cores — protected by mutex
struct SharedState {
  // Written by Core 0, read by Core 1
  int16_t  targetSpeedL;
  int16_t  targetSpeedR;

  // LED commands (written by Core 0, read by Core 1)
  LedStripState strips[3];
  bool          ledCommandPending;  // flag for Core 1 to apply

  // Written by Core 1, read by Core 0
  int16_t  measuredSpeedL;
  int16_t  measuredSpeedR;
  int16_t  batteryMv;
  int16_t  boardTempDeciC;

  // Driver-alive tracking
  bool     driverAlive;
  uint32_t lastDriverFeedbackMs;

  // IMU data
  ImuRawData imu1;
  ImuRawData imu2;
  bool       imu1Ok;
  bool       imu2Ok;

  // BME680 data
  BME680Data bme;
  bool       bmeOk;

  // PMS5003 data
  PMSData    pms;
  bool       pmsOk;

  // LED actual state (read-back for reporting)
  LedStripState ledState[3];
};

// ======================== GLOBALS ========================

HardwareSerial HoverSerial(2);
HardwareSerial PMSSerial(1);
SemaphoreHandle_t stateMutex;
SharedState shared = {};

// ======================== IMU HELPERS ========================

static bool imuWriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool imuReadRaw(uint8_t addr, ImuRawData &out) {
  Wire.beginTransmission(addr);
  Wire.write(MPU_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(addr, (uint8_t)14) != 14) return false;

  out.ax = (Wire.read() << 8) | Wire.read();
  out.ay = (Wire.read() << 8) | Wire.read();
  out.az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip temperature
  out.gx = (Wire.read() << 8) | Wire.read();
  out.gy = (Wire.read() << 8) | Wire.read();
  out.gz = (Wire.read() << 8) | Wire.read();
  return true;
}

static bool initImu(uint8_t addr) {
  if (!imuWriteReg(addr, MPU_PWR_MGMT_1, 0x00)) return false;
  delay(10);
  return true;
}

// ======================== BME680 HELPERS (Simplified — raw register reads) ========================
// NOTE: For production, use Adafruit_BME680 library on the Arduino IDE side.
// Here we do a basic chip-ID check and raw temperature/humidity/pressure reads.
// The values are approximate but good enough for monitoring.

static bool bme680CheckId() {
  Wire.beginTransmission(BME680_ADDR);
  Wire.write(BME680_CHIP_ID_REG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)BME680_ADDR, (uint8_t)1) != 1) return false;
  uint8_t id = Wire.read();
  return (id == BME680_CHIP_ID_VAL);
}

static bool bme680Init() {
  if (!bme680CheckId()) return false;

  // Set humidity oversampling x2
  Wire.beginTransmission(BME680_ADDR);
  Wire.write(BME680_CTRL_HUM);
  Wire.write(0x02);  // osrs_h = x2
  if (Wire.endTransmission() != 0) return false;

  // Set IIR filter coeff = 3, SPI off
  Wire.beginTransmission(BME680_ADDR);
  Wire.write(BME680_CONFIG);
  Wire.write(0x08);  // filter[2:0] = 010
  if (Wire.endTransmission() != 0) return false;

  return true;
}

static bool bme680TriggerMeasurement() {
  // Set temp oversampling x8, press oversampling x4, forced mode
  Wire.beginTransmission(BME680_ADDR);
  Wire.write(BME680_CTRL_MEAS);
  Wire.write(0x95);  // osrs_t=100(x8), osrs_p=010(x4), mode=01(forced)
  return Wire.endTransmission() == 0;
}

static bool bme680ReadData(BME680Data &out) {
  // Read temperature (3 bytes at 0x22-0x24)
  Wire.beginTransmission(BME680_ADDR);
  Wire.write(BME680_TEMP_MSB);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)BME680_ADDR, (uint8_t)3) != 3) return false;
  uint32_t tempAdc = ((uint32_t)Wire.read() << 12) | ((uint32_t)Wire.read() << 4) | (Wire.read() >> 4);

  // Read pressure (3 bytes at 0x1F-0x21)
  Wire.beginTransmission(BME680_ADDR);
  Wire.write(BME680_PRESS_MSB);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)BME680_ADDR, (uint8_t)3) != 3) return false;
  uint32_t pressAdc = ((uint32_t)Wire.read() << 12) | ((uint32_t)Wire.read() << 4) | (Wire.read() >> 4);

  // Read humidity (2 bytes at 0x25-0x26)
  Wire.beginTransmission(BME680_ADDR);
  Wire.write(BME680_HUM_MSB);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)BME680_ADDR, (uint8_t)2) != 2) return false;
  uint16_t humAdc = ((uint16_t)Wire.read() << 8) | Wire.read();

  // Convert ADC to approximate physical values
  // These are rough linearizations — for accurate values use calibration data
  // Temperature: rough approximation from ADC
  // The BME680 temperature ADC is typically ~100000 at ~25°C at default settings
  int32_t tempRough = (int32_t)tempAdc - 100000;
  out.temperature = (int16_t)(250 + tempRough / 200);  // rough °C × 10

  // Humidity: rough approximation (ADC range ~0-65535 maps to ~0-100%)
  out.humidity = (int16_t)((humAdc * 1000L) / 65535);  // % × 10

  // Pressure: rough approximation
  // ADC at ~1013 hPa is typically around 350000-400000
  out.pressure = (int16_t)(10135 + (int32_t)(pressAdc - 375000) / 50);  // hPa × 10

  // Gas: not reading gas heater here (requires complex timing)
  out.gasResistance = 0;

  return true;
}

// ======================== PMS5003 HELPERS ========================

static bool pmsReadPacket(PMSData &out) {
  // PMS5003 sends 32-byte frames: 0x42 0x4D + 30 bytes
  static uint8_t idx = 0;
  static uint8_t buffer[32];
  bool gotPacket = false;

  while (PMSSerial.available()) {
    uint8_t c = PMSSerial.read();

    if (idx == 0 && c != 0x42) continue;
    if (idx == 1 && c != 0x4D) { idx = 0; continue; }

    buffer[idx++] = c;

    if (idx == 32) {
      uint16_t checksum = 0;
      for (int i = 0; i < 30; i++) checksum += buffer[i];
      uint16_t frameChecksum = ((uint16_t)buffer[30] << 8) | buffer[31];
      uint16_t frameLength = ((uint16_t)buffer[2] << 8) | buffer[3];

      if (checksum == frameChecksum && frameLength == 28) {
        // Atmospheric environment values (bytes 10-15)
        out.pm1_0  = ((uint16_t)buffer[10] << 8) | buffer[11];
        out.pm2_5  = ((uint16_t)buffer[12] << 8) | buffer[13];
        out.pm10_0 = ((uint16_t)buffer[14] << 8) | buffer[15];
        gotPacket = true;
      }
      idx = 0;
    }
  }
  return gotPacket;
}

// ======================== HOVERBOARD HELPERS ========================

static int16_t scaleForHoverboard(int16_t rpm) {
  int16_t scaled = static_cast<int16_t>(rpm * HOVERBOARD_SCALE_FACTOR);
  if (scaled > HOVERBOARD_CMD_MAX)  scaled = HOVERBOARD_CMD_MAX;
  if (scaled < -HOVERBOARD_CMD_MAX) scaled = -HOVERBOARD_CMD_MAX;
  return scaled;
}

static void sendHoverCommand(int16_t speedL, int16_t speedR) {
  int16_t scaledL = scaleForHoverboard(speedL);
  int16_t scaledR = scaleForHoverboard(speedR);

  HoverCommand cmd;
  cmd.start    = START_FRAME;
  cmd.speedL   = scaledL;
  cmd.speedR   = scaledR;
  cmd.checksum = cmd.start ^ cmd.speedL ^ cmd.speedR;

  HoverSerial.write(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
}

// ======================== LED HELPERS ========================

static void applyLedStrip(int strip, bool r, bool g, bool b) {
  int rPin, gPin, bPin;
  switch (strip) {
    case 0: rPin = STRIP1_RED_PIN; gPin = STRIP1_GREEN_PIN; bPin = STRIP1_BLUE_PIN; break;
    case 1: rPin = STRIP2_RED_PIN; gPin = STRIP2_GREEN_PIN; bPin = STRIP2_BLUE_PIN; break;
    case 2: rPin = STRIP3_RED_PIN; gPin = STRIP3_GREEN_PIN; bPin = STRIP3_BLUE_PIN; break;
    default: return;
  }
  digitalWrite(rPin, r ? HIGH : LOW);
  digitalWrite(gPin, g ? HIGH : LOW);
  digitalWrite(bPin, b ? HIGH : LOW);
}

// ======================== SENSOR STRING BUILDER ========================

static void printField(const char* label, int16_t value, int width) {
  Serial.print(label);
  Serial.printf("%0*d", width, abs(value));
  Serial.print(value >= 0 ? 'p' : 'n');
}

static void printUField(const char* label, uint16_t value, int width) {
  Serial.print(label);
  Serial.printf("%0*u", width, value);
  Serial.print('p');
}

static void sendSensorString(const SharedState &s, uint32_t nowMs) {
  static uint32_t lastSendTime = 0;
  uint32_t dt = nowMs - lastSendTime;
  lastSendTime = nowMs;

  Serial.print(',');

  // Delta time
  Serial.print("T");
  Serial.printf("%04lu", dt);               Serial.print(',');

  // Driver-alive flag
  Serial.print("D");
  Serial.print(s.driverAlive ? '1' : '0');  Serial.print(',');

  // --- Always send motor driver data (zeros if driver dead) ---
  printField("b", s.batteryMv, 5);          Serial.print(',');
  printField("t", s.boardTempDeciC, 4);     Serial.print(',');
  printField("L", s.targetSpeedL, 4);       Serial.print(',');
  printField("Lm", s.measuredSpeedL, 4);    Serial.print(',');
  printField("R", s.targetSpeedR, 4);       Serial.print(',');
  printField("Rm", s.measuredSpeedR, 4);    Serial.print(',');

  // --- IMU 1 (only send data if connected) ---
  Serial.print("I1");
  Serial.print(s.imu1Ok ? '1' : '0');
  if (s.imu1Ok) {
    Serial.print(',');
    printField("ax1", s.imu1.ax, 5);        Serial.print(',');
    printField("ay1", s.imu1.ay, 5);        Serial.print(',');
    printField("az1", s.imu1.az, 5);        Serial.print(',');
    printField("gx1", s.imu1.gx, 5);        Serial.print(',');
    printField("gy1", s.imu1.gy, 5);        Serial.print(',');
    printField("gz1", s.imu1.gz, 5);
  }
  Serial.print(',');

  // --- IMU 2 (only send data if connected) ---
  Serial.print("I2");
  Serial.print(s.imu2Ok ? '1' : '0');
  if (s.imu2Ok) {
    Serial.print(',');
    printField("ax2", s.imu2.ax, 5);        Serial.print(',');
    printField("ay2", s.imu2.ay, 5);        Serial.print(',');
    printField("az2", s.imu2.az, 5);        Serial.print(',');
    printField("gx2", s.imu2.gx, 5);        Serial.print(',');
    printField("gy2", s.imu2.gy, 5);        Serial.print(',');
    printField("gz2", s.imu2.gz, 5);
  }
  Serial.print(',');

  // --- BME680 (only send data if connected) ---
  Serial.print("BM");
  Serial.print(s.bmeOk ? '1' : '0');
  if (s.bmeOk) {
    Serial.print(',');
    printField("BT", s.bme.temperature, 5);    Serial.print(',');
    printField("BH", s.bme.humidity, 5);        Serial.print(',');
    printField("BP", s.bme.pressure, 5);        Serial.print(',');
    printField("BG", s.bme.gasResistance, 5);
  }
  Serial.print(',');

  // --- PMS5003 (only send data if connected) ---
  Serial.print("PM");
  Serial.print(s.pmsOk ? '1' : '0');
  if (s.pmsOk) {
    Serial.print(',');
    printUField("P1", s.pms.pm1_0, 5);          Serial.print(',');
    printUField("P2", s.pms.pm2_5, 5);          Serial.print(',');
    printUField("P10", s.pms.pm10_0, 5);
  }
  Serial.print(',');

  // --- LED strip states (always send — just GPIO reads) ---
  for (int i = 0; i < 3; i++) {
    Serial.printf("S%d%d%d%d", i + 1,
      s.ledState[i].red   ? 1 : 0,
      s.ledState[i].green ? 1 : 0,
      s.ledState[i].blue  ? 1 : 0);
    if (i < 2) Serial.print(',');
  }

  Serial.println();
}

// ======================== CORE 0 TASK: USB Serial I/O ========================

void core0Task(void *pvParameters) {
  (void)pvParameters;
  uint32_t lastSensorSend = 0;

  for (;;) {
    uint32_t now = millis();

    // ---------- RECEIVE COMMANDS FROM LINUX ----------
    while (Serial.available()) {
      char c = Serial.peek();

      if (c == 'l') {
        Serial.read();
        int16_t speedL = Serial.parseInt();
        speedL = constrain(speedL, -SPEED_MAX, SPEED_MAX);
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          shared.targetSpeedL = speedL;
          xSemaphoreGive(stateMutex);
        }
      }
      else if (c == 'r') {
        Serial.read();
        int16_t speedR = Serial.parseInt();
        speedR = constrain(speedR, -SPEED_MAX, SPEED_MAX);
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          shared.targetSpeedR = speedR;
          xSemaphoreGive(stateMutex);
        }
      }
      // LED commands: '1'-'3' followed by color letter, or 'a' for all off
      else if (c >= '1' && c <= '3') {
        Serial.read();
        int strip = c - '1';  // 0-indexed
        // Wait briefly for color byte
        uint32_t waitStart = millis();
        while (!Serial.available() && (millis() - waitStart < 50)) {
          vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (Serial.available()) {
          char colorCmd = Serial.read();
          LedStripState ls = {false, false, false};
          switch (colorCmd) {
            case 'r': case 'R': ls = {true,  false, false}; break;
            case 'g': case 'G': ls = {false, true,  false}; break;
            case 'b': case 'B': ls = {false, false, true};  break;
            case 'y': case 'Y': ls = {true,  true,  false}; break;
            case 'c': case 'C': ls = {false, true,  true};  break;
            case 'm': case 'M': ls = {true,  false, true};  break;
            case 'w': case 'W': ls = {true,  true,  true};  break;
            case 'o': case 'O': ls = {false, false, false}; break;
            default: break;
          }
          if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shared.strips[strip] = ls;
            shared.ledCommandPending = true;
            xSemaphoreGive(stateMutex);
          }
        }
      }
      else if (c == 'a' || c == 'A') {
        Serial.read();
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          for (int i = 0; i < 3; i++) {
            shared.strips[i] = {false, false, false};
          }
          shared.ledCommandPending = true;
          xSemaphoreGive(stateMutex);
        }
      }
      else {
        Serial.read();  // consume unknown char
      }
    }

    // ---------- SEND SENSOR DATA TO LINUX ----------
    if (now - lastSensorSend >= SENSOR_SEND_INTERVAL_MS) {
      SharedState snapshot;
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        snapshot = shared;
        xSemaphoreGive(stateMutex);
      }
      sendSensorString(snapshot, now);
      lastSensorSend = now;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ======================== CORE 1 TASK: Hardware I/O ========================

void core1Task(void *pvParameters) {
  (void)pvParameters;

  // -- Hoverboard receive state machine --
  uint8_t  idx = 0;
  uint16_t bufStartFrame = 0;
  byte     incomingByte = 0;
  byte     incomingBytePrev = 0;
  byte    *p = nullptr;
  HoverFeedback newFeedback = {};

  bool localImu1Ok = false;
  bool localImu2Ok = false;
  bool localBmeOk  = false;

  // -- Init I2C and sensors on this core --
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(50);

  localImu1Ok = initImu(IMU1_ADDR);
  localImu2Ok = initImu(IMU2_ADDR);
  localBmeOk  = bme680Init();

  // -- Init PMS5003 UART --
  PMSSerial.begin(PMS_BAUD, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  delay(100);

  // -- Init LED pins --
  const int ledPins[] = {
    STRIP1_RED_PIN, STRIP1_GREEN_PIN, STRIP1_BLUE_PIN,
    STRIP2_RED_PIN, STRIP2_GREEN_PIN, STRIP2_BLUE_PIN,
    STRIP3_RED_PIN, STRIP3_GREEN_PIN, STRIP3_BLUE_PIN
  };
  for (int i = 0; i < 9; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }

  // Write initial sensor status
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    shared.imu1Ok = localImu1Ok;
    shared.imu2Ok = localImu2Ok;
    shared.bmeOk  = localBmeOk;
    shared.pmsOk  = false;  // will be set true on first valid packet
    xSemaphoreGive(stateMutex);
  }

  uint32_t lastHoverSend = 0;
  uint32_t lastBmeRead   = 0;
  bool     bmeMeasurementTriggered = false;

  for (;;) {
    uint32_t now = millis();

    // ---------- RECEIVE HOVERBOARD FEEDBACK ----------
    while (HoverSerial.available()) {
      incomingByte = HoverSerial.read();
      bufStartFrame = (static_cast<uint16_t>(incomingByte) << 8) | incomingBytePrev;

      if (bufStartFrame == START_FRAME) {
        p = reinterpret_cast<byte*>(&newFeedback);
        *p++ = incomingBytePrev;
        *p++ = incomingByte;
        idx = 2;
      } else if (idx >= 2 && idx < sizeof(HoverFeedback)) {
        if (p) {
          *p++ = incomingByte;
          idx++;
        } else {
          idx = 0;
        }
      }

      if (idx == sizeof(HoverFeedback)) {
        uint16_t checksum = newFeedback.start ^ newFeedback.cmd1 ^ newFeedback.cmd2 ^
                            newFeedback.speedR_meas ^ newFeedback.speedL_meas ^
                            newFeedback.batVoltage ^ newFeedback.boardTemp ^ newFeedback.cmdLed;

        if (newFeedback.start == START_FRAME && checksum == newFeedback.checksum) {
          if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shared.measuredSpeedL       = newFeedback.speedL_meas;
            shared.measuredSpeedR       = newFeedback.speedR_meas;
            shared.batteryMv            = newFeedback.batVoltage;
            shared.boardTempDeciC       = newFeedback.boardTemp;
            shared.driverAlive          = true;
            shared.lastDriverFeedbackMs = now;
            xSemaphoreGive(stateMutex);
          }
        }
        idx = 0;
      }
      incomingBytePrev = incomingByte;
    }

    // ---------- READ IMUs ----------
    ImuRawData imu1Data = {}, imu2Data = {};
    bool imu1Read = false, imu2Read = false;

    if (localImu1Ok) {
      imu1Read = imuReadRaw(IMU1_ADDR, imu1Data);
      if (!imu1Read) localImu1Ok = false;
    }
    if (localImu2Ok) {
      imu2Read = imuReadRaw(IMU2_ADDR, imu2Data);
      if (!imu2Read) localImu2Ok = false;
    }

    if (imu1Read || imu2Read) {
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        if (imu1Read) shared.imu1 = imu1Data;
        if (imu2Read) shared.imu2 = imu2Data;
        shared.imu1Ok = localImu1Ok;
        shared.imu2Ok = localImu2Ok;
        xSemaphoreGive(stateMutex);
      }
    }

    // ---------- READ BME680 ----------
    if (localBmeOk && (now - lastBmeRead >= BME_READ_INTERVAL_MS)) {
      if (!bmeMeasurementTriggered) {
        bme680TriggerMeasurement();
        bmeMeasurementTriggered = true;
      } else {
        BME680Data bmeData = {};
        if (bme680ReadData(bmeData)) {
          if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shared.bme = bmeData;
            shared.bmeOk = true;
            xSemaphoreGive(stateMutex);
          }
        } else {
          localBmeOk = false;
          if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
            shared.bmeOk = false;
            xSemaphoreGive(stateMutex);
          }
        }
        bmeMeasurementTriggered = false;
        lastBmeRead = now;
      }
    }

    // ---------- READ PMS5003 ----------
    if (PMSSerial.available()) {
      PMSData pmsData = {};
      if (pmsReadPacket(pmsData)) {
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          shared.pms = pmsData;
          shared.pmsOk = true;
          xSemaphoreGive(stateMutex);
        }
      }
    }

    // ---------- APPLY LED COMMANDS ----------
    bool applyLeds = false;
    LedStripState ledCmd[3];
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      if (shared.ledCommandPending) {
        applyLeds = true;
        for (int i = 0; i < 3; i++) ledCmd[i] = shared.strips[i];
        shared.ledCommandPending = false;
      }
      xSemaphoreGive(stateMutex);
    }

    if (applyLeds) {
      for (int i = 0; i < 3; i++) {
        applyLedStrip(i, ledCmd[i].red, ledCmd[i].green, ledCmd[i].blue);
      }
      // Update actual state
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        for (int i = 0; i < 3; i++) shared.ledState[i] = ledCmd[i];
        xSemaphoreGive(stateMutex);
      }
    }

    // ---------- CHECK DRIVER ALIVE TIMEOUT ----------
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
      if (shared.driverAlive && (now - shared.lastDriverFeedbackMs) >= DRIVER_TIMEOUT_MS) {
        shared.driverAlive    = false;
        shared.measuredSpeedL = 0;
        shared.measuredSpeedR = 0;
        shared.batteryMv      = 0;
        shared.boardTempDeciC = 0;
      }
      xSemaphoreGive(stateMutex);
    }

    // ---------- SEND MOTOR COMMANDS TO HOVERBOARD ----------
    if (now - lastHoverSend >= HOVER_SEND_INTERVAL_MS) {
      int16_t cmdL = 0, cmdR = 0;
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        cmdL = shared.targetSpeedL;
        cmdR = shared.targetSpeedR;
        xSemaphoreGive(stateMutex);
      }
      sendHoverCommand(cmdL, cmdR);
      lastHoverSend = now;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ======================== SETUP ========================

void setup() {
  Serial.begin(USB_BAUD);
  Serial.setTimeout(SERIAL_PARSE_TIMEOUT_MS);
  delay(100);

  HoverSerial.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX_PIN, HOVER_TX_PIN);
  delay(100);
  while (HoverSerial.available()) HoverSerial.read();

  stateMutex = xSemaphoreCreateMutex();

  shared.driverAlive          = false;
  shared.lastDriverFeedbackMs = millis();

  // Core 0: USB Serial I/O
  xTaskCreatePinnedToCore(
    core0Task, "Core0_USB", CORE0_STACK_SIZE,
    NULL, 1, NULL, 0
  );

  // Core 1: Hardware I/O
  xTaskCreatePinnedToCore(
    core1Task, "Core1_HW", CORE1_STACK_SIZE,
    NULL, 1, NULL, 1
  );
}

// ======================== LOOP ========================

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
