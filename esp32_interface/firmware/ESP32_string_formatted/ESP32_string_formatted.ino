// =============================================================================
// ESP32 Complete System Control — Dual-Core FreeRTOS Architecture
// =============================================================================
// Hardware: Hoverboard motors (UART2), 2× MPU6050 IMU (I2C), BME680 (I2C),
//           PMS5003 (UART1), 3× RGB LED strips (MOSFET digital)
//
// Architecture matches grace_esp32_firmware: pure relay, all intelligence in ROS.
//   Core 0: USB Serial I/O  — send sensor data, receive motor + LED commands
//   Core 1: Hardware I/O    — hoverboard UART, I2C sensors, PMS5003 UART
//
// Protocol (compatible with esp32_interface_node.cpp):
//   ESP32 → Linux:  ,T0005,D1,b36000p,t0250p,L0050p,Lm0048p,R0050p,Rm0049p,
//                    I11,ax100500p,...,I21,...,BT025p,H045p,P10135p,G015p,
//                    PM1,P1010,P2025,P3040\n
//   Linux → ESP32:  l50 r50\n   (motor commands)
//                   1r\n         (LED strip 1 = red)
// =============================================================================

#include <HardwareSerial.h>
#include <Wire.h>
#include <cstring>

// ======================== USER-CONFIGURABLE MACROS ========================

#define SPEED_MAX               180
#define HOVERBOARD_SCALE_FACTOR 2.5f
#define HOVERBOARD_CMD_MAX      450
#define DRIVER_TIMEOUT_MS       500
#define HOVER_SEND_INTERVAL_MS  20
#define SENSOR_SEND_INTERVAL_MS 5
#define BME_READ_INTERVAL_MS    2000
#define MPU_READ_INTERVAL_MS    10
#define USB_BAUD                115200
#define HOVER_BAUD              115200

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
#define IMU1_ADDR               0x68
#define IMU2_ADDR               0x69

// BME680 address
#define BME680_ADDR             0x77

// RGB LED strip pins (3 strips, MOSFET driven, digital on/off)
#define STRIP1_RED_PIN          32
#define STRIP1_GREEN_PIN        33
#define STRIP1_BLUE_PIN         4
#define STRIP2_RED_PIN          19
#define STRIP2_GREEN_PIN        5
#define STRIP2_BLUE_PIN         18
#define STRIP3_RED_PIN          14
#define STRIP3_GREEN_PIN        15
#define STRIP3_BLUE_PIN         27

// FreeRTOS task stack sizes
#define CORE0_STACK_SIZE        8192
#define CORE1_STACK_SIZE        8192

#define SERIAL_PARSE_TIMEOUT_MS 5

// ======================== INTERNAL CONSTANTS ========================

static constexpr uint16_t START_FRAME      = 0xABCD;
static constexpr uint8_t  MPU_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t  MPU_ACCEL_XOUT_H = 0x3B;

// ======================== DATA STRUCTURES ========================

struct ImuRawData {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
};

struct HoverCommand {
  uint16_t start;
  int16_t  speedL;
  int16_t  speedR;
  uint16_t checksum;
};

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

struct BmeData {
  int16_t temperature;    // °C (integer part)
  int16_t humidity;       // % (integer part)
  int32_t pressure;       // hPa (integer part)
  int16_t gas_resistance; // kΩ (integer part)
  bool    available;
};

struct PmsData {
  uint16_t pm1_0;
  uint16_t pm2_5;
  uint16_t pm10_0;
  bool     available;
};

// Shared state between cores — protected by mutex
struct SharedState {
  // Written by Core 0, read by Core 1
  int16_t  targetSpeedL;
  int16_t  targetSpeedR;

  // Written by Core 1, read by Core 0
  int16_t  measuredSpeedL;
  int16_t  measuredSpeedR;
  int16_t  batteryMv;
  int16_t  boardTempDeciC;

  bool     driverAlive;
  uint32_t lastDriverFeedbackMs;

  ImuRawData imu1;
  ImuRawData imu2;
  bool       imu1Ok;
  bool       imu2Ok;

  BmeData  bme;
  PmsData  pms;
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

// ======================== BME680 MINIMAL I2C DRIVER ========================
// Simplified polling — reads calibrated data via Adafruit library on Core 1

#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

static Adafruit_BME680 bme;
static bool bmeHwAvailable = false;

static bool initBme680() {
  if (bme.begin(BME680_ADDR)) {
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
    return true;
  }
  return false;
}

// ======================== PMS5003 READER ========================

static bool readPms5003Frame(PmsData &out) {
  static uint8_t idx = 0;
  static uint8_t buffer[32];
  bool gotFrame = false;

  while (PMSSerial.available()) {
    uint8_t c = PMSSerial.read();
    if (idx == 0 && c != 0x42) continue;
    if (idx == 1 && c != 0x4D) { idx = 0; continue; }

    buffer[idx++] = c;
    if (idx == 32) {
      uint16_t checksum = 0;
      for (int i = 0; i < 30; i++) checksum += buffer[i];
      uint16_t frameChecksum = (static_cast<uint16_t>(buffer[30]) << 8) | buffer[31];
      uint16_t frameLength   = (static_cast<uint16_t>(buffer[2])  << 8) | buffer[3];

      if (checksum == frameChecksum && frameLength == 28) {
        out.pm1_0  = (static_cast<uint16_t>(buffer[10]) << 8) | buffer[11];
        out.pm2_5  = (static_cast<uint16_t>(buffer[12]) << 8) | buffer[13];
        out.pm10_0 = (static_cast<uint16_t>(buffer[14]) << 8) | buffer[15];
        out.available = true;
        gotFrame = true;
      }
      idx = 0;
    }
  }
  return gotFrame;
}

// ======================== HOVERBOARD HELPERS ========================

static int16_t scaleForHoverboard(int16_t rpm) {
  int16_t scaled = static_cast<int16_t>(rpm * HOVERBOARD_SCALE_FACTOR);
  if (scaled > HOVERBOARD_CMD_MAX)  scaled = HOVERBOARD_CMD_MAX;
  if (scaled < -HOVERBOARD_CMD_MAX) scaled = -HOVERBOARD_CMD_MAX;
  return scaled;
}

static void sendHoverCommand(int16_t speedL, int16_t speedR) {
  HoverCommand cmd;
  cmd.start    = START_FRAME;
  cmd.speedL   = scaleForHoverboard(speedL);
  cmd.speedR   = scaleForHoverboard(speedR);
  cmd.checksum = cmd.start ^ cmd.speedL ^ cmd.speedR;
  HoverSerial.write(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
}

// ======================== LED HELPERS ========================

static void setStripColor(int strip, bool r, bool g, bool b) {
  int rPin, gPin, bPin;
  switch (strip) {
    case 1: rPin = STRIP1_RED_PIN; gPin = STRIP1_GREEN_PIN; bPin = STRIP1_BLUE_PIN; break;
    case 2: rPin = STRIP2_RED_PIN; gPin = STRIP2_GREEN_PIN; bPin = STRIP2_BLUE_PIN; break;
    case 3: rPin = STRIP3_RED_PIN; gPin = STRIP3_GREEN_PIN; bPin = STRIP3_BLUE_PIN; break;
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

static void sendSensorString(const SharedState &s, uint32_t nowMs) {
  static uint32_t lastSendTime = 0;
  uint32_t dt = nowMs - lastSendTime;
  lastSendTime = nowMs;

  Serial.print(',');

  // Delta time
  Serial.print("T"); Serial.printf("%04lu", dt); Serial.print(',');

  // Driver-alive flag
  Serial.print("D"); Serial.print(s.driverAlive ? '1' : '0'); Serial.print(',');

  // Hoverboard data
  printField("b", s.batteryMv, 5);       Serial.print(',');
  printField("t", s.boardTempDeciC, 4);  Serial.print(',');
  printField("L", s.targetSpeedL, 4);    Serial.print(',');
  printField("Lm", s.measuredSpeedL, 4); Serial.print(',');
  printField("R", s.targetSpeedR, 4);    Serial.print(',');
  printField("Rm", s.measuredSpeedR, 4); Serial.print(',');

  // IMU 1
  Serial.print("I1"); Serial.print(s.imu1Ok ? '1' : '0'); Serial.print(',');
  printField("ax1", s.imu1.ax, 5); Serial.print(',');
  printField("ay1", s.imu1.ay, 5); Serial.print(',');
  printField("az1", s.imu1.az, 5); Serial.print(',');
  printField("gx1", s.imu1.gx, 5); Serial.print(',');
  printField("gy1", s.imu1.gy, 5); Serial.print(',');
  printField("gz1", s.imu1.gz, 5); Serial.print(',');

  // IMU 2
  Serial.print("I2"); Serial.print(s.imu2Ok ? '1' : '0'); Serial.print(',');
  printField("ax2", s.imu2.ax, 5); Serial.print(',');
  printField("ay2", s.imu2.ay, 5); Serial.print(',');
  printField("az2", s.imu2.az, 5); Serial.print(',');
  printField("gx2", s.imu2.gx, 5); Serial.print(',');
  printField("gy2", s.imu2.gy, 5); Serial.print(',');
  printField("gz2", s.imu2.gz, 5); Serial.print(',');

  // BME680
  printField("BT", s.bme.temperature, 3);    Serial.print(',');
  printField("H",  s.bme.humidity, 3);        Serial.print(',');
  printField("P",  (int16_t)s.bme.pressure, 5); Serial.print(',');
  printField("G",  s.bme.gas_resistance, 3);  Serial.print(',');

  // PMS5003 (unsigned values, no sign char)
  Serial.print("PM"); Serial.print(s.pms.available ? '1' : '0'); Serial.print(',');
  Serial.print("P1"); Serial.printf("%03d", s.pms.pm1_0);  Serial.print(',');
  Serial.print("P2"); Serial.printf("%03d", s.pms.pm2_5);  Serial.print(',');
  Serial.print("P3"); Serial.printf("%03d", s.pms.pm10_0);

  Serial.println();
}

// ======================== CORE 0 TASK: USB Serial I/O ========================

void core0Task(void *pvParameters) {
  (void)pvParameters;
  uint32_t lastSensorSend = 0;

  for (;;) {
    uint32_t now = millis();

    // ---------- RECEIVE COMMANDS FROM LINUX ----------
    static String inputLine = "";
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (inputLine.length() > 0) {
          inputLine.trim();
          inputLine.toLowerCase();

          // 1. Motor commands (e.g., "l50 r50", "l-20", "r100")
          if (inputLine.startsWith("l") || inputLine.startsWith("r")) {
            // Find 'l' and parse number
            int l_idx = inputLine.indexOf('l');
            if (l_idx >= 0) {
              int16_t speedL = inputLine.substring(l_idx + 1).toInt();
              speedL = constrain(speedL, -SPEED_MAX, SPEED_MAX);
              if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                shared.targetSpeedL = speedL;
                xSemaphoreGive(stateMutex);
              }
            }

            // Find 'r' and parse number
            int r_idx = inputLine.indexOf('r');
            if (r_idx >= 0) {
              int16_t speedR = inputLine.substring(r_idx + 1).toInt();
              speedR = constrain(speedR, -SPEED_MAX, SPEED_MAX);
              if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
                shared.targetSpeedR = speedR;
                xSemaphoreGive(stateMutex);
              }
            }
          }
          // 2. LED commands (e.g., "1w", "2r", "3b")
          else if (inputLine.length() == 2 && inputLine[0] >= '1' && inputLine[0] <= '3') {
            int strip = inputLine[0] - '0';
            char colorCmd = inputLine[1];
            switch (colorCmd) {
              case 'r': setStripColor(strip, true,  false, false); break;
              case 'g': setStripColor(strip, false, true,  false); break;
              case 'b': setStripColor(strip, false, false, true);  break;
              case 'y': setStripColor(strip, true,  true,  false); break;
              case 'c': setStripColor(strip, false, true,  true);  break;
              case 'm': setStripColor(strip, true,  false, true);  break;
              case 'w': setStripColor(strip, true,  true,  true);  break;
              case 'o': setStripColor(strip, false, false, false); break;
            }
          }
          // 3. All LEDs off command
          else if (inputLine == "a") {
            setStripColor(1, false, false, false);
            setStripColor(2, false, false, false);
            setStripColor(3, false, false, false);
          }

          inputLine = ""; // Clear for next command
        }
      } else {
        inputLine += c;
        if (inputLine.length() > 50) inputLine = ""; // Prevent memory overflow
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

  // Hoverboard receive state machine
  uint8_t  idx = 0;
  uint16_t bufStartFrame = 0;
  byte     incomingByte = 0;
  byte     incomingBytePrev = 0;
  byte    *p = nullptr;
  HoverFeedback newFeedback = {};

  bool localImu1Ok = false;
  bool localImu2Ok = false;

  // Init I2C and sensors on this core
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(50);
  localImu1Ok = initImu(IMU1_ADDR);
  localImu2Ok = initImu(IMU2_ADDR);
  bmeHwAvailable = initBme680();

  // Init PMS5003
  PMSSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  delay(100);

  // Write initial status
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    shared.imu1Ok = localImu1Ok;
    shared.imu2Ok = localImu2Ok;
    shared.bme.available = bmeHwAvailable;
    shared.pms.available = false;
    xSemaphoreGive(stateMutex);
  }

  uint32_t lastHoverSend = 0;
  uint32_t lastBmeRead   = 0;
  uint32_t lastMpuRead   = 0;

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
        if (p) { *p++ = incomingByte; idx++; }
        else   { idx = 0; }
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
    if (now - lastMpuRead >= MPU_READ_INTERVAL_MS) {
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
      lastMpuRead = now;
    }

    // ---------- READ BME680 ----------
    if (bmeHwAvailable && (now - lastBmeRead >= BME_READ_INTERVAL_MS)) {
      if (bme.performReading()) {
        BmeData bmeLocal;
        bmeLocal.temperature    = (int16_t)bme.temperature;
        bmeLocal.humidity       = (int16_t)bme.humidity;
        bmeLocal.pressure       = (int32_t)(bme.pressure / 100.0f);  // Pa -> hPa
        bmeLocal.gas_resistance = (int16_t)(bme.gas_resistance / 1000.0f);  // Ω -> kΩ
        bmeLocal.available      = true;

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          shared.bme = bmeLocal;
          xSemaphoreGive(stateMutex);
        }
      }
      lastBmeRead = now;
    }

    // ---------- READ PMS5003 ----------
    if (PMSSerial.available()) {
      PmsData pmsLocal = shared.pms;
      if (readPms5003Frame(pmsLocal)) {
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          shared.pms = pmsLocal;
          xSemaphoreGive(stateMutex);
        }
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

  // Initialize LED pins
  const int ledPins[] = {
    STRIP1_RED_PIN, STRIP1_GREEN_PIN, STRIP1_BLUE_PIN,
    STRIP2_RED_PIN, STRIP2_GREEN_PIN, STRIP2_BLUE_PIN,
    STRIP3_RED_PIN, STRIP3_GREEN_PIN, STRIP3_BLUE_PIN
  };
  for (int pin : ledPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }

  // Initialize hoverboard UART
  HoverSerial.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX_PIN, HOVER_TX_PIN);
  delay(100);
  while (HoverSerial.available()) HoverSerial.read();

  // Create mutex
  stateMutex = xSemaphoreCreateMutex();

  shared.driverAlive          = false;
  shared.lastDriverFeedbackMs = millis();

  // Launch Core 0 task (USB Serial I/O)
  xTaskCreatePinnedToCore(core0Task, "Core0_USB", CORE0_STACK_SIZE, NULL, 1, NULL, 0);

  // Launch Core 1 task (Hardware I/O)
  xTaskCreatePinnedToCore(core1Task, "Core1_HW",  CORE1_STACK_SIZE, NULL, 1, NULL, 1);
}

// ======================== LOOP ========================
// Not used — both cores run FreeRTOS tasks above.

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}