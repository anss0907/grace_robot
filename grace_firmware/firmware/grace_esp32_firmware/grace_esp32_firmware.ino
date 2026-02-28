// =============================================================================
// GRACE Robot — High-Performance Dual-Core ESP32 Firmware
// =============================================================================
// This firmware is a pure relay between Linux (USB Serial) and hardware
// (hoverboard motor driver + dual IMUs). All intelligence stays in ROS.
//
// Core 0: USB Serial I/O      — send sensor data, receive motor commands
// Core 1: Hardware I/O         — hoverboard UART + IMU I2C
//
// Protocol (compatible with grace_interface.cpp):
//   ESP32 → Linux:  ,b36000p,t0250p,Lm0100p,Rm0100p,I11,ax100500p,...\n
//   Linux → ESP32:  L50 R50\n
// =============================================================================

#include <HardwareSerial.h>
#include <Wire.h>
#include <cstring>

// ======================== USER-CONFIGURABLE MACROS ========================
// Change these values to match your hardware setup.

// Motor speed limits (RPM). The ESP32 will clamp commands to [-SPEED_MAX, +SPEED_MAX].
#define SPEED_MAX               180

// Hoverboard protocol scale factor. Multiplied with RPM before sending to
// the hoverboard controller. Adjust if your hoverboard expects different units.
#define HOVERBOARD_SCALE_FACTOR 2.5f

// Hoverboard protocol max (after scaling). Safety clamp.
#define HOVERBOARD_CMD_MAX      450

// Driver feedback timeout (ms). If no valid feedback is received from the
// hoverboard driver within this window, driver data is reset to zero and
// the driver-alive flag is cleared.
#define DRIVER_TIMEOUT_MS       500

// Hoverboard command send interval (ms). The hoverboard protocol needs
// periodic commands. 20ms = 50Hz is a good balance.
#define HOVER_SEND_INTERVAL_MS  20

// Sensor data send interval (ms). How often to stream the sensor string
// to Linux over USB Serial. 5ms ≈ 200Hz.
#define SENSOR_SEND_INTERVAL_MS 5

// Serial baud rates
#define USB_BAUD                115200
#define HOVER_BAUD              115200

// Serial parse timeout (ms). Low value prevents parseInt() from blocking.
#define SERIAL_PARSE_TIMEOUT_MS 5

// ESP32 UART2 pins for hoverboard
#define HOVER_RX_PIN            16
#define HOVER_TX_PIN            17

// I2C pins
#define I2C_SDA_PIN             21
#define I2C_SCL_PIN             22

// MPU6050 addresses
#define IMU1_ADDR               0x68  // AD0 = GND
#define IMU2_ADDR               0x69  // AD0 = 3.3V

// FreeRTOS task stack sizes (bytes)
#define CORE0_STACK_SIZE        4096
#define CORE1_STACK_SIZE        4096

// ======================== INTERNAL CONSTANTS ========================
static constexpr uint16_t START_FRAME       = 0xABCD;
static constexpr uint8_t  MPU_PWR_MGMT_1    = 0x6B;
static constexpr uint8_t  MPU_ACCEL_XOUT_H  = 0x3B;

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

// Shared state between cores — protected by mutex
struct SharedState {
  // Written by Core 0 (USB serial), read by Core 1 (hardware)
  int16_t  targetSpeedL;
  int16_t  targetSpeedR;

  // Written by Core 1 (hardware), read by Core 0 (USB serial)
  int16_t  measuredSpeedL;
  int16_t  measuredSpeedR;
  int16_t  batteryMv;
  int16_t  boardTempDeciC;

  // Driver-alive tracking
  bool     driverAlive;           // true when fresh feedback is arriving
  uint32_t lastDriverFeedbackMs;  // timestamp of last valid feedback packet

  ImuRawData imu1;
  ImuRawData imu2;
  bool       imu1Ok;
  bool       imu2Ok;
};

// ======================== GLOBALS ========================

HardwareSerial HoverSerial(2);
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

// ======================== SENSOR STRING BUILDER ========================

static void printField(const char* label, int16_t value, int width) {
  Serial.print(label);
  Serial.printf("%0*d", width, abs(value));
  Serial.print(value >= 0 ? 'p' : 'n');
}

static void sendSensorString(const SharedState &s, uint32_t nowMs) {
  // Track time between sends for dt field
  static uint32_t lastSendTime = 0;
  uint32_t dt = nowMs - lastSendTime;
  lastSendTime = nowMs;

  Serial.print(',');

  // Delta time in ms since last sensor string (4 digits, zero-padded)
  Serial.print("T");
  Serial.printf("%04lu", dt);               Serial.print(',');

  // Driver-alive flag: D1 = driver sending data, D0 = driver off/no data
  Serial.print("D");
  Serial.print(s.driverAlive ? '1' : '0');  Serial.print(',');

  printField("b", s.batteryMv, 5);        Serial.print(',');
  printField("t", s.boardTempDeciC, 4);    Serial.print(',');

  // Target speeds (what Linux commanded)
  printField("L", s.targetSpeedL, 4);      Serial.print(',');
  printField("Lm", s.measuredSpeedL, 4);   Serial.print(',');
  printField("R", s.targetSpeedR, 4);      Serial.print(',');
  printField("Rm", s.measuredSpeedR, 4);   Serial.print(',');

  // IMU 1
  Serial.print("I1");
  Serial.print(s.imu1Ok ? '1' : '0');      Serial.print(',');
  printField("ax1", s.imu1.ax, 5);         Serial.print(',');
  printField("ay1", s.imu1.ay, 5);         Serial.print(',');
  printField("az1", s.imu1.az, 5);         Serial.print(',');
  printField("gx1", s.imu1.gx, 5);         Serial.print(',');
  printField("gy1", s.imu1.gy, 5);         Serial.print(',');
  printField("gz1", s.imu1.gz, 5);         Serial.print(',');

  // IMU 2
  Serial.print("I2");
  Serial.print(s.imu2Ok ? '1' : '0');      Serial.print(',');
  printField("ax2", s.imu2.ax, 5);         Serial.print(',');
  printField("ay2", s.imu2.ay, 5);         Serial.print(',');
  printField("az2", s.imu2.az, 5);         Serial.print(',');
  printField("gx2", s.imu2.gx, 5);         Serial.print(',');
  printField("gy2", s.imu2.gy, 5);         Serial.print(',');
  printField("gz2", s.imu2.gz, 5);

  Serial.println();
}

// ======================== CORE 0 TASK: USB Serial I/O ========================
// Runs on Core 0. Handles all communication with Linux.
// - Reads motor commands from USB Serial
// - Sends sensor data string to USB Serial

void core0Task(void *pvParameters) {
  (void)pvParameters;
  uint32_t lastSensorSend = 0;

  for (;;) {
    uint32_t now = millis();

    // ---------- RECEIVE COMMANDS FROM LINUX ----------
    while (Serial.available()) {
      char c = Serial.peek();

      if (c == 'l') {
        Serial.read();  // consume 'l'
        int16_t speedL = Serial.parseInt();
        speedL = constrain(speedL, -SPEED_MAX, SPEED_MAX);

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          shared.targetSpeedL = speedL;
          xSemaphoreGive(stateMutex);
        }
      }
      else if (c == 'r') {
        Serial.read();  // consume 'r'
        int16_t speedR = Serial.parseInt();
        speedR = constrain(speedR, -SPEED_MAX, SPEED_MAX);

        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
          shared.targetSpeedR = speedR;
          xSemaphoreGive(stateMutex);
        }
      }
      else {
        Serial.read();  // consume unknown char (spaces, newlines, etc.)
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

    // Yield briefly to avoid starving the system
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ======================== CORE 1 TASK: Hardware I/O ========================
// Runs on Core 1. Handles all hardware communication.
// - Sends motor commands to hoverboard UART
// - Receives feedback from hoverboard UART
// - Reads IMU data via I2C

void core1Task(void *pvParameters) {
  (void)pvParameters;

  // Hoverboard receive state machine
  uint8_t  idx = 0;
  uint16_t bufStartFrame = 0;
  byte     incomingByte = 0;
  byte     incomingBytePrev = 0;
  byte    *p = nullptr;
  HoverFeedback newFeedback = {};

  // Local IMU status (only written by this core)
  bool localImu1Ok = false;
  bool localImu2Ok = false;

  // Init I2C and IMUs on this core
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(50);
  localImu1Ok = initImu(IMU1_ADDR);
  localImu2Ok = initImu(IMU2_ADDR);

  // Write initial IMU status
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    shared.imu1Ok = localImu1Ok;
    shared.imu2Ok = localImu2Ok;
    xSemaphoreGive(stateMutex);
  }

  uint32_t lastHoverSend = 0;

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
          // Valid packet — update shared state and mark driver alive
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

    // Write IMU data to shared state
    if (imu1Read || imu2Read) {
      if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1)) == pdTRUE) {
        if (imu1Read) shared.imu1 = imu1Data;
        if (imu2Read) shared.imu2 = imu2Data;
        shared.imu1Ok = localImu1Ok;
        shared.imu2Ok = localImu2Ok;
        xSemaphoreGive(stateMutex);
      }
    }

    // ---------- CHECK DRIVER ALIVE TIMEOUT ----------
    // If no valid feedback for DRIVER_TIMEOUT_MS, reset driver data to zero
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
    // Always send the last commanded speed — motors keep moving until
    // Linux explicitly sends l0 r0.
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

    // Yield briefly
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
  while (HoverSerial.available()) HoverSerial.read();  // flush

  // Create mutex for shared state
  stateMutex = xSemaphoreCreateMutex();

  // Initialize shared state
  shared.driverAlive          = false;
  shared.lastDriverFeedbackMs = millis();

  // Launch Core 0 task (USB Serial I/O)
  xTaskCreatePinnedToCore(
    core0Task,       // Task function
    "Core0_USB",     // Name
    CORE0_STACK_SIZE,
    NULL,            // Parameter
    1,               // Priority
    NULL,            // Task handle
    0                // Core 0
  );

  // Launch Core 1 task (Hardware I/O)
  xTaskCreatePinnedToCore(
    core1Task,       // Task function
    "Core1_HW",      // Name
    CORE1_STACK_SIZE,
    NULL,            // Parameter
    1,               // Priority
    NULL,            // Task handle
    1                // Core 1
  );
}

// ======================== LOOP ========================
// Not used — both cores run FreeRTOS tasks above.
// Arduino's loop() runs on Core 1 by default, but we use our own tasks.

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));  // idle forever
}
