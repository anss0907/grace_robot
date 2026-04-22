// ====================================================================
// ESP32 COMPLETE SYSTEM CONTROL
// - Hoverboard Motor Controller + Environmental Sensors
// - RGB LED Strip Control with MOSFET Drivers
// ====================================================================

#include <HardwareSerial.h>
#include <cstring>  // for memcpy
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <MPU6050_light.h>

// ========== UART FOR HOVERBOARD (ESP32 UART2) ==========
constexpr int HOVER_RX_PIN = 16;  // Hoverboard TX -> ESP32 RX
constexpr int HOVER_TX_PIN = 17;  // Hoverboard RX -> ESP32 TX
HardwareSerial HoverSerial(2);     // Use UART2 on ESP32

// ========== UART FOR PMS5003 (UART1) ==========
constexpr int PMS_RX_PIN = 25;    // PMS5003 TX -> ESP32 GPIO25
constexpr int PMS_TX_PIN = 26;    // PMS5003 RX -> ESP32 GPIO26
HardwareSerial PMSSerial(1);       // Use UART1 on ESP32

// ========== I2C FOR BME680 AND MPU6050 ==========
#define BME680_I2C_ADDR 0x77
Adafruit_BME680 bme;
bool bme680Available = false;
float bmeTemp = 0.0f;
float bmeHumidity = 0.0f;
float bmePressure = 0.0f;
float bmeGas = 0.0f;
bool bmeDataValid = false;
uint32_t lastBmeRead = 0;
constexpr uint32_t BME_READ_INTERVAL = 2000;  // ms

// ========== MPU6050 IMU SENSORS ==========
MPU6050 mpu1(Wire);  // IMU #1 with AD0 LOW (address 0x68)
MPU6050 mpu2(Wire);  // IMU #2 with AD0 HIGH (address 0x69)
bool mpu1Available = false;
bool mpu2Available = false;
float mpu1AccX = 0.0f, mpu1AccY = 0.0f, mpu1AccZ = 0.0f;
float mpu1GyroX = 0.0f, mpu1GyroY = 0.0f, mpu1GyroZ = 0.0f;
float mpu2AccX = 0.0f, mpu2AccY = 0.0f, mpu2AccZ = 0.0f;
float mpu2GyroX = 0.0f, mpu2GyroY = 0.0f, mpu2GyroZ = 0.0f;
uint32_t lastMpuRead = 0;
constexpr uint32_t MPU_READ_INTERVAL = 100;  // ms

// ========== PMS5003 SENSOR DATA ==========
struct PMSData {
  uint16_t pm1_0;    // PM1.0
  uint16_t pm2_5;    // PM2.5
  uint16_t pm10_0;   // PM10
};
PMSData pmsData = {0, 0, 0};
bool pmsAvailable = false;

// ========== RGB LED STRIP CONTROL (3 STRIPS WITH MOSFET DRIVERS) ==========
// LED Strip 1
#define STRIP1_RED_PIN 4
#define STRIP1_GREEN_PIN 32
#define STRIP1_BLUE_PIN 33

// LED Strip 2
#define STRIP2_RED_PIN 18
#define STRIP2_GREEN_PIN 19
#define STRIP2_BLUE_PIN 5

// LED Strip 3
#define STRIP3_RED_PIN 27
#define STRIP3_GREEN_PIN 14
#define STRIP3_BLUE_PIN 23

// ========== HOVERBOARD COMMUNICATION ==========
constexpr uint32_t HOVER_BAUD = 115200;
constexpr uint32_t SERIAL_BAUD = 115200;
constexpr uint16_t START_FRAME = 0xABCD;
constexpr uint32_t SEND_INTERVAL = 100;      // ms
constexpr uint32_t DISPLAY_INTERVAL = 100;   // ms

// SAFETY: Hardware-safe speed limits
constexpr int16_t SPEED_MAX = 160;
constexpr int16_t SPEED_MIN = -160;
constexpr int16_t SPEED_CHANGE_MAX = 5;      // step per loop

// SAFETY: Battery and temperature thresholds
constexpr int16_t BATTERY_FULL = 4200;       // mV
constexpr int16_t BATTERY_GOOD = 3800;       // mV
constexpr int16_t BATTERY_LOW = 3650;        // mV
constexpr int16_t BATTERY_CRITICAL = 3500;   // mV
constexpr int16_t TEMP_NORMAL = 450;         // deci-deg C
constexpr int16_t TEMP_WARM = 550;           // deci-deg C
constexpr int16_t TEMP_HOT = 600;            // deci-deg C

// Command structure for TANK_STEERING mode (EFeru hoverboard firmware)
struct SerialCommand {
  uint16_t start;
  int16_t cmd1;      // Left motor command
  int16_t cmd2;      // Right motor command
  uint16_t checksum;
};

// Feedback structure from hoverboard
struct SerialFeedback {
  uint16_t start;
  int16_t cmd1;
  int16_t cmd2;
  int16_t speedR_meas;
  int16_t speedL_meas;
  int16_t batVoltage;
  int16_t boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
};

// Global variables
SerialCommand Command;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

int16_t targetSpeedL = 0;
int16_t targetSpeedR = 0;
int16_t currentSpeedL = 0;
int16_t currentSpeedR = 0;
bool emergencyStop = false;
uint32_t lastSend = 0;
uint32_t lastDisplay = 0;
uint32_t packetCount = 0;
uint32_t lastValidPacket = 0;

uint8_t idx = 0;
uint16_t bufStartFrame = 0;
byte *p = nullptr;
byte incomingByte = 0;
byte incomingBytePrev = 0;

// ========== HOVERBOARD FUNCTIONS ==========
int16_t applyRateLimit(int16_t current, int16_t target) {
  if (emergencyStop) return 0;

  int16_t diff = target - current;
  if (diff > SPEED_CHANGE_MAX) return current + SPEED_CHANGE_MAX;
  if (diff < -SPEED_CHANGE_MAX) return current - SPEED_CHANGE_MAX;
  return target;
}

int16_t scaleSpeedCommand(int16_t desiredRPM) {
  int16_t scaledCommand = static_cast<int16_t>(desiredRPM * 2.5);  // hoverboard protocol scale
  if (scaledCommand > 400) scaledCommand = 400;
  if (scaledCommand < -400) scaledCommand = -400;
  return scaledCommand;
}

void SendCommand(int16_t speedL, int16_t speedR) {
  const float batteryVolt = Feedback.batVoltage / 100.0f;
  const float temperature = Feedback.boardTemp / 10.0f;

  // Emergency stop conditions
  if (batteryVolt < BATTERY_CRITICAL / 100.0f && batteryVolt > 10.0f) {
    emergencyStop = true;
    Serial.println("WARNING EMERGENCY: Battery critically low!");
  }
  if (temperature > TEMP_HOT / 10.0f && temperature < 100.0f) {
    emergencyStop = true;
    Serial.println("WARNING EMERGENCY: Temperature too high!");
  }
  if (millis() - lastValidPacket > 2000 && packetCount > 0) {
    emergencyStop = true;
    Serial.println("WARNING EMERGENCY: Communication timeout!");
  }

  if (emergencyStop) {
    speedL = 0;
    speedR = 0;
    targetSpeedL = 0;
    targetSpeedR = 0;
  }
  currentSpeedL = applyRateLimit(currentSpeedL, speedL);
  currentSpeedR = applyRateLimit(currentSpeedR, speedR);

  const int16_t scaledSpeedL = scaleSpeedCommand(currentSpeedL);
  const int16_t scaledSpeedR = scaleSpeedCommand(currentSpeedR);

  Command.start = START_FRAME;
  Command.cmd1 = scaledSpeedL;
  Command.cmd2 = scaledSpeedR;
  Command.checksum = Command.start ^ Command.cmd1 ^ Command.cmd2;

  HoverSerial.write(reinterpret_cast<uint8_t *>(&Command), sizeof(Command));
}

void Receive() {
  if (!HoverSerial.available()) return;

  incomingByte = HoverSerial.read();
  bufStartFrame = (static_cast<uint16_t>(incomingByte) << 8) | incomingBytePrev;

  if (bufStartFrame == START_FRAME) {
    p = reinterpret_cast<byte *>(&NewFeedback);
    *p++ = incomingBytePrev;
    *p++ = incomingByte;
    idx = 2;
  } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {
    *p++ = incomingByte;
    idx++;
  }

  if (idx == sizeof(SerialFeedback)) {
    const uint16_t checksum = NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^
                              NewFeedback.speedR_meas ^ NewFeedback.speedL_meas ^
                              NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed;

    if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
      memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));
      packetCount++;
      lastValidPacket = millis();
    }
    idx = 0;
  }

  incomingBytePrev = incomingByte;
}

// ========== RGB LED FUNCTIONS ==========
void setStripColor(int strip, bool red, bool green, bool blue, const char* colorName) {
  int redPin, greenPin, bluePin;
  
  // Select pins based on strip number
  switch(strip) {
    case 1:
      redPin = STRIP1_RED_PIN;
      greenPin = STRIP1_GREEN_PIN;
      bluePin = STRIP1_BLUE_PIN;
      break;
    case 2:
      redPin = STRIP2_RED_PIN;
      greenPin = STRIP2_GREEN_PIN;
      bluePin = STRIP2_BLUE_PIN;
      break;
    case 3:
      redPin = STRIP3_RED_PIN;
      greenPin = STRIP3_GREEN_PIN;
      bluePin = STRIP3_BLUE_PIN;
      break;
    default:
      Serial.println("✗ Invalid strip number (1-3)");
      return;
  }
  
  digitalWrite(redPin, red ? HIGH : LOW);
  digitalWrite(greenPin, green ? HIGH : LOW);
  digitalWrite(bluePin, blue ? HIGH : LOW);
  
  Serial.print("✓ Strip ");
  Serial.print(strip);
  Serial.print(": ");
  Serial.print(colorName);
  Serial.print(" (R=");
  Serial.print(red ? "ON" : "OFF");
  Serial.print(" G=");
  Serial.print(green ? "ON" : "OFF");
  Serial.print(" B=");
  Serial.print(blue ? "ON" : "OFF");
  Serial.println(")");
}

void printLedStatus() {
  Serial.println("LED Status:");
  
  // Strip 1
  Serial.print("  Strip 1: ");
  Serial.print(digitalRead(STRIP1_RED_PIN) ? "R=ON " : "R=OFF");
  Serial.print(" | ");
  Serial.print(digitalRead(STRIP1_GREEN_PIN) ? "G=ON " : "G=OFF");
  Serial.print(" | ");
  Serial.println(digitalRead(STRIP1_BLUE_PIN) ? "B=ON" : "B=OFF");
  
  // Strip 2
  Serial.print("  Strip 2: ");
  Serial.print(digitalRead(STRIP2_RED_PIN) ? "R=ON " : "R=OFF");
  Serial.print(" | ");
  Serial.print(digitalRead(STRIP2_GREEN_PIN) ? "G=ON " : "G=OFF");
  Serial.print(" | ");
  Serial.println(digitalRead(STRIP2_BLUE_PIN) ? "B=ON" : "B=OFF");
  
  // Strip 3
  Serial.print("  Strip 3: ");
  Serial.print(digitalRead(STRIP3_RED_PIN) ? "R=ON " : "R=OFF");
  Serial.print(" | ");
  Serial.print(digitalRead(STRIP3_GREEN_PIN) ? "G=ON " : "G=OFF");
  Serial.print(" | ");
  Serial.println(digitalRead(STRIP3_BLUE_PIN) ? "B=ON" : "B=OFF");
}

// ========== DISPLAY FUNCTIONS ==========
void displayRealTimeData() {
  const float batteryVolt = Feedback.batVoltage / 100.0f;
  const float temperature = Feedback.boardTemp / 10.0f;

  Serial.print("[");
  Serial.print(millis() / 1000);
  Serial.print("s] ");

  Serial.print("BAT:");
  Serial.print(batteryVolt, 1);
  Serial.print("V");
  if (batteryVolt >= BATTERY_GOOD / 100.0f) Serial.print("[OK]");
  else if (batteryVolt >= BATTERY_LOW / 100.0f) Serial.print("[!]");
  else Serial.print("[WARN]");

  Serial.print(" | BOARD_TEMP:");
  Serial.print(temperature, 1);
  Serial.print("C");
  if (temperature <= TEMP_NORMAL / 10.0f) Serial.print("[OK]");
  else if (temperature <= TEMP_WARM / 10.0f) Serial.print("[!]");
  else Serial.print("[WARN]");

  Serial.print(" | L:");
  Serial.print(targetSpeedL);
  Serial.print("->");
  Serial.print(currentSpeedL);
  Serial.print("(");
  Serial.print(Feedback.speedL_meas);
  Serial.print(")");

  Serial.print(" R:");
  Serial.print(targetSpeedR);
  Serial.print("->");
  Serial.print(currentSpeedR);
  Serial.print("(");
  Serial.print(Feedback.speedR_meas);
  Serial.print(")");

  if (emergencyStop) Serial.print(" | [EMERGENCY]");

  Serial.println();

  Serial.print("BME680: ");
  if (bme680Available && bmeDataValid) {
    Serial.print("T=");
    Serial.print(bmeTemp, 1);
    Serial.print("C H=");
    Serial.print(bmeHumidity, 1);
    Serial.print("% P=");
    Serial.print(bmePressure, 1);
    Serial.print("hPa G=");
    Serial.print(bmeGas, 1);
    Serial.println("kOhm");
  } else if (!bme680Available) {
    Serial.println("N/A");
  } else {
    Serial.println("No data");
  }

  Serial.print("PMS5003: ");
  if (pmsAvailable) {
    Serial.print("PM1.0=");
    Serial.print(pmsData.pm1_0);
    Serial.print(" PM2.5=");
    Serial.print(pmsData.pm2_5);
    Serial.print(" PM10=");
    Serial.println(pmsData.pm10_0);
  } else {
    Serial.println("N/A");
  }

  Serial.print("MPU1: ");
  if (mpu1Available) {
    Serial.print("Acc=");
    Serial.print(mpu1AccX, 2);
    Serial.print(",");
    Serial.print(mpu1AccY, 2);
    Serial.print(",");
    Serial.print(mpu1AccZ, 2);
    Serial.print("g Gyro=");
    Serial.print(mpu1GyroX, 1);
    Serial.print(",");
    Serial.print(mpu1GyroY, 1);
    Serial.print(",");
    Serial.print(mpu1GyroZ, 1);
    Serial.println("°/s");
  } else {
    Serial.println("N/A");
  }

  Serial.print("MPU2: ");
  if (mpu2Available) {
    Serial.print("Acc=");
    Serial.print(mpu2AccX, 2);
    Serial.print(",");
    Serial.print(mpu2AccY, 2);
    Serial.print(",");
    Serial.print(mpu2AccZ, 2);
    Serial.print("g Gyro=");
    Serial.print(mpu2GyroX, 1);
    Serial.print(",");
    Serial.print(mpu2GyroY, 1);
    Serial.print(",");
    Serial.print(mpu2GyroZ, 1);
    Serial.println("°/s");
  } else {
    Serial.println("N/A");
  }

  // Show LED status
  printLedStatus();

  Serial.println();
}

void showDetailedStatus() {
  const float batteryVolt = Feedback.batVoltage / 100.0f;
  const float temperature = Feedback.boardTemp / 10.0f;

  Serial.println("\n+============ HOVERBOARD STATUS ============+");
  Serial.print("| BATTERY: ");
  Serial.print(batteryVolt, 2);
  Serial.println("V                        |");

  Serial.print("| TEMPERATURE: ");
  Serial.print(temperature, 1);
  Serial.println("C                 |");

  Serial.print("| CMD_LED: ");
  Serial.print(Feedback.cmdLed, HEX);
  Serial.println(" (status flags)       |");

  Serial.print("| EMERGENCY STOP: ");
  if (emergencyStop) Serial.println("ACTIVE              |");
  else Serial.println("CLEAR               |");

  Serial.print("| LEFT:  TGT=");
  Serial.print(targetSpeedL);
  Serial.print(" CUR=");
  Serial.print(currentSpeedL);
  Serial.print(" ENC=");
  Serial.println(Feedback.speedL_meas);

  Serial.print("| RIGHT: TGT=");
  Serial.print(targetSpeedR);
  Serial.print(" CUR=");
  Serial.print(currentSpeedR);
  Serial.print(" ENC=");
  Serial.println(Feedback.speedR_meas);

  Serial.print("| PACKETS: ");
  Serial.println(packetCount);
  Serial.println("+==========================================+\n");
}

void showEnvironmentalData() {
  Serial.println("\n+============== ENVIRONMENTAL SENSORS ==============+");

  // BME680
  if (bme680Available && bmeDataValid) {
    Serial.print("| BME680:");
    Serial.print(" Temp=");
    Serial.print(bmeTemp, 1);
    Serial.print("C Humidity=");
    Serial.print(bmeHumidity, 1);
    Serial.print("%");
    Serial.println();
    Serial.print("| Pressure=");
    Serial.print(bmePressure, 1);
    Serial.print("hPa Gas=");
    Serial.print(bmeGas, 1);
    Serial.println("kOhm");
  } else if (bme680Available) {
    Serial.println("| BME680: No data yet");
  } else {
    Serial.println("| BME680: Not available");
  }

  // PMS5003
  if (pmsAvailable) {
    Serial.print("| PMS5003: PM1.0=");
    Serial.print(pmsData.pm1_0);
    Serial.print("µg/m³ PM2.5=");
    Serial.print(pmsData.pm2_5);
    Serial.print("µg/m³ PM10=");
    Serial.print(pmsData.pm10_0);
    Serial.println("µg/m³");
  } else {
    Serial.println("| PMS5003: Not available");
  }

  // MPU6050 #1
  if (mpu1Available) {
    Serial.print("| MPU1 Accel: X=");
    Serial.print(mpu1AccX, 2);
    Serial.print("g Y=");
    Serial.print(mpu1AccY, 2);
    Serial.print("g Z=");
    Serial.print(mpu1AccZ, 2);
    Serial.println("g");
    Serial.print("| MPU1 Gyro: X=");
    Serial.print(mpu1GyroX, 1);
    Serial.print("°/s Y=");
    Serial.print(mpu1GyroY, 1);
    Serial.print("°/s Z=");
    Serial.print(mpu1GyroZ, 1);
    Serial.println("°/s");
  } else {
    Serial.println("| MPU6050 #1: Not available");
  }

  // MPU6050 #2
  if (mpu2Available) {
    Serial.print("| MPU2 Accel: X=");
    Serial.print(mpu2AccX, 2);
    Serial.print("g Y=");
    Serial.print(mpu2AccY, 2);
    Serial.print("g Z=");
    Serial.print(mpu2AccZ, 2);
    Serial.println("g");
    Serial.print("| MPU2 Gyro: X=");
    Serial.print(mpu2GyroX, 1);
    Serial.print("°/s Y=");
    Serial.print(mpu2GyroY, 1);
    Serial.print("°/s Z=");
    Serial.print(mpu2GyroZ, 1);
    Serial.println("°/s");
  } else {
    Serial.println("| MPU6050 #2: Not available");
  }

  Serial.println("+====================================================+\n");
}

// ========== SENSOR READ FUNCTIONS ==========
void readPMS5003() {
  static uint8_t idx = 0;
  static uint8_t buffer[32];

  while (PMSSerial.available()) {
    uint8_t c = PMSSerial.read();

    if (idx == 0 && c != 0x42) continue;  // Wait for start byte
    if (idx == 1 && c != 0x4D) {
      idx = 0;
      continue;  // Wait for second start byte
    }

    buffer[idx++] = c;

    if (idx == 32) {
      // Check length and checksum
      uint16_t checksum = 0;
      for (int i = 0; i < 30; i++) checksum += buffer[i];
      const uint16_t frameChecksum = (static_cast<uint16_t>(buffer[30]) << 8) | buffer[31];

      // Expected frame length is 28 bytes (0x1C) after the length field
      const uint16_t frameLength = (static_cast<uint16_t>(buffer[2]) << 8) | buffer[3];

      if (checksum == frameChecksum && frameLength == 28) {
        // Atmospheric concentration values per PMS5003 datasheet
        pmsData.pm1_0 = (static_cast<uint16_t>(buffer[10]) << 8) | buffer[11];
        pmsData.pm2_5 = (static_cast<uint16_t>(buffer[12]) << 8) | buffer[13];
        pmsData.pm10_0 = (static_cast<uint16_t>(buffer[14]) << 8) | buffer[15];
      }
      idx = 0;
    }
  }
}

// ========== INPUT HANDLING ==========
void processInput() {
  if (!Serial.available()) return;

  const char input = Serial.read();

  // Skip newline and carriage return
  if (input == '\n' || input == '\r') return;

  switch (input) {
    // ========== MOTOR COMMANDS ==========
    case 'l':
    case 'L': {
      const int speed = Serial.parseInt();
      targetSpeedL = constrain(speed, SPEED_MIN, SPEED_MAX);
      Serial.print("-> Left motor: ");
      Serial.print(targetSpeedL);
      if (targetSpeedL > 0) Serial.println(" (FORWARD)");
      else if (targetSpeedL < 0) Serial.println(" (REVERSE)");
      else Serial.println(" (STOPPED)");
      break;
    }

    case 'r':
    case 'R': {
      const int speed = Serial.parseInt();
      targetSpeedR = constrain(speed, SPEED_MIN, SPEED_MAX);
      Serial.print("-> Right motor: ");
      Serial.print(targetSpeedR);
      if (targetSpeedR > 0) Serial.println(" (FORWARD)");
      else if (targetSpeedR < 0) Serial.println(" (REVERSE)");
      else Serial.println(" (STOPPED)");
      break;
    }

    case 'x':
    case 'X':
      emergencyStop = true;
      targetSpeedL = 0;
      targetSpeedR = 0;
      currentSpeedL = 0;
      currentSpeedR = 0;
      Serial.println("[EMERGENCY] STOP ACTIVATED - All motors stopped!");
      break;

    case 'z':
    case 'Z':
      emergencyStop = false;
      Serial.println("[OK] Emergency stop cleared - System ready");
      break;

    case 's':
    case 'S':
      showDetailedStatus();
      break;

    case 'e':
    case 'E':
      showEnvironmentalData();
      break;

    case 'c':
    case 'C':
      Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
      Serial.println("Screen cleared - Real-time monitoring continues...\n");
      break;

    // ========== RGB LED COMMANDS (3 STRIPS) ==========
    // Format: <strip_number><color>
    // Examples: 1r, 2g, 3b, 1y, 2w, 3o
    case '1':
    case '2':
    case '3': {
      int strip = input - '0';  // Convert char to int
      
      // Read next character for color command
      while (!Serial.available()) { delay(1); }  // Wait for color command
      char colorCmd = Serial.read();
      
      switch(colorCmd) {
        case 'r':
        case 'R':
          setStripColor(strip, true, false, false, "RED");
          break;
        case 'g':
        case 'G':
          setStripColor(strip, false, true, false, "GREEN");
          break;
        case 'b':
        case 'B':
          setStripColor(strip, false, false, true, "BLUE");
          break;
        case 'y':
        case 'Y':
          setStripColor(strip, true, true, false, "YELLOW (R+G)");
          break;
        case 'c':
        case 'C':
          setStripColor(strip, false, true, true, "CYAN (G+B)");
          break;
        case 'm':
        case 'M':
          setStripColor(strip, true, false, true, "MAGENTA (R+B)");
          break;
        case 'w':
        case 'W':
          setStripColor(strip, true, true, true, "WHITE (All)");
          break;
        case 'o':
        case 'O':
          setStripColor(strip, false, false, false, "OFF");
          break;
        default:
          Serial.print("✗ Invalid color command: ");
          Serial.println(colorCmd);
          Serial.println("Valid colors: r, g, b, y, c, m, w, o");
          break;
      }
      break;
    }

    case 'a':
    case 'A':
      // All strips OFF
      setStripColor(1, false, false, false, "OFF");
      setStripColor(2, false, false, false, "OFF");
      setStripColor(3, false, false, false, "OFF");
      Serial.println("✓ All strips turned OFF");
      break;

    case 'h':
    case 'H':
      printHelp();
      break;

    default:
      Serial.print("Unknown command: ");
      Serial.print(input);
      Serial.println(" - Type 'h' for help");
      break;
  }
}

void printHelp() {
  Serial.println("\n====================================================================");
  Serial.println("   ESP32 COMPLETE SYSTEM - COMMAND REFERENCE");
  Serial.println("====================================================================");
  Serial.println();
  Serial.println("MOTOR COMMANDS:");
  Serial.println("  l<speed>        Left motor (-160 to 160) e.g., l100, l-50");
  Serial.println("  r<speed>        Right motor (-160 to 160) e.g., r100, r-50");
  Serial.println("  x               EMERGENCY STOP (all motors)");
  Serial.println("  z               Clear emergency stop");
  Serial.println();
  Serial.println("LED COMMANDS (3 Strips):");
  Serial.println("  Format: <strip><color>  e.g., 1r, 2g, 3b");
  Serial.println("  ");
  Serial.println("  Strip Numbers:  1, 2, 3");
  Serial.println("  Colors:");
  Serial.println("    r = Red         e.g., 1r = Strip 1 Red");
  Serial.println("    g = Green       e.g., 2g = Strip 2 Green");
  Serial.println("    b = Blue        e.g., 3b = Strip 3 Blue");
  Serial.println("    y = Yellow      e.g., 1y = Strip 1 Yellow");
  Serial.println("    c = Cyan        e.g., 2c = Strip 2 Cyan");
  Serial.println("    m = Magenta     e.g., 3m = Strip 3 Magenta");
  Serial.println("    w = White       e.g., 1w = Strip 1 White");
  Serial.println("    o = OFF         e.g., 2o = Strip 2 OFF");
  Serial.println("  ");
  Serial.println("  a               Turn ALL strips OFF");
  Serial.println();
  Serial.println("DISPLAY COMMANDS:");
  Serial.println("  s               Show hoverboard status");
  Serial.println("  e               Show environmental sensors");
  Serial.println("  c               Clear screen");
  Serial.println("  h               Show this help menu");
  Serial.println("====================================================================\n");
}

// ========== ARDUINO LIFECYCLE ==========
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);

  Serial.println("\n\n\n");
  Serial.println("====================================================================");
  Serial.println("   ESP32 COMPLETE SYSTEM CONTROLLER");
  Serial.println("   - Hoverboard Motor Control + All Sensors");
  Serial.println("   - RGB LED Strip Control (MOSFET Drivers)");
  Serial.println("====================================================================");
  Serial.println();

  // ========== INITIALIZE RGB LED PINS (3 STRIPS) ==========
  // Strip 1
  pinMode(STRIP1_RED_PIN, OUTPUT);
  pinMode(STRIP1_GREEN_PIN, OUTPUT);
  pinMode(STRIP1_BLUE_PIN, OUTPUT);
  digitalWrite(STRIP1_RED_PIN, LOW);
  digitalWrite(STRIP1_GREEN_PIN, LOW);
  digitalWrite(STRIP1_BLUE_PIN, LOW);
  
  // Strip 2
  pinMode(STRIP2_RED_PIN, OUTPUT);
  pinMode(STRIP2_GREEN_PIN, OUTPUT);
  pinMode(STRIP2_BLUE_PIN, OUTPUT);
  digitalWrite(STRIP2_RED_PIN, LOW);
  digitalWrite(STRIP2_GREEN_PIN, LOW);
  digitalWrite(STRIP2_BLUE_PIN, LOW);
  
  // Strip 3
  pinMode(STRIP3_RED_PIN, OUTPUT);
  pinMode(STRIP3_GREEN_PIN, OUTPUT);
  pinMode(STRIP3_BLUE_PIN, OUTPUT);
  digitalWrite(STRIP3_RED_PIN, LOW);
  digitalWrite(STRIP3_GREEN_PIN, LOW);
  digitalWrite(STRIP3_BLUE_PIN, LOW);
  
  Serial.println("[OK] 3x RGB LED strips initialized - All OFF");
  Serial.println("     Strip 1: GPIOs 19, 4, 5");
  Serial.println("     Strip 2: GPIOs 18, 23, 13");
  Serial.println("     Strip 3: GPIOs 14, 27, 15");

  // ========== INITIALIZE HOVERBOARD UART ==========
  HoverSerial.begin(HOVER_BAUD, SERIAL_8N1, HOVER_RX_PIN, HOVER_TX_PIN);
  delay(100);
  while (HoverSerial.available()) HoverSerial.read();
  Serial.print("[OK] UART2 initialized for Hoverboard (RX=");
  Serial.print(HOVER_RX_PIN);
  Serial.print(" TX=");
  Serial.print(HOVER_TX_PIN);
  Serial.println(")");

  // ========== INITIALIZE I2C FOR BME680 AND MPU6050 ==========
  Wire.begin(21, 22);  // SDA=21, SCL=22 (ESP32 standard)
  delay(100);

  if (bme.begin(BME680_I2C_ADDR)) {
    bme680Available = true;
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150);
    Serial.println("[OK] BME680 sensor initialized (I2C: SDA=21, SCL=22)");
  } else {
    Serial.println("[WARN] BME680 not found");
    bme680Available = false;
  }

  // ========== INITIALIZE MPU6050 IMU SENSORS ==========
  Serial.print("[...] Initializing MPU6050 #1 (0x68)...");
  mpu1.setAddress(0x68);
  if (mpu1.begin() == 0) {
    mpu1Available = true;
    mpu1.calcOffsets(true, true);  // gyro and accelerometer calibration
    Serial.println(" OK");
  } else {
    Serial.println(" FAILED");
    mpu1Available = false;
  }

  Serial.print("[...] Initializing MPU6050 #2 (0x69)...");
  mpu2.setAddress(0x69);
  if (mpu2.begin() == 0) {
    mpu2Available = true;
    mpu2.calcOffsets(true, true);  // gyro and accelerometer calibration
    Serial.println(" OK");
  } else {
    Serial.println(" FAILED");
    mpu2Available = false;
  }

  // ========== INITIALIZE PMS5003 (UART1) ==========
  PMSSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  delay(100);
  pmsAvailable = true;
  Serial.println("[OK] PMS5003 sensor initialized (UART1: GPIO25, GPIO26)");

  Serial.println();
  printHelp();

  delay(500);
  Serial.println("OK System ready - Starting monitoring...\n");

  lastValidPacket = millis();
}

void loop() {
  Receive();
  processInput();

  // Read BME680 periodically (cache latest values for real-time display)
  const uint32_t now = millis();
  if (bme680Available && (now - lastBmeRead >= BME_READ_INTERVAL)) {
    if (bme.performReading()) {
      bmeTemp = bme.temperature;
      bmeHumidity = bme.humidity;
      bmePressure = bme.pressure / 100.0f;       // Pa -> hPa
      bmeGas = bme.gas_resistance / 1000.0f;     // ohms -> kOhm
      bmeDataValid = true;
    } else {
      bmeDataValid = false;
    }
    lastBmeRead = now;
  }

  // Read MPU6050 IMU sensors periodically
  if ((mpu1Available || mpu2Available) && (now - lastMpuRead >= MPU_READ_INTERVAL)) {
    if (mpu1Available) {
      mpu1.update();
      mpu1AccX = mpu1.getAccX();
      mpu1AccY = mpu1.getAccY();
      mpu1AccZ = mpu1.getAccZ();
      mpu1GyroX = mpu1.getGyroX();
      mpu1GyroY = mpu1.getGyroY();
      mpu1GyroZ = mpu1.getGyroZ();
    }
    if (mpu2Available) {
      mpu2.update();
      mpu2AccX = mpu2.getAccX();
      mpu2AccY = mpu2.getAccY();
      mpu2AccZ = mpu2.getAccZ();
      mpu2GyroX = mpu2.getGyroX();
      mpu2GyroY = mpu2.getGyroY();
      mpu2GyroZ = mpu2.getGyroZ();
    }
    lastMpuRead = now;
  }

  // Read PMS5003 periodically
  if (pmsAvailable && PMSSerial.available()) {
    readPMS5003();
  }

  if (millis() - lastSend >= SEND_INTERVAL) {
    SendCommand(targetSpeedL, targetSpeedR);
    lastSend = millis();
  }

  if (millis() - lastDisplay >= DISPLAY_INTERVAL) {
    displayRealTimeData();
    lastDisplay = millis();
  }
}
