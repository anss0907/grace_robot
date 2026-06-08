// =============================================================================
// arduino_nano_interface.ino — Grace Robot Arduino Nano Interface
// =============================================================================
// Hardware:
//   US1-US4 (D2-D9)  : Ultrasonic sensors (Front, Rear, Left, Right)
//   VS1 (A6)         : 24V battery voltage divider
//   VS2 (A7)         : 19V buck converter output voltage divider
//   CS1-CS4 (A2-A5)  : ACS712 ELC-20A current sensors (100 mV/A)
//                       CS1=40V batt, CS2=24V batt, CS3=40V charger, CS4=24V charger
//   MQ  (A0)         : General gas/smoke sensor
//   MHMQ (A1)        : CO/methane gas sensor
//   R1 (D10)         : Relay — 40V charger adapter
//   R2 (D11)         : Relay — 24V charger adapter
//
// Protocol (matches esp32_interface pattern — integer-only, fixed-width):
//   Nano → ROS: ,T<4>,Uf<5><s>,Ub<5><s>,Ul<5><s>,Ur<5><s>,V1<5><s>,V2<5><s>,
//               C1<5><s>,C2<5><s>,C3<5><s>,C4<5><s>,MQ<4>,M2<4>,RB<1>\n
//     <N>  = N zero-padded digits
//     <s>  = 'p' (positive/zero) or 'n' (negative)
//
//   ROS → Nano: '0' = both off, '1' = 40V on, '2' = 24V on, '3' = both on
//
// Integer units sent on wire:
//   T   = delta time in ms (unsigned)
//   Uf/Ub/Ul/Ur = distance in tenths-of-cm (signed). -10 = no echo (= -1.0 cm)
//   V1/V2 = voltage in millivolts (signed)
//   C1-C4 = current in milliamps (signed)
//   MQ/M2 = raw ADC 0-1023 (unsigned)
//   RB    = relay bitmask 0-3 (unsigned). bit0=R1(40V), bit1=R2(24V)
//
// ROS node converts: tcm/10→cm, mV/1000→V, mA/1000→A, adc/1023→ratio
// =============================================================================

// ======================== CALIBRATION — SET THESE ========================

// Voltage divider full-scale: ADC * VREF_MV * RATIO / 1023
// For 24V with R1=30k,R2=7.5k → ratio=5 → full_scale = 5000*5 = 25000 mV
// For 19V with R1=20k,R2=10k  → ratio=3 → full_scale = 5000*3 = 15000 mV
#define VS1_FULL_SCALE_MV  25000L   // 24V battery — SET divider ratio
#define VS2_FULL_SCALE_MV  15000L   // 19V buck    — SET divider ratio

// ACS712 ELC-20A: sensitivity = 100 mV/A, zero-current output = VCC/2 = 2500 mV
// current_mA = (vout_mV - 2500) * 10
#define ACS_ZERO_MV   2500L
#define ACS_SCALE     10L    // 1000/sensitivity_mV = 1000/100 = 10

// ======================== PIN DEFINITIONS ========================

const uint8_t TRIG[] = {2, 4, 6, 8};   // Front, Rear, Left, Right
const uint8_t ECHO[] = {3, 5, 7, 9};
const uint8_t VS[]   = {A6, A7};
const uint8_t CS[]   = {A2, A3, A4, A5};
const uint8_t MQ_PIN   = A0;
const uint8_t MHMQ_PIN = A1;
const uint8_t RLY1_PIN = 10;
const uint8_t RLY2_PIN = 11;

#define RELAY_ON  LOW
#define RELAY_OFF HIGH

// ======================== TIMING ========================

#define BAUD          115200
#define INTERVAL_MS   50    // 20 Hz output

// ======================== STATE ========================

int16_t  us_tcm[4]   = {-10, -10, -10, -10};  // tenths-of-cm, -10 = no echo
uint8_t  us_idx       = 0;                      // round-robin sensor index
bool     rly1         = false;
bool     rly2         = false;
unsigned long prev_ms  = 0;
unsigned long last_ms  = 0;

// ======================== HELPERS ========================

// Append signed field: label + N zero-padded digits + 'p'/'n'
static uint8_t appendSigned(char* buf, const char* label, int32_t value, uint8_t width)
{
  uint8_t p = 0;
  while (*label) buf[p++] = *label++;

  int32_t av = value >= 0 ? value : -value;
  for (int8_t i = width - 1; i >= 0; i--) {
    buf[p + i] = '0' + (uint8_t)(av % 10);
    av /= 10;
  }
  p += width;
  buf[p++] = (value >= 0) ? 'p' : 'n';
  return p;
}

// Append unsigned field: label + N zero-padded digits
static uint8_t appendUnsigned(char* buf, const char* label, uint32_t value, uint8_t width)
{
  uint8_t p = 0;
  while (*label) buf[p++] = *label++;

  for (int8_t i = width - 1; i >= 0; i--) {
    buf[p + i] = '0' + (uint8_t)(value % 10);
    value /= 10;
  }
  p += width;
  return p;
}

// Ping one ultrasonic sensor (blocking, max 20 ms)
static int16_t pingUltrasonic(uint8_t idx)
{
  digitalWrite(TRIG[idx], LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG[idx], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG[idx], LOW);

  long dur = pulseIn(ECHO[idx], HIGH, 20000UL);
  if (dur <= 0) return -10;  // sentinel: -10 tcm = -1.0 cm

  // distance_tcm = duration_us * 1715 / 10000
  return (int16_t)((dur * 1715L) / 10000L);
}

// Read voltage in millivolts (using integer math)
static int32_t readVoltageMv(uint8_t pin, long fullScaleMv)
{
  return (long)analogRead(pin) * fullScaleMv / 1023L;
}

// Read ACS712 current in milliamps
static int32_t readCurrentMa(uint8_t pin)
{
  long vout_mv = (long)analogRead(pin) * 5000L / 1023L;
  return (vout_mv - ACS_ZERO_MV) * ACS_SCALE;
}

// ======================== SETUP ========================

void setup()
{
  Serial.begin(BAUD);

  for (uint8_t i = 0; i < 4; i++) {
    pinMode(TRIG[i], OUTPUT);
    pinMode(ECHO[i], INPUT);
    digitalWrite(TRIG[i], LOW);
  }

  pinMode(RLY1_PIN, OUTPUT);
  pinMode(RLY2_PIN, OUTPUT);
  digitalWrite(RLY1_PIN, RELAY_OFF);
  digitalWrite(RLY2_PIN, RELAY_OFF);

  Serial.println(F("========================================"));
  Serial.println(F("   Grace Arduino Nano Interface - READY"));
  Serial.println(F("   Baud: 115200  Rate: 20 Hz"));
  Serial.println(F("   Relay: 0=off 1=40V 2=24V 3=both"));
  Serial.println(F("========================================"));

  last_ms = millis();
}

// ======================== MAIN LOOP ========================

void loop()
{
  // ---- Handle relay commands ----
  while (Serial.available() > 0) {
    char cmd = Serial.read();
    switch (cmd) {
      case '0':
        rly1 = false; rly2 = false;
        digitalWrite(RLY1_PIN, RELAY_OFF);
        digitalWrite(RLY2_PIN, RELAY_OFF);
        break;
      case '1':
        rly1 = true;  rly2 = false;
        digitalWrite(RLY1_PIN, RELAY_ON);
        digitalWrite(RLY2_PIN, RELAY_OFF);
        break;
      case '2':
        rly1 = false; rly2 = true;
        digitalWrite(RLY1_PIN, RELAY_OFF);
        digitalWrite(RLY2_PIN, RELAY_ON);
        break;
      case '3':
        rly1 = true;  rly2 = true;
        digitalWrite(RLY1_PIN, RELAY_ON);
        digitalWrite(RLY2_PIN, RELAY_ON);
        break;
    }
  }

  // ---- Round-robin ultrasonic (one per cycle) ----
  us_tcm[us_idx] = pingUltrasonic(us_idx);
  us_idx = (us_idx + 1) & 0x03;

  // ---- Timed output ----
  unsigned long now = millis();
  if (now - prev_ms < INTERVAL_MS) return;

  uint32_t dt = (uint32_t)(now - last_ms);
  last_ms  = now;
  prev_ms  = now;

  // Read sensors
  int32_t v1  = readVoltageMv(VS[0], VS1_FULL_SCALE_MV);
  int32_t v2  = readVoltageMv(VS[1], VS2_FULL_SCALE_MV);
  int32_t c1  = readCurrentMa(CS[0]);
  int32_t c2  = readCurrentMa(CS[1]);
  int32_t c3  = readCurrentMa(CS[2]);
  int32_t c4  = readCurrentMa(CS[3]);
  uint16_t mq  = analogRead(MQ_PIN);
  uint16_t m2  = analogRead(MHMQ_PIN);
  uint8_t  rb  = (rly1 ? 1 : 0) | (rly2 ? 2 : 0);

  // ---- Build fixed-width machine line (115 chars + \n) ----
  char line[128];
  uint8_t p = 0;

  p += appendUnsigned(line + p, ",T",  dt, 4);
  p += appendSigned  (line + p, ",Uf", (int32_t)us_tcm[0], 5);
  p += appendSigned  (line + p, ",Ub", (int32_t)us_tcm[1], 5);
  p += appendSigned  (line + p, ",Ul", (int32_t)us_tcm[2], 5);
  p += appendSigned  (line + p, ",Ur", (int32_t)us_tcm[3], 5);
  p += appendSigned  (line + p, ",V1", v1, 5);
  p += appendSigned  (line + p, ",V2", v2, 5);
  p += appendSigned  (line + p, ",C1", c1, 5);
  p += appendSigned  (line + p, ",C2", c2, 5);
  p += appendSigned  (line + p, ",C3", c3, 5);
  p += appendSigned  (line + p, ",C4", c4, 5);
  p += appendUnsigned(line + p, ",MQ", mq, 4);
  p += appendUnsigned(line + p, ",M2", m2, 4);
  p += appendUnsigned(line + p, ",RB", rb, 1);
  line[p] = '\0';

  Serial.println(line);
}
