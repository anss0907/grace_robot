#include <Wire.h>
#include <HTS221Sensor.h>
#include <LPS22HBSensor.h>
#include <LSM6DSLSensor.h>
#include <LIS3MDLSensor.h>
#include <VL53L0X.h>

// --- I2C2 on STM32L475E-IOT01A: SDA=PB11, SCL=PB10 ---
TwoWire I2C_2(PB11, PB10);

// Sensors
HTS221Sensor   *hts;
LPS22HBSensor  *lps;
LSM6DSLSensor  *imu;
LIS3MDLSensor  *mag;
VL53L0X         tof;

static uint16_t seq = 0;

// ------------ CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) ------------
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
  }
  return crc;
}

// ----------------------------- Helpers ---------------------------------
inline void le32(uint8_t* p, int32_t v) { p[0]= (uint8_t)(v); p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }
inline void le16(uint8_t* p, uint16_t v){ p[0]=(uint8_t)(v);   p[1]=(uint8_t)(v>>8); }

// ------------------------------ Setup ----------------------------------
void setup() {
  Serial.begin(921600);             // fast & reliable over USB CDC
  while (!Serial) {}

  I2C_2.begin();
  I2C_2.setClock(400000);
  delay(5);

  // Create sensors on I2C_2
  hts = new HTS221Sensor(&I2C_2);
  lps = new LPS22HBSensor(&I2C_2);
  imu = new LSM6DSLSensor(&I2C_2, LSM6DSL_ACC_GYRO_I2C_ADDRESS_LOW); // auto-detect was OK earlier
  mag = new LIS3MDLSensor(&I2C_2);

  // Init + enable (best-effort)
  hts->begin();
  lps->begin();
  imu->begin();
  mag->begin();
  imu->Enable_X();
  imu->Enable_G();
  mag->Enable();

  // ToF
  tof.setBus(&I2C_2);
  if (tof.init(true)) {
    tof.setTimeout(100);
    tof.startContinuous(50);
  }
}

// ------------------------------- Loop ----------------------------------
void loop() {
  // Read sensors (best-effort; on fail keep 0)
  float t_c=0, rh_pct=0, p_hpa=0;
  hts->GetTemperature(&t_c);
  hts->GetHumidity(&rh_pct);
  lps->GetPressure(&p_hpa);   // library returns hPa

  int32_t acc[3]={0,0,0}, gyr[3]={0,0,0}, magxyz[3]={0,0,0};
  imu->Get_X_Axes(acc);       // mg
  imu->Get_G_Axes(gyr);       // mdps
  mag->GetAxes(magxyz);       // mgauss

  int32_t dist_mm = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) dist_mm = -1;

  // Scale to int32 payload
  int32_t temp_mC      = (int32_t) lroundf(t_c * 1000.0f);
  int32_t hum_mpermil  = (int32_t) lroundf(rh_pct * 10.0f);     // 0–1000
  int32_t press_Pa     = (int32_t) lroundf(p_hpa * 100.0f);     // hPa -> Pa
  int32_t ax=acc[0], ay=acc[1], az=acc[2];                      // mg
  int32_t gx=gyr[0], gy=gyr[1], gz=gyr[2];                      // mdps
  int32_t mx=magxyz[0], my=magxyz[1], mz=magxyz[2];             // mgauss
  int32_t tf_mm = dist_mm;

  // Build payload (12 * int32 = 48 bytes)
  uint8_t payload[48];
  uint8_t* w = payload;
  le32(w+ 0, temp_mC);
  le32(w+ 4, hum_mpermil);
  le32(w+ 8, press_Pa);
  le32(w+12, ax); le32(w+16, ay); le32(w+20, az);
  le32(w+24, gx); le32(w+28, gy); le32(w+32, gz);
  le32(w+36, mx); le32(w+40, my); le32(w+44, mz);
  // ToF: we’ll append after core 12 fields by swapping one mag if you prefer all 13.
  // Keeping it inside: replace the last mag field with ToF, or expand payload & LEN.
  // Here we EXTEND to also include ToF (int32) => 52 bytes payload:
  uint8_t payload2[52];
  memcpy(payload2, payload, 48);
  le32(payload2+48, tf_mm);
  const uint16_t payload_len = 52;

  // Build header
  uint8_t hdr[2+1+1+2+4+2]; // SOF(2)+VER(1)+ID(1)+SEQ(2)+TS(4)+LEN(2) = 12
  hdr[0]=0xAA; hdr[1]=0x55;
  hdr[2]=0x01;              // version
  hdr[3]=0x01;              // msg id
  le16(hdr+4, seq++);       // seq
  uint32_t ts = millis();
  hdr[6] = (uint8_t)(ts); hdr[7]=(uint8_t)(ts>>8); hdr[8]=(uint8_t)(ts>>16); hdr[9]=(uint8_t)(ts>>24);
  le16(hdr+10, payload_len);

  // CRC over [hdr || payload2]
  uint16_t crc = 0xFFFF;
  crc = crc16_ccitt(hdr, sizeof(hdr), crc);
  crc = crc16_ccitt(payload2, payload_len, crc);
  uint8_t crc_le[2]; le16(crc_le, crc);

  // Send frame
  Serial.write(hdr, sizeof(hdr));
  Serial.write(payload2, payload_len);
  Serial.write(crc_le, 2);

  // target loop rate
  delay(10); // ~100 Hz; adjust as needed
}
