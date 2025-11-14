// --- FIX for Arduino auto-prototype issue ---
struct SlotState;  // forward-declaration so prototypes can see the type

/*
  Arduino Nano + RFM95 (SX1276) + 2x VL53L0X
  - Two ToF sensors with simple occupancy logic
  - LoRaWAN (OTAA) uplink using MCCI LMIC (EU868 set in lmic_project_config.h)

  Wiring (Nano):
    I2C:    SDA=A4, SCL=A5
    XSHUT:  LEFT=D8, RIGHT=D9
    LoRa SPI: MOSI=D11, MISO=D12, SCK=D13
    LoRa:   nSS=D10, RST=D7, DIO0=D2, DIO1=D3, DIO2=D4

  Libraries:
    - VL53L0X (Pololu)
    - MCCI LoRaWAN LMIC
*/

#include <Wire.h>
#include <VL53L0X.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <string.h> // memset/memcpy helpers

// ====== VL53L0X hardware mapping ======
#define LEFT_XSHUT   8
#define RIGHT_XSHUT  9

// Unique I2C addresses (default: 0x29 -> assign new ones)
#define ADDR_LEFT   0x30
#define ADDR_RIGHT  0x31

// ====== Detection settings ======
const uint16_t MIN_DIST_MM = 140;   // 14 cm
const uint16_t MAX_DIST_MM = 200;   // 20 cm
const uint8_t  OCCUPY_CONSEC = 2;
const uint8_t  RELEASE_CONSEC = 3;
const unsigned long HOLD_MS = 4000; // ms
const uint16_t MAX_VALID_MM = 2000;

const uint32_t BUDGET_US = 80000;   // 80 ms
const uint16_t PERIOD_LEFT_MS  = 200;
const uint16_t PERIOD_RIGHT_MS = 230;

// ====== Per-slot state ======
struct SlotState {
  bool occupied = false;
  uint8_t hitStreak = 0;
  uint8_t missStreak = 0;
  unsigned long lastEvidenceMs = 0;
  int lastMm = -1;
  bool lastInRange = false;
};

static inline bool validReading(uint16_t mm) {
  return (mm != 0 && mm != 65535 && mm <= MAX_VALID_MM);
}

// ====== Globals ======
VL53L0X tofL, tofR;
SlotState leftS, rightS;

// ====== I2C sensor bring-up ======
void bringLowAllXshut() {
  pinMode(LEFT_XSHUT, OUTPUT);
  pinMode(RIGHT_XSHUT, OUTPUT);
  digitalWrite(LEFT_XSHUT, LOW);
  digitalWrite(RIGHT_XSHUT, LOW);
  delay(10);
}

void setupOne(VL53L0X& dev, uint8_t xshutPin, uint8_t newAddr) {
  pinMode(xshutPin, OUTPUT);
  digitalWrite(xshutPin, HIGH);  // enable this sensor only
  delay(10);

  dev.setTimeout(500);
  if (!dev.init()) {
    Serial.println(F("ERROR: VL53L0X init failed (XSHUT/I2C)!"));
    while (1) delay(1000);
  }
  dev.setAddress(newAddr);
  dev.setMeasurementTimingBudget(BUDGET_US);
  // dev.setSignalRateLimit(0.08f); // optional
}

// ====== LoRaWAN (OTAA) configuration ======
// NOTE (MCCI LMIC):
//   - DEVEUI and APPEUI (JoinEUI) must be given LSB-first here,
//   - APPKEY must be MSB-first (exactly as shown in the console).
// Example: console shows DEVEUI = 01-23-45-67-89-AB-CD-EF (MSB->LSB)
//          put it here as: { 0xEF,0xCD,0xAB,0x89,0x67,0x45,0x23,0x01 }

static const u1_t PROGMEM APPEUI[8] = {
  // JoinEUI LSB-first
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};
static const u1_t PROGMEM DEVEUI[8] = {
  // DevEUI LSB-first (replace with your own)
  0xEF,0xCD,0xAB,0x89,0x67,0x45,0x23,0x01
};
static const u1_t PROGMEM APPKEY[16] = {
  // AppKey MSB-first (exactly as shown in console)
  0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
  0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F
};

// Mandatory callbacks for MCCI LMIC (copy from PROGMEM to RAM)
extern "C" void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
extern "C" void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
extern "C" void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// LMIC pin mapping (Arduino Nano)
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 7,                // LoRa RST on D7 (kept free from sensor pins)
  .dio = {2, 3, 4}         // DIO0, DIO1, DIO2 (DIO2 optional)
};

// Send interval — example 3 minutes (TTN/TTS friendly)
const unsigned long TX_INTERVAL = 180UL * 1000UL; // 3 min
unsigned long lastSend = 0;

#ifndef QUIET_SERIAL
void onEvent(ev_t ev) {
  Serial.print(F("LMIC event: ")); Serial.println((unsigned)ev);
  switch (ev) {
    case EV_JOINING:
      Serial.println(F("Joining via OTAA..."));
      break;
    case EV_JOINED:
      Serial.println(F("OTAA join successful!"));
      // Good practice:
      LMIC_setLinkCheckMode(0); // disable LinkCheck to reduce overhead
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("OTAA join failed, retrying..."));
      break;
    default:
      break;
  }
}
#else
void onEvent(ev_t) { }
#endif

bool isJoined() {
  // In MCCI LMIC, devaddr stays 0 until joined
  return (LMIC.devaddr != 0);
}

void do_send() {
  if (!isJoined()) {
#ifndef QUIET_SERIAL
    Serial.println(F("Not joined (OTAA) yet, skipping send."));
#endif
    return;
  }
  if (LMIC.opmode & OP_TXRXPEND) {
#ifndef QUIET_SERIAL
    Serial.println(F("OP_TXRXPEND, delaying send"));
#endif
    return;
  }

  // payload: [total, leftOcc, rightOcc]
  uint8_t total = (leftS.occupied ? 1 : 0) + (rightS.occupied ? 1 : 0);
  uint8_t payload[3] = { total, (uint8_t)leftS.occupied, (uint8_t)rightS.occupied };

#ifndef QUIET_SERIAL
  Serial.print(F("Payload: total="));
  Serial.print(total);
  Serial.print(F(" L="));
  Serial.print((int)payload[1]);
  Serial.print(F(" R="));
  Serial.println((int)payload[2]);
#endif

  LMIC_setTxData2(/*port*/1, payload, sizeof(payload), /*confirmed*/0);
#ifndef QUIET_SERIAL
  Serial.println(F("TX scheduled"));
#endif
}

// ====== Sensor update ======
void updateSlot(VL53L0X& dev, SlotState& s) {
  uint16_t mm = dev.readRangeContinuousMillimeters();
  bool timeout = dev.timeoutOccurred();

  bool inRange = false;
  if (!timeout && validReading(mm)) {
    inRange = (mm >= MIN_DIST_MM && mm <= MAX_DIST_MM);
    s.lastMm = (int)mm;
  } else {
    s.lastMm = -1;
  }
  s.lastInRange = inRange;

  if (inRange) {
    s.hitStreak++;
    s.missStreak = 0;
    s.lastEvidenceMs = millis();
  } else {
    s.missStreak++;
    s.hitStreak = 0;
  }

  bool holdActive = (millis() - s.lastEvidenceMs) < HOLD_MS;

  if (!s.occupied) {
    if (inRange && s.hitStreak >= OCCUPY_CONSEC) {
      s.occupied = true;
    }
  } else {
    if (!holdActive && !inRange && s.missStreak >= RELEASE_CONSEC) {
      s.occupied = false;
    }
  }
}

void printSlot(const char* name, const SlotState& s) {
#ifndef QUIET_SERIAL
  Serial.print(name);
  Serial.print(F(": "));
  Serial.print(s.occupied ? F("OCCUPIED") : F("FREE"));
  Serial.print(F("  dist="));
  if (s.lastMm < 0) Serial.print(F("-"));
  else { Serial.print(s.lastMm); Serial.print(F("mm")); }
  Serial.print(F("  inRange="));
  Serial.print(s.lastInRange ? F("1") : F("0"));
#endif
}

// ====== Setup ======
void setup() {
#ifndef QUIET_SERIAL
  Serial.begin(9600);
  delay(100);
  Serial.println(F("Bike counter + LoRaWAN OTAA (EU868)"));
#endif

  // I2C + sensors
  Wire.begin();
  bringLowAllXshut();
  setupOne(tofL, LEFT_XSHUT, ADDR_LEFT);
  setupOne(tofR, RIGHT_XSHUT, ADDR_RIGHT);
  tofL.startContinuous(PERIOD_LEFT_MS);
  tofR.startContinuous(PERIOD_RIGHT_MS);
#ifndef QUIET_SERIAL
  Serial.println(F("VL53L0X ready, addresses: LEFT=0x30, RIGHT=0x31"));
#endif

  // --- LMIC init ---
  os_init();
  LMIC_reset();

  // Region is set in lmic_project_config.h (EU868)
  LMIC_setLinkCheckMode(0);    // disable LinkCheck (lower overhead)
  LMIC_setAdrMode(1);          // enable ADR
  LMIC_setDrTxpow(DR_SF7, 14); // start at SF7, 14 dBm (network may adjust via ADR)
  // Optional: increase if you use an inaccurate RC oscillator:
  // LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // Kick off the OTAA join
  LMIC_startJoining();

  // Allow the first send attempt once joined
  lastSend = millis() - TX_INTERVAL;
}

// ====== Main loop ======
void loop() {
  // Drive LMIC
  os_runloop_once();

  // Update sensors
  updateSlot(tofL, leftS);
  updateSlot(tofR, rightS);

#ifndef QUIET_SERIAL
  int total = (leftS.occupied ? 1 : 0) + (rightS.occupied ? 1 : 0);
  printSlot("LEFT", leftS);
  Serial.print(F(" | "));
  printSlot("RIGHT", rightS);
  Serial.print(F(" | Total bikes: "));
  Serial.println(total);
#endif

  // Periodic LoRaWAN uplink
  unsigned long now = millis();
  if (now - lastSend >= TX_INTERVAL) {
    lastSend = now;
    do_send();
  }

  delay(40); // be kind to CPU/I2C
}
