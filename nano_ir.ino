/**
 * ============================================================
 *  IR PIT / STEP DETECTOR  —  Arduino Nano (I2C Slave)
 * ============================================================
 *  IR Sensor Logic (e.g. TCRT5000 module with comparator):
 *    OUTPUT = 0 (LOW)  → object/ground detected → SAFE
 *    OUTPUT = 1 (HIGH) → nothing detected       → PIT / STEP → STOP
 *
 *  Stop condition: ANY single sensor reads HIGH → stop motors.
 *
 *  Pin Map:
 *    D2 → Front-Left  IR (FL)
 *    D3 → Front-Right IR (FR)
 *    D4 → Rear-Left   IR (BL)
 *    D5 → Rear-Right  IR (BR)
 *
 *  I2C: Slave at address 0x08
 *    SDA → A4,  SCL → A5
 *
 *  Protocol — Master reads 2 bytes:
 *    Byte 0 (STATUS):
 *      Bit 0 = FL pit  (1 = no ground = STOP)
 *      Bit 1 = FR pit
 *      Bit 2 = BL pit
 *      Bit 3 = BR pit
 *      Bit 7 = ANY_DANGER (1 if any sensor sees no ground)
 *    Byte 1 (RAW): same bits, no danger flag
 * ============================================================
 */

#include <Wire.h>

// ── Pins ─────────────────────────────────────────────────────
#define PIN_IR_FL  2
#define PIN_IR_FR  3
#define PIN_IR_BL  4
#define PIN_IR_BR  5

// ── I2C ──────────────────────────────────────────────────────
#define I2C_SLAVE_ADDR  0x08

// ── Timing ───────────────────────────────────────────────────
#define READ_INTERVAL_MS  20
#define DEBOUNCE_COUNT     3    // must trigger N consecutive reads to count

// ── State ────────────────────────────────────────────────────
volatile uint8_t g_status_byte = 0x00;
volatile uint8_t g_raw_byte    = 0x00;
uint8_t debounce[4] = {0, 0, 0, 0};
unsigned long lastReadTime = 0;

// ── I2C request handler ───────────────────────────────────────
void onI2CRequest() {
  Wire.write(g_status_byte);
  Wire.write(g_raw_byte);
}

// ── Setup ────────────────────────────────────────────────────
void setup() {
  pinMode(PIN_IR_FL, INPUT);
  pinMode(PIN_IR_FR, INPUT);
  pinMode(PIN_IR_BL, INPUT);
  pinMode(PIN_IR_BR, INPUT);

  Wire.begin(I2C_SLAVE_ADDR);
  Wire.onRequest(onI2CRequest);

  Serial.begin(115200);
  Serial.println(F("[IR Detector] Ready. 0=safe, 1=pit."));
}

// ── Read one IR sensor ────────────────────────────────────────
/**
 * Returns true if the sensor sees a PIT (no ground).
 *
 * Your IR module:
 *   0 (LOW)  = object detected = ground present = safe  → return false
 *   1 (HIGH) = nothing below   = pit / edge     = STOP  → return true
 */
bool readIR(uint8_t pin) {
  return digitalRead(pin) == HIGH;   // HIGH = no object = PIT
}

// ── Debounce ─────────────────────────────────────────────────
bool debounceRead(uint8_t idx, bool rawPit) {
  if (rawPit) {
    if (debounce[idx] < DEBOUNCE_COUNT) debounce[idx]++;
  } else {
    if (debounce[idx] > 0) debounce[idx]--;
  }
  return (debounce[idx] >= DEBOUNCE_COUNT);
}

// ── Build I2C bytes ───────────────────────────────────────────
void buildBytes(bool pit[4]) {
  uint8_t raw  = 0;
  uint8_t stat = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (pit[i]) {
      raw  |= (1 << i);
      stat |= (1 << i);
    }
  }
  if (raw != 0) stat |= 0x80;   // Bit 7 = ANY sensor saw a pit
  g_raw_byte    = raw;
  g_status_byte = stat;
}

// ── Loop ─────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();
  if (now - lastReadTime < READ_INTERVAL_MS) return;
  lastReadTime = now;

  bool pit[4];
  uint8_t pins[4] = {PIN_IR_FL, PIN_IR_FR, PIN_IR_BL, PIN_IR_BR};

  for (uint8_t i = 0; i < 4; i++) {
    pit[i] = debounceRead(i, readIR(pins[i]));
  }

  buildBytes(pit);

  if (g_raw_byte) {
    Serial.print(F("[PIT] FL:"));  Serial.print(pit[0]);
    Serial.print(F(" FR:"));       Serial.print(pit[1]);
    Serial.print(F(" BL:"));       Serial.print(pit[2]);
    Serial.print(F(" BR:"));       Serial.println(pit[3]);
  }
}
