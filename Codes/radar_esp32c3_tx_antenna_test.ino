/*
 * Radar TX Antenna Test (ESP32-C3) — Single TX Channel Bring-Up
 * =================================================================
 * Purpose-built bench firmware for testing one transmitter antenna
 * at a time. Auto-inits the ADAR2001 on boot in SPI-direct mode with
 * TX1 active for 16-20 GHz mid-band output. Single-character serial
 * commands switch between TX1 / TX2 / TX3 / TX4.
 *
 * SPI behavior on the wire is byte-for-byte identical to the
 * validated radar_esp32c3_controller.ino — same 24-bit framing, same
 * bias / multiplier / splitter / channel register values, same
 * Mode 0 / 4 MHz timing. The differences vs the validated firmware:
 *   - Auto-runs SPI verify + init at boot (no need to send 'I')
 *   - TX-only: drops 'w'/'x' RX commands and CS_RX (this PCB has no
 *     ADAR2004), so spiWrite/spiRead operate on PIN_CS_TX directly
 *   - Drops state-machine programming from initChip — pure SPI-direct
 *     mode, since this test never pulses TxADV/MADV/etc.
 *   - Smaller command surface: digit keys 0-4 set the channel
 *
 * Workflow:
 *   1. Wire ESP32-C3 to PCB per the standard pin map.
 *   2. Apply 4.5 GHz CW at -20 dBm to J6 (RFIN).
 *   3. Connect SM200B to J2 (RFOUT1) for the first test.
 *   4. Power up. Firmware auto-inits and prints status.
 *   5. Look for 18 GHz fundamental on SM200B at ~+2 dBm.
 *   6. To test antennas 2/3/4: move SMA cable to J3/J4/J5 and send
 *      '2' / '3' / '4' over serial. Look for 18 GHz at the new port.
 *
 * PIN MAPPING (ESP32-C3 -> PCB J1 logic header):
 *   GPIO  6 = SCK  (FSPI)         -> ADAR SCLK   (J1 pin 6)
 *   GPIO  7 = MOSI (FSPI)         -> ADAR SDIO   (J1 pin 7)
 *   GPIO  5 = MISO (FSPI)         <- ADAR SDO    (J1 pin 8)
 *   GPIO  4 = CS_TX               -> ADAR2001 CS (J1 pin 5)
 *   GPIO  0 = TxADV               -> ADAR2001 TxADV (J1 pin 1)
 *   GPIO  1 = TxRST               -> ADAR2001 TxRST (J1 pin 2)
 *   GPIO  3 = MADV                -> ADAR2001 MADV  (J1 pin 3)
 *   GPIO 10 = MRST                -> ADAR2001 MRST  (J1 pin 4)
 *   GPIO 18 = GPIO_DIR            -> 74AXP2T45 DIR  (J1 pin 9)
 *
 * State-machine pins are unused in SPI-direct mode but kept defined
 * so they boot to a known LOW state. GPIO 8 (CS_RX on the controller
 * firmware) is left as input — no ADAR2004 on this PCB.
 *
 * Power: 5 V -> J7 pin 1, ESP32-C3 3V3 -> J1 pin 10, common GND.
 *
 * Serial commands (115200 baud):
 *   1, 2, 3, 4   - Activate that TX channel (others off)
 *   0            - Disable all PAs (multiplier and splitters stay on)
 *   I            - Re-run full chip init from scratch
 *   V            - SPI scratchpad verify (write 0xA5 / 0x5A, read back)
 *   R            - Read back current TX register state
 *   ?            - Help
 */

#include <SPI.h>

// FSPI bus
#define PIN_SCK        6
#define PIN_MISO       5
#define PIN_MOSI       7

// CS for ADAR2001
#define PIN_CS_TX      4

// State-machine pins (kept for safe boot state, unused in SPI-direct)
#define PIN_TxADV      0
#define PIN_TxRST      1
#define PIN_MADV       3
#define PIN_MRST      10

// Level-shifter direction control
#define PIN_GPIO_DIR  18

#define SPI_SPEED        4000000
#define MULT_EN_16_20    0x6E
#define MULT_PASS_16_20  0x9F

// =============== SPI ===============

void spiWrite(uint16_t addr, uint8_t data) {
  uint8_t b0 = (addr >> 8) & 0x7F;
  uint8_t b1 = addr & 0xFF;
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS_TX, LOW);
  SPI.transfer(b0); SPI.transfer(b1); SPI.transfer(data);
  digitalWrite(PIN_CS_TX, HIGH);
  SPI.endTransaction();
}

uint8_t spiRead(uint16_t addr) {
  uint8_t b0 = 0x80 | ((addr >> 8) & 0x7F);
  uint8_t b1 = addr & 0xFF;
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS_TX, LOW);
  SPI.transfer(b0); SPI.transfer(b1);
  uint8_t r = SPI.transfer(0x00);
  digitalWrite(PIN_CS_TX, HIGH);
  SPI.endTransaction();
  return r;
}

// =============== Verify ===============

bool verifySPI() {
  Serial.println(F("--- SPI Verification ---"));
  spiWrite(0x00A, 0xA5); delay(1);
  uint8_t r1 = spiRead(0x00A);
  bool ok1 = (r1 == 0xA5);
  Serial.print(F("  wrote 0xA5, read 0x")); Serial.print(r1, HEX);
  Serial.println(ok1 ? F(" PASS") : F(" FAIL"));

  spiWrite(0x00A, 0x5A); delay(1);
  uint8_t r2 = spiRead(0x00A);
  bool ok2 = (r2 == 0x5A);
  Serial.print(F("  wrote 0x5A, read 0x")); Serial.print(r2, HEX);
  Serial.println(ok2 ? F(" PASS") : F(" FAIL"));

  return ok1 && ok2;
}

// =============== Init ===============

void initChip() {
  Serial.println(F("--- Init ADAR2001 (SPI-direct, 16-20 GHz) ---"));

  // Soft reset (defensive)
  spiWrite(0x000, 0x81); delay(10);
  spiWrite(0x000, 0x18);

  // Power on
  spiWrite(0x010, 0x01);

  // Bias (datasheet defaults, defensive writes)
  spiWrite(0x011, 0xBB);
  spiWrite(0x012, 0x0B);
  spiWrite(0x013, 0x75);
  spiWrite(0x014, 0xB5);
  spiWrite(0x015, 0x0C);

  // Sequencers off, latch bypass on
  spiWrite(0x016, 0x10);
  spiWrite(0x018, 0x10);

  // Multiplier path: 4-5 GHz in -> 16-20 GHz out, mid-band
  spiWrite(0x047, MULT_EN_16_20);
  spiWrite(0x048, MULT_PASS_16_20);

  // Splitters all on (matches validated firmware)
  spiWrite(0x046, 0x07);

  // TX1 active by default
  spiWrite(0x045, 0xC0);

  Serial.println(F("  Init complete. TX1 active."));
  Serial.println(F("  Expect Vin current ~190 mA."));
  Serial.println(F("  Apply 4.5 GHz CW @ -20 dBm to J6 (RFIN)."));
  Serial.println(F("  Look for 18 GHz at ~+2 dBm on RFOUT1 (J2)."));
}

// =============== Channel switching ===============

void setChannel(uint8_t ch) {
  uint8_t val;
  const __FlashStringHelper *name;
  switch (ch) {
    case 0: val = 0x00; name = F("all OFF (multiplier still on)"); break;
    case 1: val = 0xC0; name = F("TX1 -> RFOUT1 (J2)");             break;
    case 2: val = 0x30; name = F("TX2 -> RFOUT2 (J3)");             break;
    case 3: val = 0x0C; name = F("TX3 -> RFOUT3 (J4)");             break;
    case 4: val = 0x03; name = F("TX4 -> RFOUT4 (J5)");             break;
    default: Serial.println(F("ERR: ch must be 0-4")); return;
  }
  spiWrite(0x045, val);
  spiWrite(0x046, 0x07);   // splitters all on (defensive, matches validated)
  delayMicroseconds(50);    // PA settle

  // Read back to confirm
  uint8_t got = spiRead(0x045);
  Serial.print(F("  TX_EN1 = 0x")); Serial.print(got, HEX);
  Serial.print(F("  -> ")); Serial.println(name);
  if (got != val) Serial.println(F("  WARN: readback mismatch — check latch bypass"));
}

// =============== Readback ===============

void readBack() {
  Serial.println(F("--- TX register readback ---"));
  Serial.print(F("  PWRON          0x010 = 0x")); Serial.println(spiRead(0x010), HEX);
  Serial.print(F("  TX_SEQ_SETUP   0x016 = 0x")); Serial.println(spiRead(0x016), HEX);
  Serial.print(F("  MULT_SEQ_SETUP 0x018 = 0x")); Serial.println(spiRead(0x018), HEX);
  Serial.print(F("  TX_EN1         0x045 = 0x")); Serial.println(spiRead(0x045), HEX);
  Serial.print(F("  TX_EN2         0x046 = 0x")); Serial.println(spiRead(0x046), HEX);
  Serial.print(F("  MULT_EN        0x047 = 0x")); Serial.println(spiRead(0x047), HEX);
  Serial.print(F("  MULT_PASS      0x048 = 0x")); Serial.println(spiRead(0x048), HEX);
}

// =============== Serial handler ===============

void processCommand() {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);

  switch (c) {
    case '0': case '1': case '2': case '3': case '4':
      setChannel(c - '0');
      break;
    case 'I': case 'i':
      initChip();
      break;
    case 'V': case 'v':
      verifySPI();
      break;
    case 'R': case 'r':
      readBack();
      break;
    case '?':
      Serial.println(F("Commands: 1/2/3/4 = activate TX channel, 0 = all off"));
      Serial.println(F("          I = re-init, V = SPI verify, R = readback"));
      break;
    default:
      Serial.print(F("Unknown: ")); Serial.println(cmd);
  }
}

// =============== Setup ===============

void setup() {
  Serial.begin(115200);
  delay(200);

  // Pin modes - safe boot state
  pinMode(PIN_CS_TX,    OUTPUT);
  pinMode(PIN_TxADV,    OUTPUT);
  pinMode(PIN_TxRST,    OUTPUT);
  pinMode(PIN_MADV,     OUTPUT);
  pinMode(PIN_MRST,     OUTPUT);
  pinMode(PIN_GPIO_DIR, OUTPUT);

  digitalWrite(PIN_CS_TX,    HIGH);
  digitalWrite(PIN_TxADV,    LOW);
  digitalWrite(PIN_TxRST,    LOW);
  digitalWrite(PIN_MADV,     LOW);
  digitalWrite(PIN_MRST,     LOW);
  digitalWrite(PIN_GPIO_DIR, HIGH);   // 4-wire SPI write direction

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  delay(50);  // Let chip POR settle

  Serial.println();
  Serial.println(F("=== Radar TX Antenna Test (ESP32-C3) ==="));
  Serial.println(F("Single-channel SPI-direct testing for 16-20 GHz"));
  Serial.println();

  // Auto-run verify and init at startup
  if (verifySPI()) {
    initChip();
  } else {
    Serial.println(F("SPI verify FAILED. Check wiring, GPIO_DIR polarity, power."));
    Serial.println(F("Send V to retry, or I to force init anyway."));
  }

  Serial.println();
  Serial.println(F("Send '?' for commands, 1/2/3/4 to switch channels."));
}

void loop() {
  if (Serial.available()) processCommand();
}
