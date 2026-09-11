/*
 * Radar TX-as-Source Loopback Test (ESP32-C3)
 * =================================================================
 * Combined TX + RX bench test that uses the TX board's 18 GHz output
 * as the RF input to the RX board. Eliminates the need for a 20 GHz
 * signal generator — only two 4.5 GHz sources (SG396) needed.
 *
 * SIGNAL FLOW:
 *   SG396 #1 (4.500000 GHz, -20 dBm) -> TX J6 -> 4x mult -> 18.000 GHz
 *     out at TX J2 -> 20 dB pad -> RX J2 (RFIN1) -> mixer
 *   SG396 #2 (4.500250 GHz, -20 dBm) -> RX J6 -> 4x mult -> 18.001 GHz
 *     internal LO -> mixer
 *   IF = |18.001 GHz - 18.000 GHz| = 1 MHz at RX J7 IFOUT1+/-
 *
 * Both chips configured in SPI-direct mode for the mid-band 16-20 GHz
 * multiplier path, single channel active each side (TX1 and RX1).
 *
 * This firmware programs both chips in a single auto-init at boot.
 * Behavior on the SPI wire is byte-for-byte identical to the
 * validated initADAR2001() and initADAR2004() routines in
 * radar_arduino_controller.ino.
 *
 * PIN MAPPING (ESP32-C3 -> shared between TX J1 and RX J1):
 *
 *   ESP32 GPIO   Signal           TX J1 pin   RX J1 pin
 *   ----------   --------------   ---------   ---------
 *   GPIO  0      TxADV/RxADV          1           3
 *   GPIO  1      TxRST/RxRST          2           4
 *   GPIO  3      MADV (shared)        3           1
 *   GPIO 10      MRST (shared)        4           2
 *   GPIO  6      SCK  (shared)        6           5
 *   GPIO  7      MOSI (shared)        7           8
 *   GPIO  5      MISO (shared)        8           9
 *   GPIO 18      GPIO_DIR (shared)    9           7
 *   GPIO  4      CS_TX                5          (n/c)
 *   GPIO  8      CS_RX               (n/c)        6
 *   3V3 pin      VLogic              10          10
 *   GND pin      GND                 12          12
 *
 * Eight signals fan out from one ESP32 GPIO to both J1 headers
 * (use a breadboard rail). Two unique signals (CS_TX, CS_RX) go to
 * one J1 only. Power: PSU 5V in parallel to TX J7 and RX J9 Vin
 * pins; PSU GND in parallel to both ground pins.
 *
 * RF cabling:
 *   SG396 #1 -> TX J6 (LO for TX side)
 *   SG396 #2 -> RX J6 (LO for RX side, set 250 kHz higher than #1)
 *   TX J2 -> 20 dB SMA pad -> RX J2 (RF feed)
 *   RX J7 pin 1 (IFOUT1_P) -> USB-6366 AI 0 BNC center
 *   RX J7 pin 2 or 8 (GND) -> USB-6366 AI 0 BNC shell
 *   USB-6366 AI 0 FS/GS switch: GS (single-ended, simplest)
 *
 * Serial commands (115200 baud):
 *   I    - Re-run full init for both chips
 *   V    - SPI verify on both TX and RX
 *   R    - Read back current TX and RX register state
 *   t N  - Set TX channel (1-4): N=1 -> J2, N=2 -> J3, etc.
 *   r N  - Set RX channel (1-4): N=1 -> J2, N=2 -> J3, etc.
 *   ?    - Help
 */

#include <SPI.h>

// FSPI bus
#define PIN_SCK        6
#define PIN_MISO       5
#define PIN_MOSI       7

// CS lines
#define PIN_CS_TX      4
#define PIN_CS_RX      8

// State-machine pins (unused in SPI-direct, but kept LOW for safety)
#define PIN_TxADV_RxADV  0
#define PIN_TxRST_RxRST  1
#define PIN_MADV         3
#define PIN_MRST        10

// Level-shifter direction control
#define PIN_GPIO_DIR    18

#define SPI_SPEED        4000000

// =============== SPI ===============

void spiWrite(uint8_t csPin, uint16_t addr, uint8_t data) {
  uint8_t b0 = (addr >> 8) & 0x7F;
  uint8_t b1 = addr & 0xFF;
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(b0); SPI.transfer(b1); SPI.transfer(data);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

uint8_t spiRead(uint8_t csPin, uint16_t addr) {
  uint8_t b0 = 0x80 | ((addr >> 8) & 0x7F);
  uint8_t b1 = addr & 0xFF;
  SPI.beginTransaction(SPISettings(SPI_SPEED, MSBFIRST, SPI_MODE0));
  digitalWrite(csPin, LOW);
  SPI.transfer(b0); SPI.transfer(b1);
  uint8_t r = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return r;
}

// =============== Verify ===============

bool verifySPI() {
  Serial.println(F("--- SPI Verification ---"));
  spiWrite(PIN_CS_TX, 0x00A, 0xA5); delay(1);
  uint8_t rt = spiRead(PIN_CS_TX, 0x00A);
  bool tx_ok = (rt == 0xA5);
  Serial.print(F("  TX: wrote 0xA5, read 0x")); Serial.print(rt, HEX);
  Serial.println(tx_ok ? F(" PASS") : F(" FAIL"));

  spiWrite(PIN_CS_RX, 0x00A, 0x5A); delay(1);
  uint8_t rr = spiRead(PIN_CS_RX, 0x00A);
  bool rx_ok = (rr == 0x5A);
  Serial.print(F("  RX: wrote 0x5A, read 0x")); Serial.print(rr, HEX);
  Serial.println(rx_ok ? F(" PASS") : F(" FAIL"));

  return tx_ok && rx_ok;
}

// =============== Init: TX (ADAR2001, mid-band, TX1 active) ===============

void initTX() {
  Serial.println(F("--- Init TX (ADAR2001, SPI-direct, TX1 active) ---"));
  spiWrite(PIN_CS_TX, 0x000, 0x81); delay(10);
  spiWrite(PIN_CS_TX, 0x000, 0x18);
  spiWrite(PIN_CS_TX, 0x010, 0x01);
  // Bias (ADAR2001 datasheet defaults)
  spiWrite(PIN_CS_TX, 0x011, 0xBB);
  spiWrite(PIN_CS_TX, 0x012, 0x0B);
  spiWrite(PIN_CS_TX, 0x013, 0x75);
  spiWrite(PIN_CS_TX, 0x014, 0xB5);
  spiWrite(PIN_CS_TX, 0x015, 0x0C);
  // Sequencers off, latch bypass on
  spiWrite(PIN_CS_TX, 0x016, 0x10);
  spiWrite(PIN_CS_TX, 0x018, 0x10);
  // Multiplier mid-band 16-20 GHz
  spiWrite(PIN_CS_TX, 0x047, 0x6E);
  spiWrite(PIN_CS_TX, 0x048, 0x9F);
  // TX1 active, splitters all on
  spiWrite(PIN_CS_TX, 0x045, 0xC0);
  spiWrite(PIN_CS_TX, 0x046, 0x07);
  Serial.println(F("  TX1 active. Expect 18 GHz at J2 (~-3 dBm)."));
}

// =============== Init: RX (ADAR2004, mid-band, RX1 active) ===============

void initRX() {
  Serial.println(F("--- Init RX (ADAR2004, SPI-direct, RX1 active) ---"));
  spiWrite(PIN_CS_RX, 0x000, 0x81); delay(10);
  spiWrite(PIN_CS_RX, 0x000, 0x18);
  spiWrite(PIN_CS_RX, 0x010, 0x01);
  // Bias (ADAR2004 datasheet defaults — DIFFERENT from ADAR2001)
  spiWrite(PIN_CS_RX, 0x011, 0x55);
  spiWrite(PIN_CS_RX, 0x012, 0x07);
  spiWrite(PIN_CS_RX, 0x013, 0x78);
  spiWrite(PIN_CS_RX, 0x014, 0x7A);
  spiWrite(PIN_CS_RX, 0x015, 0x2A);
  spiWrite(PIN_CS_RX, 0x016, 0xC0);
  spiWrite(PIN_CS_RX, 0x017, 0x04);
  // Sequencers off
  spiWrite(PIN_CS_RX, 0x018, 0x00);
  spiWrite(PIN_CS_RX, 0x019, 0x00);
  // Multiplier mid-band 16-20 GHz
  spiWrite(PIN_CS_RX, 0x02F, 0xEE);
  // Splitters all on
  spiWrite(PIN_CS_RX, 0x02E, 0x07);
  // Gains all max
  spiWrite(PIN_CS_RX, 0x02C, 0x77);
  spiWrite(PIN_CS_RX, 0x02D, 0x77);
  // RX1 active: LNA + MIX + IFAMP + CH1
  spiWrite(PIN_CS_RX, 0x02B, 0xE8);
  Serial.println(F("  RX1 active. Expect 1 MHz IF at J7 IFOUT1+/-."));
}

// =============== Channel switching ===============

void setTXChannel(uint8_t ch) {
  uint8_t v;
  switch (ch) {
    case 1: v = 0xC0; break;
    case 2: v = 0x30; break;
    case 3: v = 0x0C; break;
    case 4: v = 0x03; break;
    default: Serial.println(F("ERR: TX ch must be 1-4")); return;
  }
  spiWrite(PIN_CS_TX, 0x045, v);
  spiWrite(PIN_CS_TX, 0x046, 0x07);
  delayMicroseconds(50);
  uint8_t got = spiRead(PIN_CS_TX, 0x045);
  Serial.print(F("  TX_EN1 = 0x")); Serial.print(got, HEX);
  Serial.print(F("  -> TX")); Serial.print(ch);
  Serial.println(F(" active"));
}

void setRXChannel(uint8_t ch) {
  uint8_t v;
  switch (ch) {
    case 1: v = 0xE8; break;
    case 2: v = 0xE4; break;
    case 3: v = 0xE2; break;
    case 4: v = 0xE1; break;
    default: Serial.println(F("ERR: RX ch must be 1-4")); return;
  }
  spiWrite(PIN_CS_RX, 0x02B, v);
  spiWrite(PIN_CS_RX, 0x02E, 0x07);
  delayMicroseconds(50);
  uint8_t got = spiRead(PIN_CS_RX, 0x02B);
  Serial.print(F("  RX_EN_SPI = 0x")); Serial.print(got, HEX);
  Serial.print(F("  -> RX")); Serial.print(ch);
  Serial.println(F(" active"));
}

// =============== Readback ===============

void readBack() {
  Serial.println(F("--- Register readback ---"));
  Serial.println(F("TX (ADAR2001):"));
  Serial.print(F("  PWRON      0x010 = 0x")); Serial.println(spiRead(PIN_CS_TX, 0x010), HEX);
  Serial.print(F("  TX_EN1     0x045 = 0x")); Serial.println(spiRead(PIN_CS_TX, 0x045), HEX);
  Serial.print(F("  TX_EN2     0x046 = 0x")); Serial.println(spiRead(PIN_CS_TX, 0x046), HEX);
  Serial.print(F("  MULT_EN    0x047 = 0x")); Serial.println(spiRead(PIN_CS_TX, 0x047), HEX);
  Serial.print(F("  MULT_PASS  0x048 = 0x")); Serial.println(spiRead(PIN_CS_TX, 0x048), HEX);
  Serial.println(F("RX (ADAR2004):"));
  Serial.print(F("  PWRON       0x010 = 0x")); Serial.println(spiRead(PIN_CS_RX, 0x010), HEX);
  Serial.print(F("  RX_EN_SPI   0x02B = 0x")); Serial.println(spiRead(PIN_CS_RX, 0x02B), HEX);
  Serial.print(F("  RX_GAIN12   0x02C = 0x")); Serial.println(spiRead(PIN_CS_RX, 0x02C), HEX);
  Serial.print(F("  RX_GAIN34   0x02D = 0x")); Serial.println(spiRead(PIN_CS_RX, 0x02D), HEX);
  Serial.print(F("  SPLT_EN_SPI 0x02E = 0x")); Serial.println(spiRead(PIN_CS_RX, 0x02E), HEX);
  Serial.print(F("  MULT_SPI    0x02F = 0x")); Serial.println(spiRead(PIN_CS_RX, 0x02F), HEX);
}

// =============== Serial handler ===============

void processCommand() {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);

  switch (c) {
    case 'I': case 'i':
      initTX(); initRX(); break;
    case 'V': case 'v':
      verifySPI(); break;
    case 'R':
      readBack(); break;
    case 't':
      if (cmd.length() > 2) setTXChannel(cmd.substring(2).toInt());
      else Serial.println(F("Usage: t N (N=1-4)"));
      break;
    case 'r':
      if (cmd.length() > 2) setRXChannel(cmd.substring(2).toInt());
      else Serial.println(F("Usage: r N (N=1-4)"));
      break;
    case '?':
      Serial.println(F("Commands:"));
      Serial.println(F("  I = re-init both chips"));
      Serial.println(F("  V = SPI verify"));
      Serial.println(F("  R = readback registers"));
      Serial.println(F("  t N = set TX channel (1-4)"));
      Serial.println(F("  r N = set RX channel (1-4)"));
      break;
    default:
      Serial.print(F("Unknown: ")); Serial.println(cmd);
  }
}

// =============== Setup ===============

void setup() {
  Serial.begin(115200);
  delay(3000);   // simple 3-sec delay (matches barebones_test.ino which works)

  pinMode(PIN_CS_TX,         OUTPUT);
  pinMode(PIN_CS_RX,         OUTPUT);
  pinMode(PIN_TxADV_RxADV,   OUTPUT);
  pinMode(PIN_TxRST_RxRST,   OUTPUT);
  pinMode(PIN_MADV,          OUTPUT);
  pinMode(PIN_MRST,          OUTPUT);
  pinMode(PIN_GPIO_DIR,      OUTPUT);

  digitalWrite(PIN_CS_TX,       HIGH);
  digitalWrite(PIN_CS_RX,       HIGH);
  digitalWrite(PIN_TxADV_RxADV, LOW);
  digitalWrite(PIN_TxRST_RxRST, LOW);
  digitalWrite(PIN_MADV,        LOW);
  digitalWrite(PIN_MRST,        LOW);
  digitalWrite(PIN_GPIO_DIR,    HIGH);

  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  delay(50);

  Serial.println();
  Serial.println(F("=== TX-as-Source Loopback Test (ESP32-C3) ==="));
  Serial.println(F("TX 18 GHz output -> 20 dB pad -> RX RFIN1"));
  Serial.println(F("SG396 #1 = 4.500000 GHz @ -20 dBm to TX J6"));
  Serial.println(F("SG396 #2 = 4.500250 GHz @ -20 dBm to RX J6"));
  Serial.println(F("Expected IF: 1 MHz at RX J7 IFOUT1+/-"));
  Serial.println();

  if (verifySPI()) {
    initTX(); initRX();
  } else {
    Serial.println(F("SPI verify FAILED. Check wiring, GPIO_DIR, power."));
  }

  Serial.println();
  Serial.println(F("Send '?' for commands."));
}

void loop() {
  if (Serial.available()) processCommand();
}
