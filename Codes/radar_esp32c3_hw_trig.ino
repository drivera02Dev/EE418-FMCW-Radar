/*
 * FMCW MIMO RADAR — ESP32-C3 Controller, HARDWARE TRIGGER Version
 * ===============================================================
 * 1:1 port of the bench-validated Arduino HW-trigger firmware to
 * ESP32-C3-DevKitC-02 v1.1. Behavior on the wire is byte-for-byte
 * identical: same 24-bit SPI framing, same pulse widths, same
 * register init, same MIMO frame loop, same TRIG pulse to USB-6356
 * PFI0.
 *
 * Differences from the Arduino HW-trigger version are limited to
 * platform glue:
 *   - ESP32-C3 GPIO numbers instead of Uno digital pins
 *   - SPI.begin(SCK, MISO, MOSI, -1) explicit-pin form (FSPI bus)
 *   - delay(200) at boot in place of while(!Serial)
 *   - PIN_GPIO_DIR driven HIGH (new PCB uses 74AXP2T45 level
 *     shifters that need a DIR signal; the old TXB0108 setup did not)
 *
 * Same SPI + state machine as software-sync version, but adds:
 *   GPIO 19 = TRIG_OUT -> USB-6356 PFI0 (rising edge starts DAQ capture)
 *
 * In this mode, ESP32-C3 controls the entire MIMO frame timing:
 *   1. MATLAB pre-arms the DAQ (ramp + capture waiting for trigger)
 *   2. MATLAB sends 'F' (do one full MIMO frame)
 *   3. ESP32-C3: reset -> MADV -> for each TX: TxADV -> TRIG pulse -> wait Tp
 *   4. ESP32-C3 replies "OK FRAME" when all 4 channels are done
 *   5. MATLAB reads the captured data from DAQ buffer
 *
 * Advantage: ~4 ms per frame (250 fps) vs ~44 ms (23 fps) with software sync
 * Cost: 1 extra wire (ESP32-C3 GPIO 19 -> USB-6356 PFI0)
 *
 * PIN MAPPING (ESP32-C3 -> PCB J1 logic header + DAQ trigger):
 *   GPIO  6 = SCK  (FSPI)         -> ADAR SCLK   (J1 pin 6)
 *   GPIO  7 = MOSI (FSPI)         -> ADAR SDIO   (J1 pin 7)
 *   GPIO  5 = MISO (FSPI)         <- ADAR SDO    (J1 pin 8)
 *   GPIO  4 = CS_TX               -> ADAR2001 CS (J1 pin 5)
 *   GPIO  8 = CS_RX               -> (TX-only board: not connected;
 *                                     onboard LED + strapping pin,
 *                                     stays HIGH so neither matters)
 *   GPIO  0 = TxADV               -> ADAR2001 TxADV (J1 pin 1)
 *   GPIO  1 = TxRST               -> ADAR2001 TxRST (J1 pin 2)
 *   GPIO  3 = MADV                -> ADAR2001 MADV  (J1 pin 3)
 *   GPIO 10 = MRST                -> ADAR2001 MRST  (J1 pin 4)
 *   GPIO 18 = GPIO_DIR            -> 74AXP2T45 DIR  (J1 pin 9)
 *   GPIO 19 = TRIG_OUT            -> USB-6356 PFI0 (direct wire)
 *
 * NOTE: GPIO 19 -> PFI0 does NOT need level shifting. PFI inputs on
 *       the USB-6356 are 5 V tolerant TTL with V_IH around 2.0 V, so
 *       the ESP32-C3's 3.3 V output drives them directly. Run a
 *       single jumper from GPIO 19 to PFI0, and tie ESP32-C3 GND to
 *       USB-6356 D GND so the trigger has a return path.
 *
 * AVOIDED PINS (ESP32-C3 specific):
 *   GPIO 11-17: reserved for internal SPI flash
 *   GPIO 20, 21: UART0 RX/TX (USB-UART for the serial monitor)
 *   GPIO 2, 9: strapping pins (boot-mode select, BOOT button)
 *
 * GPIO 18 and 19 are technically the C3's native USB D-/D+, but on
 * the DevKitC-02 v1.1 the USB connector is wired to the external
 * USB-UART chip (not the native USB), so these are free as GPIOs.
 *
 * SERIAL COMMANDS (same as software version, plus 'F' and 'T'):
 *   I         — Init both ICs
 *   V         — Verify SPI
 *   S         — Enable sequencers
 *   D         — Disable sequencers
 *   P         — Pulse TxADV (single advance)
 *   R         — Reset state machines
 *   M         — Pulse MADV
 *   F         — Full MIMO frame (4 channels, hardware-timed)
 *   F N       — N consecutive MIMO frames (e.g. "F 100" for 100 frames)
 *   C N       — Set channel directly (1-4, SPI mode)
 *   W/w/X/x   — Raw SPI write/read
 *   T N       — Set chirp duration in us (default 1000 = 1 ms)
 *   ?         — Help
 */

#include <SPI.h>

// ===================== PIN DEFINITIONS =====================
// VSPI bus pins (ESP32 Arduino core defaults)
#define PIN_SCK        6
#define PIN_MISO       5
#define PIN_MOSI       7

// CS lines
#define PIN_CS_TX      4
#define PIN_CS_RX      8

// State-machine control pins
#define PIN_TxADV      0
#define PIN_TxRST      1
#define PIN_MADV       3
#define PIN_MRST      10

// Level-shifter direction control (new on this PCB; not present in
// the original Arduino+TXB0108 setup). HIGH = A→B = MCU→ADAR for the
// 74AXP2T45.
#define PIN_GPIO_DIR  18

// Hardware trigger to USB-6356 PFI0 (direct, no level shift needed)
#define PIN_TRIG      19

#define SPI_SPEED  4000000
#define MULT_EN_16_20    0x6E
#define MULT_PASS_16_20  0x9F

// Configurable chirp duration (default 1 ms)
unsigned long chirpUs = 1000;

// ===================== CORE SPI =====================

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
  uint8_t result = SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return result;
}

// ===================== PULSE HELPERS =====================

void pulseTxADV() {
  digitalWrite(PIN_TxADV, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_TxADV, LOW);
  delayMicroseconds(50);   // PA settling
}

void pulseTxRST() {
  digitalWrite(PIN_TxRST, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_TxRST, LOW);
}

void pulseMADV() {
  digitalWrite(PIN_MADV, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_MADV, LOW);
  delayMicroseconds(100);
}

void pulseMRST() {
  digitalWrite(PIN_MRST, HIGH);
  delayMicroseconds(1);
  digitalWrite(PIN_MRST, LOW);
}

void pulseTrigger() {
  // Rising edge tells USB-6356 to start AO ramp + AI capture
  // Pulse width: 10 µs (DAQ only needs the edge, but a clean pulse helps)
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
}

// ===================== IC INIT =====================

bool verifySPI() {
  Serial.println(F("--- SPI Verification ---"));
  spiWrite(PIN_CS_TX, 0x00A, 0xA5); delay(1);
  uint8_t rd_tx = spiRead(PIN_CS_TX, 0x00A);
  bool tx_ok = (rd_tx == 0xA5);
  Serial.print(F("  TX: 0x")); Serial.print(rd_tx, HEX);
  Serial.println(tx_ok ? F(" PASS") : F(" FAIL"));

  spiWrite(PIN_CS_RX, 0x00A, 0x5A); delay(1);
  uint8_t rd_rx = spiRead(PIN_CS_RX, 0x00A);
  bool rx_ok = (rd_rx == 0x5A);
  Serial.print(F("  RX: 0x")); Serial.print(rd_rx, HEX);
  Serial.println(rx_ok ? F(" PASS") : F(" FAIL"));

  return tx_ok && rx_ok;
}

void initADAR2001() {
  Serial.println(F("--- Init ADAR2001 ---"));
  spiWrite(PIN_CS_TX, 0x000, 0x81); delay(10);
  spiWrite(PIN_CS_TX, 0x000, 0x18);
  spiWrite(PIN_CS_TX, 0x010, 0x01);
  // Bias (Table 9)
  spiWrite(PIN_CS_TX, 0x011, 0xBB);
  spiWrite(PIN_CS_TX, 0x012, 0x0B);
  spiWrite(PIN_CS_TX, 0x013, 0x75);
  spiWrite(PIN_CS_TX, 0x014, 0xB5);
  spiWrite(PIN_CS_TX, 0x015, 0x0C);
  // Sequencers off, latch bypass on
  spiWrite(PIN_CS_TX, 0x016, 0x10);
  spiWrite(PIN_CS_TX, 0x018, 0x10);
  // Multiplier 16-20 GHz
  spiWrite(PIN_CS_TX, 0x047, MULT_EN_16_20);
  spiWrite(PIN_CS_TX, 0x048, MULT_PASS_16_20);
  // TX CH1 active (SPI direct)
  spiWrite(PIN_CS_TX, 0x045, 0xC0);
  spiWrite(PIN_CS_TX, 0x046, 0x07);
  // Mult modes: 0=sleep, 1=16-20GHz
  spiWrite(PIN_CS_TX, 0x070, 0x00); spiWrite(PIN_CS_TX, 0x071, 0x00);
  spiWrite(PIN_CS_TX, 0x072, MULT_EN_16_20); spiWrite(PIN_CS_TX, 0x073, MULT_PASS_16_20);
  // TX modes: 0=sleep, 1=CH1, 2=CH2, 3=CH3, 4=CH4
  spiWrite(PIN_CS_TX, 0x050, 0x00); spiWrite(PIN_CS_TX, 0x051, 0x00);
  spiWrite(PIN_CS_TX, 0x052, 0xC0); spiWrite(PIN_CS_TX, 0x053, 0x07);
  spiWrite(PIN_CS_TX, 0x054, 0x30); spiWrite(PIN_CS_TX, 0x055, 0x07);
  spiWrite(PIN_CS_TX, 0x056, 0x0C); spiWrite(PIN_CS_TX, 0x057, 0x07);
  spiWrite(PIN_CS_TX, 0x058, 0x03); spiWrite(PIN_CS_TX, 0x059, 0x07);
  // State assignments
  spiWrite(PIN_CS_TX, 0x03C, 0x10);
  spiWrite(PIN_CS_TX, 0x019, 0x12);
  spiWrite(PIN_CS_TX, 0x01A, 0x34);
  Serial.println(F("  Done."));
}

void initADAR2004() {
  Serial.println(F("--- Init ADAR2004 ---"));
  spiWrite(PIN_CS_RX, 0x000, 0x81); delay(10);
  spiWrite(PIN_CS_RX, 0x000, 0x18);
  spiWrite(PIN_CS_RX, 0x010, 0x01);
  // ADAR2004 bias (DIFFERENT from ADAR2001)
  spiWrite(PIN_CS_RX, 0x011, 0x55);
  spiWrite(PIN_CS_RX, 0x012, 0x07);
  spiWrite(PIN_CS_RX, 0x013, 0x78);
  spiWrite(PIN_CS_RX, 0x014, 0x7A);
  spiWrite(PIN_CS_RX, 0x015, 0x2A);
  spiWrite(PIN_CS_RX, 0x016, 0xC0);
  spiWrite(PIN_CS_RX, 0x017, 0x04);
  spiWrite(PIN_CS_RX, 0x018, 0x00);
  spiWrite(PIN_CS_RX, 0x019, 0x00);
  spiWrite(PIN_CS_RX, 0x02F, 0xEE);
  spiWrite(PIN_CS_RX, 0x02B, 0xEF);
  spiWrite(PIN_CS_RX, 0x02E, 0x07);
  spiWrite(PIN_CS_RX, 0x02C, 0x77);
  spiWrite(PIN_CS_RX, 0x02D, 0x77);
  Serial.println(F("  Done."));
}

void enableSequencers() {
  spiWrite(PIN_CS_TX, 0x018, 0x90);
  spiWrite(PIN_CS_TX, 0x016, 0x90);
  spiWrite(PIN_CS_TX, 0x017, 0x03);
}

void disableSequencers() {
  spiWrite(PIN_CS_TX, 0x016, 0x10);
  spiWrite(PIN_CS_TX, 0x018, 0x10);
}

void setChannelDirect(uint8_t ch) {
  uint8_t val;
  switch (ch) {
    case 1: val = 0xC0; break; case 2: val = 0x30; break;
    case 3: val = 0x0C; break; case 4: val = 0x03; break;
    default: val = 0x00; break;
  }
  spiWrite(PIN_CS_TX, 0x045, val);
  spiWrite(PIN_CS_TX, 0x046, 0x07);
}

// ===================== MIMO FRAME (hardware-timed) =====================

void doMIMOFrame() {
  /*
   * Full MIMO frame with hardware triggers:
   *   1. Reset → sleep
   *   2. MADV → multiplier active
   *   3. For each TX (1-4):
   *      a. TxADV → PA active (50 µs settle)
   *      b. Pulse TRIG → DAQ starts ramp + capture
   *      c. Wait chirpUs for sweep to complete
   *
   * DAQ must be pre-armed (waiting for PFI0 rising edge) before 'F' is sent.
   * Each TRIG pulse starts one chirp's worth of AO + AI on the USB-6356.
   */

  // Reset
  pulseMRST();
  pulseTxRST();
  delayMicroseconds(10);

  // Multiplier on
  pulseMADV();
  // Extra settle on first activation (multiplier cold start)
  delayMicroseconds(200);

  // 4 TX channels
  for (uint8_t ch = 0; ch < 4; ch++) {
    pulseTxADV();             // Next TX channel (50 µs settle included)
    pulseTrigger();           // Rising edge → DAQ starts this chirp
    delayMicroseconds(chirpUs + 100);  // Wait for chirp + margin
  }
}

void doMultiFrame(uint16_t numFrames) {
  /*
   * Execute N consecutive MIMO frames at maximum rate.
   * DAQ must be pre-armed for N*4 triggered acquisitions.
   */
  for (uint16_t f = 0; f < numFrames; f++) {
    // Reset
    pulseMRST();
    pulseTxRST();
    delayMicroseconds(10);

    // Multiplier on
    pulseMADV();
    if (f == 0) delayMicroseconds(200);  // extra settle on first frame only
    else delayMicroseconds(50);

    // 4 TX channels
    for (uint8_t ch = 0; ch < 4; ch++) {
      pulseTxADV();
      pulseTrigger();
      delayMicroseconds(chirpUs + 100);
    }
  }
}

// ===================== SERIAL HANDLER =====================

void processCommand() {
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;
  char c = cmd.charAt(0);

  switch (c) {
    case 'I':
      initADAR2001(); initADAR2004();
      Serial.println(F("OK INIT"));
      break;
    case 'V':
      Serial.println(verifySPI() ? F("OK VERIFY") : F("FAIL VERIFY"));
      break;
    case 'S':
      enableSequencers();
      Serial.println(F("OK SEQ_ON"));
      break;
    case 'D':
      disableSequencers();
      Serial.println(F("OK SEQ_OFF"));
      break;
    case 'P':
      pulseTxADV();
      Serial.println(F("OK TxADV"));
      break;
    case 'R':
      pulseMRST(); pulseTxRST(); delayMicroseconds(10);
      Serial.println(F("OK RESET"));
      break;
    case 'M':
      pulseMADV();
      Serial.println(F("OK MADV"));
      break;
    case 'F': {
      // "F" = 1 frame, "F 100" = 100 frames
      uint16_t nf = 1;
      if (cmd.length() > 2) {
        nf = strtol(cmd.substring(2).c_str(), NULL, 10);
        if (nf < 1) nf = 1;
        if (nf > 10000) nf = 10000;
      }
      if (nf == 1) {
        doMIMOFrame();
        Serial.println(F("OK FRAME"));
      } else {
        doMultiFrame(nf);
        Serial.print(F("OK FRAMES ")); Serial.println(nf);
      }
      break;
    }
    case 'C': {
      uint8_t ch = cmd.substring(2).toInt();
      if (ch >= 1 && ch <= 4) {
        setChannelDirect(ch);
        Serial.print(F("OK CH")); Serial.println(ch);
      } else Serial.println(F("ERR CH 1-4"));
      break;
    }
    case 'T': {
      chirpUs = strtol(cmd.substring(2).c_str(), NULL, 10);
      if (chirpUs < 100) chirpUs = 100;
      if (chirpUs > 100000) chirpUs = 100000;
      Serial.print(F("OK CHIRP_US ")); Serial.println(chirpUs);
      break;
    }
    case 'W': {
      uint16_t a = strtol(cmd.substring(2,5).c_str(), NULL, 16);
      uint8_t  d = strtol(cmd.substring(6,8).c_str(), NULL, 16);
      spiWrite(PIN_CS_TX, a, d);
      Serial.print(F("OK WR_TX 0x")); Serial.print(a,HEX);
      Serial.print(F("=0x")); Serial.println(d,HEX);
      break;
    }
    case 'w': {
      uint16_t a = strtol(cmd.substring(2,5).c_str(), NULL, 16);
      uint8_t  d = strtol(cmd.substring(6,8).c_str(), NULL, 16);
      spiWrite(PIN_CS_RX, a, d);
      Serial.print(F("OK WR_RX 0x")); Serial.print(a,HEX);
      Serial.print(F("=0x")); Serial.println(d,HEX);
      break;
    }
    case 'X': {
      uint16_t a = strtol(cmd.substring(2,5).c_str(), NULL, 16);
      uint8_t  v = spiRead(PIN_CS_TX, a);
      Serial.print(F("OK RD_TX 0x")); Serial.print(a,HEX);
      Serial.print(F("=0x")); Serial.println(v,HEX);
      break;
    }
    case 'x': {
      uint16_t a = strtol(cmd.substring(2,5).c_str(), NULL, 16);
      uint8_t  v = spiRead(PIN_CS_RX, a);
      Serial.print(F("OK RD_RX 0x")); Serial.print(a,HEX);
      Serial.print(F("=0x")); Serial.println(v,HEX);
      break;
    }
    case '?':
      Serial.println(F("=== FMCW MIMO Radar (ESP32-C3 HW Trigger) ==="));
      Serial.println(F("I=Init V=Verify S=SeqOn D=SeqOff"));
      Serial.println(F("P=TxADV R=Reset M=MADV C N=SetCh"));
      Serial.println(F("F=1frame F N=Nframes T N=chirp(us)"));
      Serial.println(F("W/w=Write X/x=Read (hex)"));
      Serial.println(F("GPIO19->PFI0 triggers each chirp"));
      break;
    default:
      Serial.print(F("ERR: ")); Serial.println(cmd);
      break;
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);  // ESP32: let serial monitor attach after auto-reset
  pinMode(PIN_CS_TX, OUTPUT); pinMode(PIN_CS_RX, OUTPUT);
  pinMode(PIN_TxADV, OUTPUT); pinMode(PIN_TxRST, OUTPUT);
  pinMode(PIN_MADV, OUTPUT);  pinMode(PIN_MRST, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_GPIO_DIR, OUTPUT);
  digitalWrite(PIN_CS_TX, HIGH); digitalWrite(PIN_CS_RX, HIGH);
  digitalWrite(PIN_TxADV, LOW);  digitalWrite(PIN_TxRST, LOW);
  digitalWrite(PIN_MADV, LOW);   digitalWrite(PIN_MRST, LOW);
  digitalWrite(PIN_TRIG, LOW);
  digitalWrite(PIN_GPIO_DIR, HIGH);  // 4-wire SPI write direction (A→B)
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
  Serial.println(F(""));
  Serial.println(F("=== FMCW MIMO Radar (ESP32-C3 HW Trigger) ==="));
  Serial.println(F("GPIO19->PFI0 | 4MHz SPI | 24-bit Mode 0"));
  Serial.println(F("Send ? for help, I to init"));
}

void loop() {
  if (Serial.available()) processCommand();
}
