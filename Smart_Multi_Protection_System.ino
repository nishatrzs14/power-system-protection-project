/*****************************************************************************
 *  Smart Multi-Protection System : Load-wise Fault Isolation
 *  (Individual Trip in Every Branch)
 *
 *  Board   : Arduino UNO
 *  Sensors : ZMPT101B (AC voltage)  -> A0
 *            ACS712-5A x4 (current) -> A1 (Branch 1) ... A4 (Branch 4)
 *  Relays  : 1-channel relay x4     -> D2 (Branch 1) ... D5 (Branch 4)
 *  GSM     : SIM900A (SoftwareSerial) -> D6 / D7
 *
 *  What it does
 *   - Reads mains voltage and the current of each of the 4 branches.
 *   - Overload / short-circuit  -> only that branch trips, others keep running.
 *   - Over / under voltage      -> all branches trip, auto restore when the
 *                                  voltage is stable again.
 *   - Sends an SMS for every fault (non-blocking, healthy branches are still
 *     monitored while the SMS is being sent).
 *
 *  Serial Monitor (115200 baud) commands:
 *     V228  -> calibrate voltage (type the real mains voltage from a multimeter)
 *     R1..R4 / RA -> reset (re-close) a tripped branch / all branches
 *     S     -> print status
 *
 *  !!! SAFETY: this project works with 220V AC. Use proper insulation, an
 *  enclosure, a fuse/MCB on the input, and never touch the circuit while it
 *  is powered. This is a learning / demo project, not a certified protection
 *  device.
 *****************************************************************************/

#include <SoftwareSerial.h>
#include <EEPROM.h>

// ============================ 1. PIN MAP ====================================
#define NUM_BRANCHES 4

const uint8_t PIN_VOLTAGE = A0;                                  // ZMPT101B OUT
const uint8_t PIN_CURRENT[NUM_BRANCHES] = {A1, A2, A3, A4};      // ACS712 OUT
const uint8_t PIN_RELAY[NUM_BRANCHES]   = {2, 3, 4, 5};          // Relay IN

const uint8_t GSM_RX_PIN = 6;   // Arduino RX  <-  SIM900A TXD
const uint8_t GSM_TX_PIN = 7;   // Arduino TX  ->  SIM900A RXD

SoftwareSerial gsm(GSM_RX_PIN, GSM_TX_PIN);

// ========================= 2. USER SETTINGS =================================
// ---- SMS ----
const char PHONE_NUMBER[] = "+8801XXXXXXXXX";   // <-- put your number here

// ---- Relay wiring ----
// Pin level that CONNECTS the load (relay energised, load wired on NO).
// Most blue relay modules are active-LOW -> LOW. If yours is active-HIGH, use HIGH.
const uint8_t RELAY_ON_LEVEL = LOW;

// ---- Voltage protection (RMS volts) ----
const float OVER_VOLTAGE_V  = 245.0;
const float UNDER_VOLTAGE_V = 180.0;
const float VOLTAGE_HYST_V  = 5.0;     // must be back inside the limits by this margin
const float VOLT_CAL_DEFAULT = 0.0;    // volts per ADC count. 0 = not calibrated yet
                                       // (use Serial command V<volts> to calibrate)

// ---- Current protection (RMS amps, per branch) ----
// Set the overload limit BETWEEN "1 bulb" and "2 bulbs" current:
//   40W bulb : 0.18A / 0.36A -> use ~0.27
//   60W bulb : 0.27A / 0.55A -> use ~0.40   (default)
//  100W bulb : 0.45A / 0.91A -> use ~0.68
const float OVERLOAD_A[NUM_BRANCHES] = {0.40, 0.40, 0.40, 0.40};
const float SHORT_CIRCUIT_A = 3.50;     // instant trip (ACS712-5A max ~5A peak)
const float CURRENT_NOISE_A = 0.05;     // readings below this are treated as 0

// ---- Timing / filtering ----
const uint8_t       VOLTAGE_CONFIRM_SCANS  = 3;        // ~0.6 s of bad voltage
const uint8_t       OVERLOAD_CONFIRM_SCANS = 3;        // ~0.6 s of overload
const bool          AUTO_RECLOSE           = true;     // overload only (never after short)
const uint8_t       MAX_RECLOSE_ATTEMPTS   = 3;
const unsigned long RECLOSE_DELAY_MS       = 30000UL;  // wait before re-closing
const unsigned long RECLOSE_OK_MS          = 60000UL;  // healthy this long = recovered
const unsigned long VOLTAGE_RESTORE_MS     = 10000UL;  // stable voltage before restore
const unsigned long INRUSH_BLANK_MS        = 600UL;    // ignore bulb inrush after connect
const unsigned long STATUS_PRINT_MS        = 2000UL;

// ---- ADC / sensor constants ----
const float ADC_REF_V          = 5.0;
const float ADC_STEPS          = 1023.0;
const float ACS712_V_PER_A     = 0.185;        // 5A module = 185 mV/A
const unsigned long SAMPLE_WINDOW_US = 40000UL; // 40 ms = 2 full cycles @ 50 Hz

// ============================ 3. DATA TYPES =================================
const uint8_t FAULT_NONE      = 0;
const uint8_t FAULT_OVERLOAD  = 1;
const uint8_t FAULT_SHORT     = 2;
const uint8_t FAULT_OVERVOLT  = 3;
const uint8_t FAULT_UNDERVOLT = 4;

struct Branch {
  float         current;        // Irms (A)
  bool          tripped;
  bool          lockedOut;      // needs manual reset
  uint8_t       fault;
  uint8_t       overloadCount;
  uint8_t       recloseCount;
  unsigned long tripTime;
  unsigned long closeTime;
};

Branch br[NUM_BRANCHES];

float         mainsV         = 0.0;
float         mainsCounts    = 0.0;
float         voltCalFactor  = VOLT_CAL_DEFAULT;
uint8_t       voltFault      = FAULT_NONE;
uint8_t       voltBadCount   = 0;
bool          voltOkTiming   = false;
unsigned long voltOkSince    = 0;
unsigned long lastStatus     = 0;

// EEPROM layout
const int     EE_ADDR_MAGIC = 0;
const int     EE_ADDR_VCAL  = 1;
const uint8_t EE_MAGIC      = 0xA5;

// ---- SMS queue / state machine ----
#define SMS_QUEUE_LEN 4
#define SMS_MAX_LEN   110

char          smsQueue[SMS_QUEUE_LEN][SMS_MAX_LEN];
uint8_t       smsHead  = 0;
uint8_t       smsCount = 0;
uint8_t       smsState = 0;          // 0 idle, 1 wait '>' prompt, 2 wait result
uint8_t       smsRetry = 0;
unsigned long smsT0    = 0;
unsigned long smsNextAllowed = 0;

char          gsmRx[40];
uint8_t       gsmRxLen = 0;

char          cmdBuf[12];
uint8_t       cmdLen = 0;

// ============================ 4. HELPERS ====================================
const char* faultName(uint8_t f) {
  switch (f) {
    case FAULT_OVERLOAD:  return "Overload";
    case FAULT_SHORT:     return "Short circuit/L-G fault";
    case FAULT_OVERVOLT:  return "Over voltage";
    case FAULT_UNDERVOLT: return "Under voltage";
    default:              return "None";
  }
}

void relayConnect(uint8_t i) { digitalWrite(PIN_RELAY[i], RELAY_ON_LEVEL);  }
void relayTrip(uint8_t i)    { digitalWrite(PIN_RELAY[i], !RELAY_ON_LEVEL); }

// RMS of the AC component (DC offset removed) in ADC counts
float readRmsCounts(uint8_t pin) {
  analogRead(pin);                       // dummy read: let the ADC mux settle
  long sum = 0, sumSq = 0, n = 0;
  unsigned long t0 = micros();
  while (micros() - t0 < SAMPLE_WINDOW_US) {
    long d = (long)analogRead(pin) - 512;
    sum   += d;
    sumSq += d * d;
    n++;
  }
  if (n == 0) return 0.0;
  float mean = (float)sum / n;
  float var  = (float)sumSq / n - mean * mean;
  if (var < 0.0) var = 0.0;
  return sqrt(var);
}

float measureVoltage() {
  mainsCounts = readRmsCounts(PIN_VOLTAGE);
  return mainsCounts * voltCalFactor;
}

float measureCurrent(uint8_t i) {
  float counts = readRmsCounts(PIN_CURRENT[i]);
  float amps   = counts * (ADC_REF_V / ADC_STEPS) / ACS712_V_PER_A;
  if (amps < CURRENT_NOISE_A) amps = 0.0;
  return amps;
}

// ============================ 5. GSM / SMS ==================================
void gsmReadChars() {
  while (gsm.available()) {
    char c = gsm.read();
    if (gsmRxLen >= sizeof(gsmRx) - 1) {          // rolling buffer
      memmove(gsmRx, gsmRx + 20, gsmRxLen - 20);
      gsmRxLen -= 20;
    }
    gsmRx[gsmRxLen++] = c;
    gsmRx[gsmRxLen]   = 0;
  }
}

void gsmClearRx() {
  gsmRxLen = 0;
  gsmRx[0] = 0;
}

// Blocking AT command - used ONLY during setup
bool gsmSend(const __FlashStringHelper* cmd, const char* expect, unsigned long timeoutMs) {
  gsmClearRx();
  gsm.println(cmd);
  unsigned long t0 = millis();
  while (millis() - t0 < timeoutMs) {
    gsmReadChars();
    if (strstr(gsmRx, expect)) return true;
  }
  return false;
}

void gsmInit() {
  gsm.begin(9600);
  delay(1000);

  bool ok = false;
  for (uint8_t k = 0; k < 8 && !ok; k++) ok = gsmSend(F("AT"), "OK", 1000);
  if (!ok) {
    Serial.println(F("[GSM] No response - check wiring/power. SMS may fail."));
    return;
  }
  gsmSend(F("ATE0"), "OK", 1000);              // echo off
  gsmSend(F("AT+CMGF=1"), "OK", 1000);         // SMS text mode
  gsmSend(F("AT+CSCS=\"GSM\""), "OK", 1000);

  bool reg = false;
  for (uint8_t k = 0; k < 20 && !reg; k++) {   // wait for network (max ~20 s)
    gsmSend(F("AT+CREG?"), "OK", 1000);
    if (strstr(gsmRx, ",1") || strstr(gsmRx, ",5")) reg = true;
    else delay(1000);
  }
  Serial.println(reg ? F("[GSM] Network registered.") : F("[GSM] Not registered yet (check SIM/signal)."));
}

void queueSms(const char* text) {
  if (smsCount >= SMS_QUEUE_LEN) return;       // queue full -> drop
  uint8_t tail = (smsHead + smsCount) % SMS_QUEUE_LEN;
  strncpy(smsQueue[tail], text, SMS_MAX_LEN - 1);
  smsQueue[tail][SMS_MAX_LEN - 1] = 0;
  smsCount++;
}

void popSms() {
  smsHead = (smsHead + 1) % SMS_QUEUE_LEN;
  if (smsCount > 0) smsCount--;
}

void smsFinished(bool success) {
  if (success) {
    Serial.println(F("[SMS] Sent."));
    popSms();
    smsRetry = 0;
    smsNextAllowed = millis() + 1000UL;
  } else {
    gsm.write((uint8_t)27);                    // ESC: cancel any pending SMS
    Serial.println(F("[SMS] Failed."));
    if (++smsRetry >= 2) { popSms(); smsRetry = 0; }   // give up after 2 tries
    smsNextAllowed = millis() + 3000UL;
  }
  smsState = 0;
}

// Call this often - it never blocks
void smsTask() {
  gsmReadChars();
  switch (smsState) {
    case 0:
      if (smsCount > 0 && (long)(millis() - smsNextAllowed) >= 0) {
        gsmClearRx();
        gsm.print(F("AT+CMGS=\""));
        gsm.print(PHONE_NUMBER);
        gsm.print(F("\"\r"));
        smsT0 = millis();
        smsState = 1;
      }
      break;

    case 1:                                    // waiting for '>' prompt
      if (strchr(gsmRx, '>')) {
        gsm.print(smsQueue[smsHead]);
        gsm.write((uint8_t)26);                // Ctrl+Z = send
        gsmClearRx();
        smsT0 = millis();
        smsState = 2;
      } else if (millis() - smsT0 > 5000UL) {
        smsFinished(false);
      }
      break;

    case 2:                                    // waiting for result
      if (strstr(gsmRx, "OK"))                                   smsFinished(true);
      else if (strstr(gsmRx, "ERROR") || millis() - smsT0 > 20000UL) smsFinished(false);
      break;
  }
}

void queueFaultSms(uint8_t i) {
  char msg[SMS_MAX_LEN];
  char val[10];
  dtostrf(br[i].current, 1, 2, val);
  snprintf(msg, sizeof(msg), "ALERT: Branch %u tripped - %s (%sA). %s",
           (unsigned)(i + 1), faultName(br[i].fault), val,
           br[i].lockedOut ? "Locked out, manual reset needed."
                           : "Other branches ON.");
  queueSms(msg);
}

// ====================== 6. BRANCH CONTROL / PROTECTION ======================
void connectBranch(uint8_t i) {
  relayConnect(i);
  br[i].tripped       = false;
  br[i].fault         = FAULT_NONE;
  br[i].overloadCount = 0;
  br[i].closeTime     = millis();
}

void tripBranch(uint8_t i, uint8_t fault) {
  if (br[i].tripped) return;

  relayTrip(i);                                // trip FIRST, then do the rest
  br[i].tripped       = true;
  br[i].fault         = fault;
  br[i].tripTime      = millis();
  br[i].overloadCount = 0;

  if (fault == FAULT_SHORT) {
    br[i].lockedOut = true;                    // never auto-close into a short
  } else if (fault == FAULT_OVERLOAD &&
             (!AUTO_RECLOSE || br[i].recloseCount >= MAX_RECLOSE_ATTEMPTS)) {
    br[i].lockedOut = true;
  }

  Serial.print(F("[TRIP] Branch "));
  Serial.print(i + 1);
  Serial.print(F(" - "));
  Serial.print(faultName(fault));
  Serial.print(F(" ("));
  Serial.print(br[i].current, 2);
  Serial.println(F(" A)"));

  // SMS only on first trip or when finally locked out (avoids SMS spam)
  if ((fault == FAULT_OVERLOAD || fault == FAULT_SHORT) &&
      (br[i].recloseCount == 0 || br[i].lockedOut)) {
    queueFaultSms(i);
  }
}

void resetBranch(uint8_t i) {
  if (!br[i].tripped) {
    Serial.print(F("Branch ")); Serial.print(i + 1); Serial.println(F(" is already ON."));
    return;
  }
  if (voltFault != FAULT_NONE) {
    Serial.println(F("Voltage fault active - cannot reset yet."));
    return;
  }
  br[i].lockedOut    = false;
  br[i].recloseCount = 0;
  connectBranch(i);
  Serial.print(F("[RESET] Branch ")); Serial.print(i + 1); Serial.println(F(" connected."));
}

void handleVoltage() {
  if (voltCalFactor <= 0.0) return;            // not calibrated -> voltage protection off

  bool over  = mainsV > OVER_VOLTAGE_V;
  bool under = mainsV < UNDER_VOLTAGE_V;
  char msg[SMS_MAX_LEN];
  char val[10];
  dtostrf(mainsV, 1, 1, val);

  if (over || under) {
    voltOkTiming = false;
    if (voltBadCount < 255) voltBadCount++;

    if (voltBadCount >= VOLTAGE_CONFIRM_SCANS && voltFault == FAULT_NONE) {
      voltFault = over ? FAULT_OVERVOLT : FAULT_UNDERVOLT;
      for (uint8_t i = 0; i < NUM_BRANCHES; i++) tripBranch(i, voltFault);
      snprintf(msg, sizeof(msg), "ALERT: %s %sV. All branches tripped.", faultName(voltFault), val);
      queueSms(msg);
    }
  } else {
    voltBadCount = 0;
    if (voltFault != FAULT_NONE) {
      bool inBand = (mainsV < OVER_VOLTAGE_V - VOLTAGE_HYST_V) &&
                    (mainsV > UNDER_VOLTAGE_V + VOLTAGE_HYST_V);
      if (!inBand) {
        voltOkTiming = false;
      } else if (!voltOkTiming) {
        voltOkTiming = true;
        voltOkSince  = millis();
      } else if (millis() - voltOkSince >= VOLTAGE_RESTORE_MS) {
        voltFault    = FAULT_NONE;
        voltOkTiming = false;
        for (uint8_t i = 0; i < NUM_BRANCHES; i++) {
          if (br[i].tripped &&
              (br[i].fault == FAULT_OVERVOLT || br[i].fault == FAULT_UNDERVOLT)) {
            connectBranch(i);
            delay(200);                        // stagger to reduce inrush
          }
        }
        Serial.println(F("[VOLT] Voltage normal - branches restored."));
        snprintf(msg, sizeof(msg), "Voltage normal (%sV). Branches restored.", val);
        queueSms(msg);
      }
    }
  }
}

void checkBranch(uint8_t i) {
  Branch &b = br[i];

  if (b.tripped) { b.current = 0.0; return; }

  b.current = measureCurrent(i);
  if (millis() - b.closeTime < INRUSH_BLANK_MS) return;     // ignore inrush

  // ---- short circuit / L-G fault (very high current): confirm once, then trip ----
  if (b.current >= SHORT_CIRCUIT_A) {
    float again = measureCurrent(i);
    if (again >= SHORT_CIRCUIT_A) {
      b.current = again;
      tripBranch(i, FAULT_SHORT);
      return;
    }
  }

  // ---- overload (both lamps ON in the same branch) ----
  if (b.current >= OVERLOAD_A[i]) {
    if (++b.overloadCount >= OVERLOAD_CONFIRM_SCANS) tripBranch(i, FAULT_OVERLOAD);
  } else {
    b.overloadCount = 0;
  }
}

void handleRecovery() {
  char msg[SMS_MAX_LEN];
  for (uint8_t i = 0; i < NUM_BRANCHES; i++) {
    Branch &b = br[i];

    // automatic re-close after overload
    if (b.tripped && b.fault == FAULT_OVERLOAD && !b.lockedOut &&
        voltFault == FAULT_NONE && millis() - b.tripTime >= RECLOSE_DELAY_MS) {
      b.recloseCount++;
      connectBranch(i);
      Serial.print(F("[RECLOSE] Branch ")); Serial.print(i + 1);
      Serial.print(F(" attempt ")); Serial.println(b.recloseCount);
    }

    // stayed healthy long enough -> recovered
    if (!b.tripped && b.recloseCount > 0 && millis() - b.closeTime >= RECLOSE_OK_MS) {
      b.recloseCount = 0;
      snprintf(msg, sizeof(msg), "Branch %u restored normally.", (unsigned)(i + 1));
      queueSms(msg);
    }
  }
}

// ============================ 7. SERIAL / STATUS ============================
void printStatus() {
  if (voltCalFactor > 0.0) {
    Serial.print(F("V=")); Serial.print(mainsV, 1);
  } else {
    Serial.print(F("V=uncalibrated (raw=")); Serial.print(mainsCounts, 1);
    Serial.print(F(" counts, send V<volts>)"));
  }
  for (uint8_t i = 0; i < NUM_BRANCHES; i++) {
    Serial.print(F(" | B")); Serial.print(i + 1); Serial.print(F(": "));
    Serial.print(br[i].current, 2); Serial.print(F("A "));
    if (br[i].tripped) {
      Serial.print(F("TRIP("));
      Serial.print(faultName(br[i].fault));
      Serial.print(br[i].lockedOut ? F(",LOCKED)") : F(")"));
    } else {
      Serial.print(F("ON"));
    }
  }
  Serial.println();
}

void calibrateVoltage(float actualV) {
  if (actualV < 100.0 || actualV > 300.0) {
    Serial.println(F("Usage: V<measured mains volts>, e.g. V228"));
    return;
  }
  float counts = 0.0;
  for (uint8_t k = 0; k < 5; k++) counts += readRmsCounts(PIN_VOLTAGE);
  counts /= 5.0;
  if (counts < 5.0) {
    Serial.println(F("No AC signal on A0 - check ZMPT101B wiring / trimpot."));
    return;
  }
  voltCalFactor = actualV / counts;
  EEPROM.write(EE_ADDR_MAGIC, EE_MAGIC);
  EEPROM.put(EE_ADDR_VCAL, voltCalFactor);
  Serial.print(F("Calibrated & saved. Factor = "));
  Serial.println(voltCalFactor, 4);
}

void processCommand() {
  char c0 = toupper(cmdBuf[0]);
  if (c0 == 'R') {
    if (toupper(cmdBuf[1]) == 'A') {
      for (uint8_t i = 0; i < NUM_BRANCHES; i++) resetBranch(i);
    } else {
      int n = atoi(cmdBuf + 1);
      if (n >= 1 && n <= NUM_BRANCHES) resetBranch(n - 1);
      else Serial.println(F("Use R1..R4 or RA"));
    }
  } else if (c0 == 'V') {
    calibrateVoltage(atof(cmdBuf + 1));
  } else if (c0 == 'S') {
    printStatus();
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdLen > 0) {
        cmdBuf[cmdLen] = 0;
        processCommand();
        cmdLen = 0;
      }
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    }
  }
}

// ============================ 8. SETUP / LOOP ===============================
void setup() {
  // Relays first: force every branch OFF so nothing glitches at power-up
  for (uint8_t i = 0; i < NUM_BRANCHES; i++) {
    digitalWrite(PIN_RELAY[i], !RELAY_ON_LEVEL);
    pinMode(PIN_RELAY[i], OUTPUT);
    br[i].tripped = true;
  }

  Serial.begin(115200);
  Serial.println(F("\nSmart Multi-Protection System - starting..."));

  // Load saved voltage calibration
  if (EEPROM.read(EE_ADDR_MAGIC) == EE_MAGIC) {
    float f;
    EEPROM.get(EE_ADDR_VCAL, f);
    if (f > 0.05 && f < 50.0) voltCalFactor = f;
  }
  if (voltCalFactor <= 0.0) {
    Serial.println(F("WARNING: voltage NOT calibrated -> over/under-voltage protection is OFF."));
    Serial.println(F("Measure mains with a multimeter and send e.g.  V228"));
  }

  gsmInit();

  // Initial voltage check, then connect branches one by one
  mainsV = measureVoltage();
  bool voltOk = (voltCalFactor <= 0.0) ||
                (mainsV <= OVER_VOLTAGE_V && mainsV >= UNDER_VOLTAGE_V);
  if (voltOk) {
    for (uint8_t i = 0; i < NUM_BRANCHES; i++) {
      connectBranch(i);
      delay(300);
    }
  } else {
    voltFault = (mainsV > OVER_VOLTAGE_V) ? FAULT_OVERVOLT : FAULT_UNDERVOLT;
    for (uint8_t i = 0; i < NUM_BRANCHES; i++) br[i].fault = voltFault;
    Serial.println(F("Voltage out of range at start - branches kept OFF."));
  }

  queueSms("Smart protection system started.");
  Serial.println(F("System ready. Commands: V<volts>, R1..R4, RA, S"));
}

void loop() {
  // 1) mains voltage
  mainsV = measureVoltage();
  handleVoltage();
  smsTask();

  // 2) every branch, one by one (SMS keeps running in between)
  for (uint8_t i = 0; i < NUM_BRANCHES; i++) {
    checkBranch(i);
    smsTask();
  }

  // 3) auto re-close / recovery, serial commands, status print
  handleRecovery();
  handleSerial();
  smsTask();

  if (millis() - lastStatus >= STATUS_PRINT_MS) {
    lastStatus = millis();
    printStatus();
  }
}
