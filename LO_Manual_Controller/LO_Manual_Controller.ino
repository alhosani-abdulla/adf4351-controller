#include <Arduino.h>
#include <SPI.h>

extern "C" {
  #include "LORegisterLibrary.h"
}

/* ---------- Pins ---------- */
static const int PIN_LE    = 10;  // Latch Enable
static const int PIN_CE    = 2;   // Chip Enable
static const int PIN_LOSET = 6;   // (unused here) external trigger
static const int PIN_RESET = 7;   // (unused here) external reset
static const int PIN_CALIB = 8;   // (unused here) external band toggle
static const int PIN_MUX = 9;     // MUXOUT (Multiplexer Output)


/* ---------- Sweep bands ---------- */
static const double A_MIN  = 650.0, A_MAX  = 850.0, A_STEP  = 1.0;
static const double B_MIN  = 900.0, B_MAX  = 960.0, B_STEP  = 0.2;

/* ---------- State ---------- */
static bool   bandHigh      = false;   // false=A band, true=B band
static double curFreq       = A_MIN;
static bool   mtld          = false;   // Mute Till Lock Detect
static bool   muxIsLD       = true;    // true = MUXOUT is DigitalLockDetect

// NEW: user-settable RF output power for MAIN output (allowed: -4, -1, +2, +5 dBm)
static int8_t output_power_dBm = +5;

/* ---------- Utils ---------- */
static double quantizeToStep(double f, double fmin, double step) {
  long n = lround((f - fmin) / step);
  return fmin + n * step;
}

static double nextFreq(double f, bool hi) {
  const double fmin = hi ? B_MIN : A_MIN;
  const double fmax = hi ? B_MAX : A_MAX;
  const double step = hi ? B_STEP : A_STEP;
  double q = quantizeToStep(f, fmin, step);
  double nxt = q + step;
  if (nxt > fmax) nxt = fmax;  // change to wrap if you want
  return nxt;
}

static inline void writeADF4351(uint32_t reg) {
  digitalWrite(PIN_LE, LOW);
  SPI.transfer((reg >> 24) & 0xFF);
  SPI.transfer((reg >> 16) & 0xFF);
  SPI.transfer((reg >> 8)  & 0xFF);
  SPI.transfer(reg & 0xFF);
  digitalWrite(PIN_LE, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_LE, LOW);
}

// Prints the raw characters and their ASCII codes for debugging "Unknown" cases.
static void debugPrintBytes(const String& s) {
  Serial.print(F("DBG raw: '"));
  for (size_t i=0;i<s.length();++i) {
    char c = s[i];
    if (c=='\r') Serial.print("\\r");
    else if (c=='\n') Serial.print("\\n");
    else Serial.print(c);
  }
  Serial.println(F("'"));
  Serial.print(F("DBG hex: "));
  for (size_t i=0;i<s.length();++i) {
    char c = s[i];
    char buf[5];
    sprintf(buf, "%02X ", (unsigned char)c);
    Serial.print(buf);
  }
  Serial.println();
}

/* Returns true on success, false on error (prints error) */
static bool programLO(double freqMHz) {
  // --- calculate_regs ---
  CalcParams cp;
  cp.device_type  = DEV_ADF4351;
  cp.freq_MHz     = freqMHz;
  cp.ref_freq_MHz = 25.0;
  cp.r_cnt        = 1;
  cp.ref_doubler  = false;
  cp.ref_div2     = false;
  cp.fb_sel       = Fb_Fundamental;
  cp.bs_clk_div   = 0;               // AUTO
  cp.bs_clk_mode  = BandClk_Auto;    // AUTO (cutoff 125 kHz)
  cp.enable_gcd   = true;
  cp.phase_adjust = false;

  CalcResult cr;
  char err[128];
  int rc = calculate_regs(&cp, &cr, err, sizeof(err));
  if (rc != 0) {
    Serial.print(F("calculate_regs ERR @ "));
    Serial.print(freqMHz, 3);
    Serial.print(F(" MHz: "));
    Serial.println(err);
    return false;
  }

  // --- make_regs ---
  PackParams pp;
  memset(&pp, 0, sizeof(pp));

  // Core numbers
  pp.device_type  = DEV_ADF4351;
  pp.INTv         = cr.INTv;
  pp.FRACv        = cr.FRACv;
  pp.MODv         = cr.MODv;
  pp.prescaler    = cr.prescaler;

  // R1 phase fields
  pp.phase_adjust = false;   // per your current plan
  pp.phase_word   = 1;

  // R2 (set MUXOUT so the board LED likely shows lock)
  pp.n_s_mode                 = LowSpurMode;       // try spur mode first
  pp.mux_out                  = muxIsLD ? Mux_DigitalLockDetect : Mux_ThreeState;
  pp.ref_doubler              = false;
  pp.ref_div_2                = false;
  pp.r_cnt                    = 1;
  pp.double_buff_r4           = false;
  pp.charge_pump_current_mA   = 2.50;
  pp.pd_polarity              = PD_Positive;
  pp.powerdown                = false;
  pp.cp_three_state           = false;
  pp.counter_reset            = false;

  // R3
  pp.bs_clk_mode          = cr.bs_clk_mode;
  pp.csr                  = false;
  pp.clk_div_mode         = ClkDiv_Off;
  pp.clock_divider_value  = 150;
  // ABP & charge-cancel are derived in make_regs from FRAC==0

  // R4
  pp.fb_sel               = Fb_Fundamental;
  pp.output_divider       = cr.output_divider;
  pp.bs_clk_div           = cr.bs_clk_div;
  pp.vco_powerdown        = false;
  pp.mute_till_ld         = mtld;
  pp.aux_output_select    = Aux_Divided;
  pp.aux_output_enable    = false;
  pp.aux_output_power_dBm = -4;
  pp.output_enable        = true;
  pp.output_power_dBm     = output_power_dBm;

  // R5
  pp.ld_pin_mode          = LD_Digital;

  uint32_t regs[6];
  rc = make_regs(&pp, regs, err, sizeof(err));
  if (rc != 0) {
    Serial.print(F("make_regs ERR @ "));
    Serial.print(freqMHz, 3);
    Serial.print(F(" MHz: "));
    Serial.println(err);
    return false;
  }

  // Program sequence R5..R0
  writeADF4351(regs[5]);
  writeADF4351(regs[4]);
  writeADF4351(regs[3]);
  writeADF4351(regs[2]);
  writeADF4351(regs[1]);
  writeADF4351(regs[0]);

  Serial.print(F("Prog: "));
  Serial.print(freqMHz, 3);
  Serial.print(F(" MHz, Pout="));
  Serial.print(output_power_dBm);
  Serial.println(F(" dBm"));
  return true;
}

/* ---------- Serial command interface ---------- */
/*
  Commands (enter in Serial Monitor with "No line ending" or newline):
    f <freqMHz>  -> program that exact frequency (e.g. "f 742.4")
    n            -> advance to next step in the current band and program it
    a            -> toggle band (A: 650–850, step 1.0) / (B: 900–960, step 0.2)
    r            -> reset to band start (A_MIN or B_MIN)
    m 0|1        -> MUXOUT mode: 0 = three-state, 1 = digital lock detect (LED test)
    t            -> toggle Mute-Till-Lock (shows as R4 DB10)
    s            -> show status
    ?            -> help
*/

static String readLine() {
  static String buf;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;     // ignore CR
    if (c == '\n') {             // LF ends a line
      String out = buf;
      buf = "";
      return out;
    }
    buf += c;
  }
  return String();               // no complete line yet
}

static void printStatus() {
  Serial.print(F("Band: "));
  Serial.println(bandHigh ? F("High (900–960, 0.2 MHz)") : F("Low (650–850, 1.0 MHz)"));
  Serial.print(F("Freq: ")); Serial.print(curFreq, 3); Serial.println(F(" MHz"));
  Serial.print(F("Output Power: ")); Serial.print(output_power_dBm); Serial.println(F(" dBm"));
  Serial.print(F("MUXOUT: ")); Serial.println(muxIsLD ? F("DigitalLockDetect") : F("ThreeState"));
  Serial.print(F("MTLD: ")); Serial.println(mtld ? F("ON") : F("OFF"));
}

static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  f <MHz>  - program frequency (e.g. f 742.4)"));
  Serial.println(F("  n        - next step in current band (and program)"));
  Serial.println(F("  a        - toggle band"));
  Serial.println(F("  r        - reset to band start"));
  Serial.println(F("  m 0|1    - MUXOUT: 0=ThreeState, 1=DigitalLockDetect"));
  Serial.println(F("  t        - toggle Mute-Till-Lock"));
  Serial.println(F("  p <val>  - MAIN output power: -4, -1, +2, +5 (e.g. p +5)"));
  Serial.println(F("  s        - status"));
  Serial.println(F("  ?        - help"));
}

void setup() {
  pinMode(PIN_LE, OUTPUT);   digitalWrite(PIN_LE, LOW);
  pinMode(PIN_CE, OUTPUT);   digitalWrite(PIN_CE, HIGH);

  // If you still have the three GPIO lines from the Pi connected and want them as inputs:
  pinMode(PIN_LOSET, INPUT);
  pinMode(PIN_RESET, INPUT);
  pinMode(PIN_CALIB, INPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));

  Serial.begin(115200);
  Serial.setTimeout(50);   // keeps readStringUntil / serial reads snappy
  while (!Serial) {}

  Serial.println(F("\nADF4351 interactive test"));
  printHelp();

  // Start at beginning of low band
  bandHigh = false;
  curFreq = quantizeToStep(A_MIN, A_MIN, A_STEP);

  // Program the first point so you can observe the LED
  programLO(curFreq);
  printStatus();

  pinMode(PIN_MUX, INPUT);      // IMPORTANT: no INPUT_PULLUP (would pull to 5V)
  // optional: a quick print so we know state at boot
  Serial.print(F("MUXOUT@boot="));
  Serial.println(digitalRead(PIN_MUX));
}

// Simple edge/logging helper
static int lastMux = -1;

void loop() {
  String line = readLine();
  if (line.length() == 0) return;

  // Normalize: trim spaces/tabs around, but keep the first char (command letter)
  String raw = line;             // keep a copy for debugging if needed
  line.trim();
  if (line.length() == 0) return;

  // Extract command letter and argument (if any)
  char op = line[0];
  if (op >= 'a' && op <= 'z') op = (char)(op - 'a' + 'A');

  String arg = (line.length() > 1) ? line.substring(1) : String("");
  arg.trim();  // <-- critical: handles "f 650", "m    1", etc.

  // Uncomment for debugging weird cases:
  // debugPrintBytes(raw);
  // Serial.print(F("Parsed op=")); Serial.print(op); Serial.print(F(" arg='")); Serial.print(arg); Serial.println("'");

  if (op == 'F') {
    // Frequency: accepts "f650" or "f 650" or "f   650.125"
    double mhz = arg.toFloat();
    if (mhz > 10.0) {
      curFreq = mhz;
      programLO(curFreq);
      printStatus();
    } else {
      Serial.println(F("Usage: f 742.4"));
    }
  }
  else if (op == 'N') {
    curFreq = nextFreq(curFreq, bandHigh);
    programLO(curFreq);
    printStatus();
  }
  else if (op == 'A') {
    bandHigh = !bandHigh;
    curFreq = bandHigh ? B_MIN : A_MIN;
    programLO(curFreq);
    printStatus();
  }
  else if (op == 'R') {
    curFreq = bandHigh ? B_MIN : A_MIN;
    programLO(curFreq);
    printStatus();
  }
  else if (op == 'M') {
    // MUXOUT: accepts "m1" or "m 1" (anything nonzero becomes true)
    int v = arg.toInt();
    muxIsLD = (v != 0);
    programLO(curFreq);
    printStatus();
  }
  else if (op == 'T') {
    mtld = !mtld;
    programLO(curFreq);
    printStatus();
  }
  else if (op == 'P') {
    // Accepts: -4, -1, +2, +5 (with or without '+' and with/without space)
    int val = arg.toInt();  // handles "+5" -> 5, "-1" -> -1
    bool ok = (val == -4) || (val == -1) || (val == 2) || (val == 5);
    if (!ok) {
      Serial.println(F("Invalid power. Allowed: -4, -1, +2, +5  (example: p +5)"));
    } else {
      output_power_dBm = (int8_t)val;
      Serial.print(F("Set MAIN output power to "));
      Serial.print(output_power_dBm);
      Serial.println(F(" dBm"));
      // Re-program current frequency so change takes effect immediately
      programLO(curFreq);
      printStatus();
    }
  }
  else if (op == 'S') {
    printStatus();
  }
  else if (op == '?') {
    printHelp();
  }
  else {
    // Show raw bytes to reveal hidden characters if something odd came in
    Serial.println(F("Unknown. Type ?"));
    debugPrintBytes(raw);
  }
  
  // Poll MUXOUT ~1 kHz (cheap and reliable)
  static uint32_t lastUs = 0;
  uint32_t now = micros();
  if (now - lastUs >= 1000) {   // every 1 ms
    lastUs = now;
    int v = digitalRead(PIN_MUX);
    if (v != lastMux) {
      lastMux = v;
      // If MUXOUT is set to Digital Lock Detect:
      // 1 = LOCKED, 0 = UNLOCKED (per datasheet Digital LD)
      Serial.print(F("LD="));
      Serial.print(v);
      Serial.print(F("  ("));
      Serial.print(v ? F("LOCKED") : F("UNLOCKED"));
      Serial.println(F(")"));
    }
  }
}