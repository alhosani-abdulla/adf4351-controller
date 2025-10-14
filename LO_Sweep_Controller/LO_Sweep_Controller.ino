#include <Arduino.h>
#include <SPI.h>

extern "C" {
  #include "LORegisterLibrary.h"
}

/* ---------- Pins ---------- */
static const int PIN_LE    = 10;  // Latch Enable (LE)
static const int PIN_CE    = 2;   // Chip Enable (CE), HIGH = enabled
static const int PIN_LOSET = 6;   // From Pi: program/advance
static const int PIN_RESET = 7;   // From Pi: reset sweep to band start
static const int PIN_CALIB = 8;   // From Pi: select band (low/high)

/* ---------- Sweep bands & steps ---------- */
static const double A_MIN  = 650.0, A_MAX  = 850.0, A_STEP  = 2.0;
static const double B_MIN  = 902.6, B_MAX  = 957.6, B_STEP  = 0.2;

/* ---------- Globals ---------- */
static bool   bandHigh = true;     // false = Band A, true = Band B
static double curFreq  = B_MIN;

static int prevLOSet = HIGH;
static int prevReset = HIGH;
static int prevCalib = HIGH;

/* ---------- Helpers ---------- */
static double quantizeToStep(double f, double fmin, double step) {
  long n = lround((f - fmin) / step);
  return fmin + n * step;
}

static double nextFreq(double f, bool hiBand) {
  const double fmin = hiBand ? B_MIN : A_MIN;
  const double fmax = hiBand ? B_MAX : A_MAX;
  const double step = hiBand ? B_STEP : A_STEP;

  double q = quantizeToStep(f, fmin, step);
  double next = q + step;
  if (next > fmax) next = fmax;   // change to: next = fmin;  // if you prefer wrap
  return next;
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

static bool programLO(double freqMHz) {
  // ---- calculate_regs ----
  CalcParams cp;
  cp.device_type  = DEV_ADF4351;
  cp.freq_MHz     = freqMHz;
  cp.ref_freq_MHz = 25.0;
  cp.r_cnt        = 1;
  cp.ref_doubler  = false;
  cp.ref_div2     = false;
  cp.fb_sel       = Fb_Fundamental;
  cp.bs_clk_div   = 0;               // AUTO
  cp.bs_clk_mode  = BandClk_Auto;    // AUTO (125 kHz cutoff)
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

  // ---- make_regs ----
  PackParams pp;
  memset(&pp, 0, sizeof(pp));

  pp.device_type  = DEV_ADF4351;

  // R0/R1 core
  pp.INTv         = cr.INTv;
  pp.FRACv        = cr.FRACv;
  pp.MODv         = cr.MODv;
  pp.prescaler    = cr.prescaler;
  pp.phase_adjust = false;
  pp.phase_word   = 1;

  // R2
  pp.n_s_mode                 = LowSpurMode;      // try LowSpur first
  pp.mux_out                  = Mux_ThreeState;
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
  pp.bs_clk_mode          = cr.bs_clk_mode;  // resolved
  pp.csr                  = false;
  pp.clk_div_mode         = ClkDiv_Off;
  pp.clock_divider_value  = 150;

  // R4
  pp.fb_sel               = Fb_Fundamental;
  pp.output_divider       = cr.output_divider;
  pp.bs_clk_div           = cr.bs_clk_div;
  pp.vco_powerdown        = false;
  pp.mute_till_ld         = false;
  pp.aux_output_select    = Aux_Divided;
  pp.aux_output_enable    = false;
  pp.aux_output_power_dBm = -4;
  pp.output_enable        = true;
  pp.output_power_dBm     = +5;

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

  // Recommended order: R5..R0
  writeADF4351(regs[5]);
  writeADF4351(regs[4]);
  writeADF4351(regs[3]);
  writeADF4351(regs[2]);
  writeADF4351(regs[1]);
  writeADF4351(regs[0]);

  Serial.print(F("Prog: "));
  Serial.print(freqMHz, 3);
  Serial.println(F(" MHz"));
  return true;
}

/* ---------- GPIO edge handling ---------- */
static void handleCalibToggle() {
  int s = digitalRead(PIN_CALIB);
  if (s != prevCalib) {
    prevCalib = s;
    if (s == LOW) {                // falling edge: toggle band
      bandHigh = !bandHigh;
      curFreq = bandHigh ? B_MIN : A_MIN;
      curFreq = quantizeToStep(curFreq, bandHigh?B_MIN:A_MIN, bandHigh?B_STEP:A_STEP);
      Serial.print(F("CALIB falling: band -> "));
      Serial.println(bandHigh ? F("High") : F("Low"));
    }
  }
}

static void handleReset() {
  int s = digitalRead(PIN_RESET);
  if (s != prevReset) {
    prevReset = s;
    if (s == LOW) {                // falling edge: reset to start of current band
      curFreq = bandHigh ? B_MIN : A_MIN;
      curFreq = quantizeToStep(curFreq, bandHigh?B_MIN:A_MIN, bandHigh?B_STEP:A_STEP);
      Serial.println(F("Reset to band start"));
    }
  }
}

static void handleLOSet() {
  int s = digitalRead(PIN_LOSET);
  if (s != prevLOSet) {
    if (s == LOW) {
      Serial.println(F("LOSET falling: program"));
      // Falling edge: program current frequency
      programLO(curFreq);
    } else {
      // Rising edge: advance to next
      curFreq = nextFreq(curFreq, bandHigh);
      Serial.print(F("LOSET rising: advance -> "));
      Serial.println(curFreq, 3);
    }
    prevLOSet = s;
  }
}

/* ---------- Setup / Loop ---------- */
void setup() {
  pinMode(PIN_LE, OUTPUT);   digitalWrite(PIN_LE, LOW);
  pinMode(PIN_CE, OUTPUT);   digitalWrite(PIN_CE, HIGH);

  pinMode(PIN_LOSET, INPUT);
  pinMode(PIN_RESET, INPUT);
  pinMode(PIN_CALIB, INPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));

  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("\nADF4351 sweep ready."));
  Serial.println(F("Use GPIO: LOSet(D6)=program/advance, Reset(D7)=reset, Calib(D8)=toggle band."));

  prevLOSet = digitalRead(PIN_LOSET);
  prevReset = digitalRead(PIN_RESET);
  prevCalib = digitalRead(PIN_CALIB);

  bandHigh = false;
  curFreq = quantizeToStep(B_MIN, B_MIN, B_STEP);
  // Program the initial frequency so LD can assert at boot (like the manual sketch)
  programLO(curFreq);
}\

 
void loop() {
  handleCalibToggle();
  handleReset();
  handleLOSet();
}
