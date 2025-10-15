#include <Arduino.h>
#include <SPI.h>

extern "C" {
  #include "LORegisterLibrary.h"
}

/* ---------- Pins ---------- */
static const int PIN_LE       = 10;  // Latch Enable (LE)
static const int PIN_CE       = 2;   // Chip Enable (CE), HIGH = enabled
static const int PIN_LOSET    = 6;   // From Pi: program/advance
static const int PIN_RESET    = 7;   // From Pi: reset sweep to band start
static const int PIN_VCO_CTRL = 8;   // From Pi: VCO power control (HIGH=on, LOW=off)

/* ---------- Frequency band configuration ---------- */
// Basic sweep example - configure for your needs
// Set BAND_SELECT to choose between two preset bands,
// or modify FREQ_MIN/FREQ_MAX/FREQ_STEP directly
#define BAND_SELECT 'A'  // 'A' or 'B' (see below)

// Preset band configurations (modify as needed)
#if BAND_SELECT == 'A'
  static const double FREQ_MIN  = 650.0;   // MHz
  static const double FREQ_MAX  = 850.0;   // MHz
  static const double FREQ_STEP = 2.0;     // MHz
  static const int8_t OUTPUT_POWER = +5;   // dBm
#elif BAND_SELECT == 'B'
  static const double FREQ_MIN  = 900.0;   // MHz
  static const double FREQ_MAX  = 960.0;   // MHz
  static const double FREQ_STEP = 0.2;     // MHz
  static const int8_t OUTPUT_POWER = +5;   // dBm
#else
  #error "BAND_SELECT must be 'A' or 'B'"
#endif

/* ---------- Globals ---------- */
static double curFreq  = FREQ_MIN;
static bool   vcoPowerOn = false;  // VCO power state

static int prevLOSet    = HIGH;
static int prevReset    = HIGH;
static int prevVCOCtrl  = HIGH;

/* ---------- Helpers ---------- */
static double quantizeToStep(double f, double fmin, double step) {
  long n = lround((f - fmin) / step);
  return fmin + n * step;
}

static double nextFreq(double f) {
  double q = quantizeToStep(f, FREQ_MIN, FREQ_STEP);
  double next = q + FREQ_STEP;
  if (next > FREQ_MAX) next = FREQ_MAX;   // change to: next = FREQ_MIN;  // if you prefer wrap
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
  pp.vco_powerdown        = !vcoPowerOn;  // Control VCO power based on global state
  pp.mute_till_ld         = false;
  pp.aux_output_select    = Aux_Divided;
  pp.aux_output_enable    = false;
  pp.aux_output_power_dBm = -4;
  pp.output_enable        = true;
  pp.output_power_dBm     = OUTPUT_POWER;

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
  Serial.print(F(" MHz @ "));
  Serial.print(OUTPUT_POWER);
  Serial.println(F(" dBm"));
  return true;
}

/* ---------- GPIO edge handling ---------- */
static void handleVCOControl() {
  int s = digitalRead(PIN_VCO_CTRL);
  if (s != prevVCOCtrl) {
    prevVCOCtrl = s;
    if (s == HIGH) {
      // HIGH: Turn VCO ON (sweep start)
      vcoPowerOn = true;
      Serial.println(F("VCO ON - Sweep enabled"));
      // Reprogram current frequency with VCO on
      programLO(curFreq);
    } else {
      // LOW: Turn VCO OFF (sweep end)
      vcoPowerOn = false;
      Serial.println(F("VCO OFF - Sweep disabled"));
      // Reprogram to power down VCO
      programLO(curFreq);
    }
  }
}

static void handleReset() {
  int s = digitalRead(PIN_RESET);
  if (s != prevReset) {
    prevReset = s;
    if (s == LOW) {                // falling edge: reset to start of band
      curFreq = FREQ_MIN;
      curFreq = quantizeToStep(curFreq, FREQ_MIN, FREQ_STEP);
      Serial.println(F("Reset to band start"));
    }
  }
}

static void handleLOSet() {
  int s = digitalRead(PIN_LOSET);
  if (s != prevLOSet) {
    if (s == LOW) {
      // Falling edge: program current frequency (only if VCO is on)
      if (vcoPowerOn) {
        Serial.println(F("LOSET falling: program"));
        programLO(curFreq);
      } else {
        Serial.println(F("LOSET falling: skipped (VCO off)"));
      }
    } else {
      // Rising edge: advance to next frequency
      curFreq = nextFreq(curFreq);
      Serial.print(F("LOSET rising: advance -> "));
      Serial.print(curFreq, 3);
      Serial.println(F(" MHz"));
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
  pinMode(PIN_VCO_CTRL, INPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));

  Serial.begin(115200);
  while (!Serial) {}
  
  Serial.println(F("\n=== ADF4351 Basic Sweep ==="));
  Serial.print(F("Range: "));
  Serial.print(FREQ_MIN, 1);
  Serial.print(F("-"));
  Serial.print(FREQ_MAX, 1);
  Serial.print(F(" MHz, step "));
  Serial.print(FREQ_STEP, 1);
  Serial.print(F(" MHz @ "));
  Serial.print(OUTPUT_POWER);
  Serial.println(F(" dBm"));
  Serial.println(F("GPIO Control:"));
  Serial.println(F("  D6 (LOSET): program/advance frequency"));
  Serial.println(F("  D7 (RESET): reset to band start"));
  Serial.println(F("  D8 (VCO_CTRL): VCO power (HIGH=on, LOW=off)"));
  Serial.println();

  prevLOSet = digitalRead(PIN_LOSET);
  prevReset = digitalRead(PIN_RESET);
  prevVCOCtrl = digitalRead(PIN_VCO_CTRL);

  // Initialize VCO state from pin
  vcoPowerOn = (prevVCOCtrl == HIGH);
  
  curFreq = quantizeToStep(FREQ_MIN, FREQ_MIN, FREQ_STEP);
  
  // Program initial frequency with current VCO state
  Serial.print(F("Initial frequency: "));
  Serial.print(curFreq, 3);
  Serial.print(F(" MHz, VCO: "));
  Serial.println(vcoPowerOn ? F("ON") : F("OFF"));
  programLO(curFreq);
}

void loop() {
  handleVCOControl();
  handleReset();
  handleLOSet();
}
