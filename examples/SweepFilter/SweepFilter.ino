#include <Arduino.h>
#include <SPI.h>

extern "C" {
  #include "ADF4351Controller.h"
}

/* ---------- Pins ---------- */
static const int PIN_LE         = 10;  // Latch Enable (LE)
static const int PIN_CE         = 2;   // Chip Enable (CE), HIGH = enabled
static const int PIN_LOSET      = 6;   // From Pi: program/advance
static const int PIN_RESET      = 7;   // From Pi: reset sweep to band start
static const int PIN_PD_CTRL    = 8;   // From Pi: Power-down control (HIGH=on, LOW=off)

/* ---------- Frequency band configuration ---------- */
// Band B (Filter/Signal Injection): 900-960 MHz, step 0.2 MHz
static const double FREQ_MIN  = 900.2;
static const double FREQ_MAX  = 960.2;
static const double FREQ_STEP = 0.2;

/* ---------- Globals ---------- */
static double curFreq      = FREQ_MIN;
static bool   powerdownLO  = true;   // Power-down state (true=off, false=on)
static int8_t outputPower  = +5;     // Current output power setting (+5 or -4 dBm)

static int prevLOSet      = HIGH;
static int prevReset      = HIGH;
static int prevPDCtrl     = HIGH;

/* ---------- Helpers ---------- */
static double quantizeToStep(double f, double fmin, double step) {
  long n = lround((f - fmin) / step);
  return fmin + n * step;
}

static double nextFreq(double f) {
  double q = quantizeToStep(f, FREQ_MIN, FREQ_STEP);
  double next = q + FREQ_STEP;
  if (next > FREQ_MAX) next = FREQ_MAX;
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
  pp.n_s_mode                 = LowSpurMode;
  pp.mux_out                  = Mux_ThreeState;
  pp.ref_doubler              = false;
  pp.ref_div_2                = false;
  pp.r_cnt                    = 1;
  pp.double_buff_r4           = false;
  pp.charge_pump_current_mA   = 2.50;
  pp.pd_polarity              = PD_Positive;
  pp.powerdown                = powerdownLO;  // Use power-down control from global state
  pp.cp_three_state           = false;
  pp.counter_reset            = false;

  // R3
  pp.bs_clk_mode          = cr.bs_clk_mode;
  pp.csr                  = false;
  pp.clk_div_mode         = ClkDiv_Off;
  pp.clock_divider_value  = 150;

  // R4
  pp.fb_sel               = Fb_Fundamental;
  pp.output_divider       = cr.output_divider;
  pp.bs_clk_div           = cr.bs_clk_div;
  pp.vco_powerdown        = false;  // Keep VCO powered (use R2 powerdown instead)
  pp.mute_till_ld         = true;
  pp.aux_output_select    = Aux_Divided;
  pp.aux_output_enable    = false;
  pp.aux_output_power_dBm = -4;
  pp.output_enable        = true;
  pp.output_power_dBm     = outputPower;  // Use current power setting

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
  Serial.print(outputPower);
  Serial.println(F(" dBm"));
  return true;
}

/* ---------- GPIO edge handling ---------- */
static void handlePowerControl() {
  int s = digitalRead(PIN_PD_CTRL);
  if (s != prevPDCtrl) {
    prevPDCtrl = s;
    if (s == HIGH) {
      // HIGH: Power ON (sweep enabled)
      powerdownLO = false;
      Serial.println(F("LO ON - Sweep enabled"));
      programLO(curFreq);
    } else {
      // LOW: Power DOWN (sweep disabled)
      powerdownLO = true;
      Serial.println(F("LO OFF - Sweep disabled"));
      programLO(curFreq);
    }
  }
}

static void handleReset() {
  int s = digitalRead(PIN_RESET);
  if (s != prevReset) {
    prevReset = s;
    if (s == LOW) {
      curFreq = FREQ_MIN;
      curFreq = quantizeToStep(curFreq, FREQ_MIN, FREQ_STEP);
      
      // Toggle power level on each reset
      if (outputPower == +5) {
        outputPower = -4;
        Serial.println(F("Reset to band start @ -4 dBm"));
      } else {
        outputPower = +5;
        Serial.println(F("Reset to band start @ +5 dBm"));
      }
    }
  }
}

static void handleLOSet() {
  int s = digitalRead(PIN_LOSET);
  if (s != prevLOSet) {
    if (s == LOW) {
      // Falling edge: program current frequency (only if LO is powered on)
      if (!powerdownLO) {
        Serial.println(F("LOSET falling: program"));
        programLO(curFreq);
      } else {
        Serial.println(F("LOSET falling: skipped (LO powered down)"));
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
  pinMode(PIN_PD_CTRL, INPUT);

  SPI.begin();
  SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));

  Serial.begin(115200);
  while (!Serial) {}
  
  Serial.println(F("\n=== ADF4351 Filter Sweep (Dual Power) ==="));
  Serial.println(F("Band: 900-960 MHz, step 0.2 MHz"));
  Serial.println(F("Dual sweep: +5 dBm, then -4 dBm (toggles on RESET)"));
  Serial.println(F("GPIO Control:"));
  Serial.println(F("  D6 (LOSET): program/advance frequency"));
  Serial.println(F("  D7 (RESET): reset to band start + toggle power"));
  Serial.println(F("  D8 (PD_CTRL): LO power-down (HIGH=on, LOW=off)"));
  Serial.println();

  prevLOSet = digitalRead(PIN_LOSET);
  prevReset = digitalRead(PIN_RESET);
  prevPDCtrl = digitalRead(PIN_PD_CTRL);

  // Initialize states from pins
  powerdownLO = (prevPDCtrl == LOW);  // LOW = powered down, HIGH = powered on
  
  curFreq = quantizeToStep(FREQ_MIN, FREQ_MIN, FREQ_STEP);
  
  // Program initial frequency with current settings
  Serial.print(F("Initial frequency: "));
  Serial.print(curFreq, 3);
  Serial.print(F(" MHz, LO: "));
  Serial.print(powerdownLO ? F("OFF") : F("ON"));
  Serial.print(F(", Power: "));
  Serial.print(outputPower);
  Serial.println(F(" dBm"));
  programLO(curFreq);
}

void loop() {
  handlePowerControl();
  handleReset();
  handleLOSet();
}
