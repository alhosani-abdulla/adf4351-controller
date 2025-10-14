#include "LORegisterLibrary.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

/* ---------- Utilities (same behavior as Python) ---------- */

int GCD_int(int a, int b) {
    while (1) {
        if (a == 0) return b;
        else if (b == 0) return a;
        else if (a > b) a %= b;
        else            b %= a;
    }
}

double roundNearestEven(double x) {
    double ip;
    double frac = modf(x, &ip);
    if (fabs(frac) == 0.5) {
        /* if integer part even -> return ip; else to nearest away-from-zero */
        if (fmod(ip, 2.0) == 0.0) return ip;
        else return ip + (x > 0 ? 1.0 : -1.0);
    } else {
        /* C99 round() is ties-away-from-zero; but we only reach here when not .5,
           so it equals bankers for non-ties. */
        return round(x);
    }
}

/* ---------- Calculation (pyadf435x.calculate_regs) ---------- */

int calculate_regs(const CalcParams* p, CalcResult* r, char* err, int errlen) {
    if (!p || !r) { if (err) snprintf(err, errlen, "Null parameter"); return -1; }
    if (p->r_cnt < 1 || p->r_cnt > 1023) {
        if (err) snprintf(err, errlen, "r_cnt=%d is out of range (must be 1–1023)", p->r_cnt);
        return -2;
    }

    /* PFD in MHz */
    double pfd = (p->ref_freq_MHz * (p->ref_doubler ? 2.0 : 1.0)) /
                 ((p->ref_div2 ? 2.0 : 1.0) * (double)p->r_cnt);

    /* Auto-set band-select clock mode if not specified */
    BSClkMode bs_clk_mod = p->bs_clk_mode;
    if (bs_clk_mod == BandClk_Auto) {
        bs_clk_mod = (pfd >= 0.125) ? BandClk_High : BandClk_Low;  // cutoff = 125 kHz
    }

    /* Find output_divider = 2^k s.t. 2200/output_div <= freq (k in [0..6]) */
    int log2_od;
    int output_divider = 1;
    for (log2_od = 0; log2_od < 7; ++log2_od) {
        output_divider = 1 << log2_od;
        if (2200.0 / (double)output_divider <= p->freq_MHz) break;
    }
    if (log2_od == 7) { /* fallback to max */
        output_divider = 64;
    }

    /* VCO and prescaler selection per datasheet */
    double vco_MHz = p->freq_MHz * (double)output_divider;
    // If VCO > 3600 MHz => prescaler must be 8/9; else use 4/5
    Prescaler prescaler = (vco_MHz > 3600.0) ? PRESC_8_9 : PRESC_4_5; 

    /* N */
    double N = (p->fb_sel == Fb_Fundamental)
                 ? (p->freq_MHz * output_divider / pfd)
                 : (p->freq_MHz / pfd);

    int INTv = (int)floor(N);
    int MODv = (int)roundNearestEven(1000.0 * pfd);
    int FRACv = (int)roundNearestEven((N - (double)INTv) * (double)MODv);

    // Reduce by GCD if enabled
    if (p->enable_gcd) {
        int g = GCD_int(MODv, FRACv);        // works fine even if FRACv==0
        if (g > 1) { MODv /= g; FRACv /= g; }
    }
    // Int-N safety: after GCD, MOD will be 1; bump to 2
    if (FRACv == 0 && MODv == 1) MODv = 2;

    // pfd in MHz, FRACv already computed
    if (FRACv != 0) {
        if (pfd > 32.0) { if (err) snprintf(err, errlen, "Frac-N PFD must be <= 32 MHz"); return -3; }
    } else {
        bool band_select_enabled = !p->phase_adjust; // DB28=1 disables band-select on R0 updates
        if (band_select_enabled) {
            if (pfd > 45.0) { if (err) snprintf(err, errlen, "Int-N PFD must be <= 45 MHz when band-select is enabled"); return -4; }
            if (pfd > 32.0 && p->device_type == DEV_ADF4351 && bs_clk_mod == BandClk_Low) {
                if (err) snprintf(err, errlen, "Use High band-select mode when PFD > 32 MHz (Int-N)"); return -5; }
        } else { // band-select disabled (phase-adjust enabled)
            if (pfd > 90.0) { if (err) snprintf(err, errlen, "Int-N PFD must be <= 90 MHz when band-select is disabled"); return -6; }
        }
    }

    /* --- Band-select clock divider (auto if 0) --- */
    /* Limits: Low mode fBS ≤ 125 kHz, High mode fBS ≤ 500 kHz
    Divider range 1..255 (and ≤254 in High mode on ADF4351) */

    double fbs_max_kHz = (bs_clk_mod == BandClk_Low) ? 125.0 : 500.0;
    int band_div = p->bs_clk_div;

    /* Auto-select if <= 0 */
    if (band_div <= 0) {
        band_div = (int)ceil((1000.0 * pfd) / fbs_max_kHz);  // pfd in MHz → kHz
    }

    /* Clamp per datasheet */
    if (p->device_type == DEV_ADF4351 && bs_clk_mod == BandClk_High && band_div > 254)
        band_div = 254;
    if (band_div < 1)   band_div = 1;
    if (band_div > 255) band_div = 255;

    /* Ensure fBS ≤ limit */
    double fbs_kHz = (1000.0 * pfd) / (double)band_div;
    if (fbs_kHz > fbs_max_kHz) {
        band_div = (int)ceil((1000.0 * pfd) / fbs_max_kHz);
        if (p->device_type == DEV_ADF4351 && bs_clk_mod == BandClk_High && band_div > 254)
            band_div = 254;
        if (band_div > 255) band_div = 255;
    }

    /* band-select clock frequency in kHz: 1000*PFD / divider */
    double band_clk_kHz = 1000.0 * pfd / (double)band_div;

    if (band_clk_kHz > 500.0) {
        if (err) snprintf(err, errlen, "Band Select Clock Frequency > 500 kHz");
        return -7;
    } else if (band_clk_kHz > 125.0) {
        if (p->device_type == DEV_ADF4351) {
            if (bs_clk_mod == BandClk_Low) {
                if (err) snprintf(err, errlen,
                    "Band Select Clock >125 kHz in Low mode; set High mode or lower it");
                return -8;
            }
        } else {
            if (err) snprintf(err, errlen, "Band Select Clock >125 kHz (ADF4350)"); return -9;
        }
    }

    /* Fill result */
    r->INTv           = INTv;
    r->MODv           = MODv;
    r->FRACv          = FRACv;
    r->output_divider = output_divider;
    r->bs_clk_div     = band_div;
    r->bs_clk_mode    = bs_clk_mod;
    r->prescaler      = prescaler;
    return 0;
}

/* ---------- Lookup Tables & Utility Converters ---------- */

typedef struct { double key; int code; } DPair;

static const struct {
    DPair cp[16];
    int   n_cp;
} LUTS = {
    .cp = {
        {0.31,0}, {0.63,1}, {0.94,2}, {1.25,3},
        {1.56,4}, {1.88,5}, {2.19,6}, {2.50,7},
        {2.81,8}, {3.13,9}, {3.44,10}, {3.75,11},
        {4.06,12},{4.38,13},{4.49,14},{5.00,15}
    },
    .n_cp = 16
};

static int lut_cp(double mA, int* code_out) {
    for (int i = 0; i < LUTS.n_cp; i++)
        if (fabs(LUTS.cp[i].key - mA) < 1e-9) { *code_out = LUTS.cp[i].code; return 0; }
    return -1;
}

static int lut_output_power_dBm(int dBm, int* code_out) {
    switch (dBm) {
        case -4: *code_out=0; return 0;
        case -1: *code_out=1; return 0;
        case +2: *code_out=2; return 0;
        case +5: *code_out=3; return 0;
        default: return -1;
    }
}

static inline int is_power_of_two_and_le(int v, int maxpow2) {
    return (v > 0 && v <= maxpow2 && (v & (v - 1)) == 0);
}

/* ---------- Register pack (pyadf435x.make_regs) ---------- */

int make_regs(const PackParams* p, uint32_t regs[6], char* err, int errlen) {
    if (!p || !regs) return -1;

    if (p->INTv < 0 || p->INTv > 65535) { if (err) snprintf(err, errlen, "INT out of range"); return -2; }
    if (p->FRACv < 0 || p->FRACv > 4095) { if (err) snprintf(err, errlen, "FRAC out of range"); return -3; }
    if (p->MODv < 0 || p->MODv > 4095)   { if (err) snprintf(err, errlen, "MOD out of range"); return -4; }

    // prescaler is now an enum: PRESC_4_5 or PRESC_8_9
    int min_INT = (p->prescaler == PRESC_8_9) ? 75 : 23;
    if (p->INTv < min_INT) {
        if (err) snprintf(err, errlen,
            "INT=%d violates minimum %d for prescaler %s. Adjust output divider or lower PFD (increase R).",
            p->INTv, min_INT, (p->prescaler == PRESC_8_9) ? "8/9" : "4/5");
        return -10;
    }

    int cp_code, main_pwr_code, aux_pwr_code;
    if (lut_cp(p->charge_pump_current_mA, &cp_code) != 0)
        { if (err) snprintf(err, errlen, "charge_pump_current invalid"); return -5; }
    if (lut_output_power_dBm(p->output_power_dBm, &main_pwr_code) != 0)
        { if (err) snprintf(err, errlen, "output_power invalid"); return -6; }
    if (lut_output_power_dBm(p->aux_output_power_dBm, &aux_pwr_code) != 0)
        { if (err) snprintf(err, errlen, "aux_output_power invalid"); return -7; }

    if (!is_power_of_two_and_le(p->output_divider, 64))
        { if (err) snprintf(err, errlen, "output_divider must be power of two <= 64"); return -8; }

    int output_divider_select = (int)(log((double)p->output_divider)/log(2.0));
    if ((1 << output_divider_select) != p->output_divider)
        { if (err) snprintf(err, errlen, "output_divider_select not integer"); return -9; }

    uint32_t R[6] = {0};

    // Determine if in Integer-N mode
    bool intN_mode = (p->FRACv == 0);
    /* R0
    * DB31        : Reserved (must be 0)
    * DB30..DB15  : INT   (16 bits)
    * DB14..DB3   : FRAC  (12 bits)
    * DB2..DB0    : Control = 0b000  (selects Register 0)
    */

    R[0] = ((uint32_t)p->INTv                    << 15) |  // DB30..15
           ((uint32_t)p->FRACv                    << 3) |  // DB14..3
           0x0UL;                                          // DB2..0

    /* R1
    * DB31..DB29 : Reserved (0)
    * DB28       : Phase Adjust enable (0=off, 1=on)
    * DB27       : Prescaler (0 = 4/5, 1 = 8/9)
    * DB26..DB15 : PHASE (12 bits)  — conventionally 1 when Phase Adjust is off
    * DB14..DB3  : MOD   (12 bits)
    * DB2..DB0   : Control = 0b001
    */
    
    // Validate PHASE width
    uint16_t phase12 = (uint16_t)(p->phase_adjust ? p->phase_word : 1u);
    if (phase12 > 0x0FFF) {
        if (err) snprintf(err, errlen, "PHASE must be 12-bit (0..4095)");
        return -9;
    }
    
    int prescaler_bit = (p->prescaler == PRESC_8_9) ? 1 : 0;
    int phase_adjust_bit = p->phase_adjust ? 1 : 0;

    R[1] = ((uint32_t)phase_adjust_bit           << 28) |  // DB28
           ((uint32_t)prescaler_bit              << 27) |  // DB27
           ((uint32_t)phase12                    << 15) |  // DB26..15
           ((uint32_t)p->MODv                    << 3)  |  // DB14..3
           0x1UL;                                          // DB2..0

    /* R2
    * DB31        : Reserved (0)
    * DB30..DB29  : Low-Noise/Low-Spur mode (2 bits)
    *               00 = Low Noise Mode, 11 = Low Spur Mode (recommended encodings)
    * DB28..DB26  : MUXOUT (3 bits)
    * DB25        : Reference Doubler (1/0)
    * DB24        : RDIV2 (Reference ÷2) (1/0)
    * DB23..DB14  : R Counter (10 bits, 1..1023)
    * DB13        : Double Buffer R4 (1/0)
    * DB12..DB9   : Charge Pump Current code (4 bits, 0..15) — from your LUT
    * DB8         : LDF (Lock Detect Function) - Implemented using intN_mode flag.
    *               0 = monitor 40 PFD cycles  (recommended for Frac-N)
    *               1 = monitor 5  PFD cycles  (recommended for Int-N)
    * DB7         : LDP (Lock Detect Precision) - Implemented using intN_mode flag.
    *               0 = 10 ns (pairs with LDF=0)   → Frac-N recommended DB8:DB7 = 00
    *               1 = 6  ns (pairs with LDF=1)   → Int-N  recommended DB8:DB7 = 11
    * DB6         : PD Polarity (0=Neg, 1=Pos)
    * DB5         : Power Down
    * DB4         : CP Three-State
    * DB3         : Counter Reset
    * DB2..DB0    : Control = 0b010
    */

    /* LDF/LDP mapping */
    uint32_t lns_bits = (p->n_s_mode == LowSpurMode) ? 0b11u : 0b00u;
    
    R[2] = (lns_bits                             << 29) |  // DB30..29
           ((uint32_t)p->mux_out                 << 26) |  // DB28..26
           ((uint32_t)(p->ref_doubler?1:0)       << 25) |  // DB25
           ((uint32_t)(p->ref_div_2?1:0)         << 24) |  // DB24
           ((uint32_t)p->r_cnt                   << 14) |  // DB23..14
           ((uint32_t)(p->double_buff_r4?1:0)    << 13) |  // DB13
           (cp_code                              << 9)  |  // DB12..9
           ((uint32_t)(intN_mode?1:0)            << 8)  |  // DB8  LDF
           ((uint32_t)(intN_mode?1:0)            << 7)  |  // DB7  LDP
           ((uint32_t)p->pd_polarity             << 6)  |  // DB6
           ((uint32_t)(p->powerdown?1:0)         << 5)  |  // DB5
           ((uint32_t)(p->cp_three_state?1:0)    << 4)  |  // DB4
           ((uint32_t)(p->counter_reset?1:0)     << 3)  |  // DB3
           0x2UL;                                          // DB2..0

    /* R3
    * DB31..DB24 : Reserved (0)
    * DB23       : Band Select Clock Mode
    *              0 = Low  (set from PFD < 125 kHz)
    *              1 = High (set from PFD >= 125 kHz)
    * DB22       : ABP (Anti-Backlash Pulse Width)
    *              0 = 6 ns (recommended for Frac-N, FRAC≠0)
    *              1 = 3 ns (recommended for Int-N, FRAC=0)
    * DB21       : Charge Cancel (Int-N:1, Frac-N:0)
    * DB20..DB19 : Reserved (0)
    * DB18       : CSR (Cycle Slip Reduction, 0/1, usually 0)
    * DB17       : Reserved (0)
    * DB16..DB15 : ClkDivMode
    *              00 = Off
    *              01 = FastLock
    *              10 = Resync
    * DB14..DB3  : Clock Divider Value (12 bits, e.g. 150)
    * DB2..DB0   : Control = 0b011
    */
    
    if (p->csr && !p->ref_div_2) {
        if (err) snprintf(err, errlen, "Warning: CSR requires ref_div_2=1 for 50%% duty");
        // (do not return; just warn)
    }
    int bs_bit = (p->bs_clk_mode == BandClk_High) ? 1 : 0;

    R[3] = ((uint32_t)bs_bit                     << 23) |  // DB23
           ((uint32_t)(intN_mode ? 1 : 0)        << 22) |  // DB22
           ((uint32_t)(intN_mode ? 1 : 0)        << 21) |  // DB21  
           ((uint32_t)(p->csr ? 1 : 0)           << 18) |  // DB18  
           ((uint32_t)p->clk_div_mode            << 15) |  // DB16..15
           ((uint32_t)p->clock_divider_value     << 3)  |  // DB14..3
           0x3UL;                                          // DB2..0

    if (p->device_type != DEV_ADF4351) {
        // Mask out ADF4351-only fields for other devices (ADF4350)
        R[3] &= ~((1u << 23) | (1u << 22) | (1u << 21));
    }

    /* R4
    * DB31..DB24 : Reserved (0)
    * DB23       : Feedback Select (0=Divider, 1=Fundamental)
    * DB22..DB20 : Output Divider Select (log2 of output_divider)
    * DB19..DB12 : Band Select Clock Divider (8 bits)
    * DB11       : VCO Powerdown
    * DB10       : Mute Till Lock Detect
    * DB9        : Aux Output Select (0=Divided, 1=Fundamental)
    * DB8        : Aux Output Enable
    * DB7..DB6   : Aux Output Power (per LUT → 0..3)
    * DB5        : RF Output Enable
    * DB4..DB3   : RF Output Power (per LUT → 0..3)
    * DB2..DB0   : Control = 0b100
    */

    if (p->fb_sel == Fb_Fundamental && p->output_divider != 1)
        fprintf(stderr, "Warning: Fundamental feedback used with divider >1\n");

    R[4] = ((uint32_t)p->fb_sel                  << 23) |  // DB23
           ((uint32_t)output_divider_select      << 20) |  // DB22..20
           ((uint32_t)p->bs_clk_div              << 12) |  // DB19..12
           ((uint32_t)(p->vco_powerdown ? 1:0)   << 11) |  // DB11
           ((uint32_t)(p->mute_till_ld ? 1:0)    << 10) |  // DB10
           ((uint32_t)p->aux_output_select        << 9) |  // DB9
           ((uint32_t)(p->aux_output_enable ? 1:0)<< 8) |  // DB8
           ((uint32_t)aux_pwr_code                << 6) |  // DB7..6
           ((uint32_t)(p->output_enable ? 1:0)    << 5) |  // DB5
           ((uint32_t)main_pwr_code               << 3) |  // DB4..3
           0x4UL;                                          // DB2..0

    /* R5
    * DB31..DB24 : Reserved (0)
    * DB23..DB22 : Lock Detect Pin Mode (per enum: 0=Low, 1=Digital LD, 3=High)
    * DB21..DB19 : Reserved = 0b011 (must be set)
    * DB18..DB3  : Reserved (0)
    * DB2..DB0   : Control = 0b101
    *
    * NOTE on AVR width: use 32-bit literals (UL) so (3 << 19) does not vanish.
    */

    R[5] = ((uint32_t)p->ld_pin_mode             << 22) |  // DB23..22
           (3UL                                  << 19) |  // DB21..19 
           0x5UL;                                          // DB2..0

    for (int i=0;i<6;i++) regs[i] = R[i];
    return 0;
}