#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ======================== Enums / Encodings ======================== */
typedef enum { PRESC_4_5 = 0, PRESC_8_9 = 1 } Prescaler;
typedef enum { DEV_ADF4350 = 0, DEV_ADF4351 = 1 } DeviceType;
typedef enum { LowNoiseMode = 0, LowSpurMode = 1 } NSMod;
typedef enum { PD_Negative = 0, PD_Positive = 1 } PDPolarity;
typedef enum { BandClk_Auto = -1, BandClk_Low = 0, BandClk_High = 1 } BSClkMode;
typedef enum { ClkDiv_Off = 0, ClkDiv_FastLock = 1, ClkDiv_Resync = 2 } ClkDivMode;
typedef enum { Fb_Divider = 0, Fb_Fundamental = 1 } FBSelect;
typedef enum { Aux_Divided = 0, Aux_Fundamental = 1 } AuxOutSel;
typedef enum { LD_Low = 0, LD_Digital = 1, LD_High = 3 } LDPinMode;
typedef enum {
    Mux_ThreeState        = 0b000,  // Three-State output
    Mux_DVDD              = 0b001,  // DVDD
    Mux_DGND              = 0b010,  // DGND
    Mux_RCounter          = 0b011,  // R counter output
    Mux_NDivider          = 0b100,  // N divider output
    Mux_AnalogLockDetect  = 0b101,  // Analog LD
    Mux_DigitalLockDetect = 0b110,  // Digital LD
    // 0b111 reserved
} MuxOut;

/* Utilities */
int GCD_int(int a, int b);
double roundNearestEven(double x);

/* ======================== Calculate (R0 seeds) ======================== */
/* Mirrors pyadf435x.calculate_regs: computes INT/FRAC/MOD, output divider,
   prescaler, and a valid band-select clock divider/mode from user inputs. */
typedef struct {
    /* Reference / PFD setup */
    DeviceType device_type;        // ADF4351 default
    double     ref_freq_MHz;       // Reference Frequency in MHz - 25 MHz for our ADF4351 eval board
    int        r_cnt;              // R counter (1...1023)
    bool       ref_doubler;        // Reference Doubler - default false
    bool       ref_div2;           // Reference Divide by 2 - default false

    /* Target output*/
    double     freq_MHz;           // Desired RF out Frequency in MHz
    FBSelect   fb_sel;             // Feedback Select - default Fundamental

    /* Band-select clock (auto if desired) */
    int        bs_clk_div;         // Band Select Clock Divider (0=AUTO → computed)
    BSClkMode  bs_clk_mode;        // Band Select Clock Mode (BandClk_Auto chooses Low/High by 125 kHz PFD cutoff)
    
    /* Behavior */
    bool       enable_gcd;         // Enable GCD reduction of FRAC/MOD - default True
    bool       phase_adjust;       // Phase Adjust Enable - default false (True -> Disables VCO Band Select)
} CalcParams;

typedef struct {
    /* R0/R1 seed values */
    int        INTv, FRACv, MODv;  // Determines overall division ratio from VCO output to PFD input
    int        output_divider;     // power of two <= 64
    Prescaler  prescaler;          // (DB27, R1) Dual-Modulus Prescaler - PRESC_4_5 or PRESC_8_9

    /* Band select (used in R3/R4) */
    int        bs_clk_div;         // Band Select Clock Divider (1...255)
    BSClkMode  bs_clk_mode;        // Band Select Clock Mode
} CalcResult;

/* Returns 0 on success; nonzero on error with msg filled (if not NULL) */
int calculate_regs(const CalcParams* p, CalcResult* r, char* err, int errlen);

/* ======================== Pack (R0..R5) ======================== */
/* Mirrors pyadf435x.make_regs: packs the final 6 registers from parameters.
   Fields below are grouped by register/bit for clarity. */
typedef struct {
    DeviceType device_type;

    /* --------------------- R0 (DB30..DB0: [INT|FRAC|000]) ---------------------
       DB30..15: INT (16b)  | DB14..3: FRAC (12b) | DB2..0: 000
       Constraints: INT≥23 (4/5) or INT≥75 (8/9); FRAC,MOD ∈ [0..4095]
    */
    int        INTv, FRACv, MODv;

    /* --------------------- R1 (DB31..DB0: [PA|PR|PHASE|MOD|001]) --------------
       DB28: PhaseAdjust (0/1) - default false (True -> Disables VCO Band Select)
       DB27: Dual-Modulus Prescaler - PRESC_4_5 or PRESC_8_9 (0=4/5,1=8/9)
       DB26..15: PHASE (12b)  (when PhaseAdjust=0, conventionally PHASE=1)
       DB14..3 : MOD (12b)
       DB2..0  : 001
    */
    bool       phase_adjust;            
    uint16_t   phase_word;              
    Prescaler  prescaler;               

    /* --------------------- R2 (DB31..DB0) ------------------------------------
       DB30..29: LowNoise/LowSpur (00/11 recommended)
       DB28..26: MUXOUT - default ThreeState
       DB25    : Ref doubler (0/1)
       DB24    : RDIV2 (0/1) — divide ref by 2 abd makes 50% duty at PFD; required for CSR
       DB23..14: R counter (10b, 1..1023)
       DB13    : Double buffer for RF Output Divider R4 - default false
       DB12..9 : Charge pump current code (0..15)
       DB8     : LDF   (0=40 cycles Frac-N, 1=5 cycles Int-N) ← we set from FRAC==0
       DB7     : LDP   (0=10 ns Frac-N, 1=6 ns Int-N)        ← we set from FRAC==0
       DB6     : PD polarity (0=Neg,1=Pos) - Positive for passive/non-inverting filter
       DB5     : Power down - default false
       DB4     : CP three-state - default false
       DB3     : N & R counter reset - default false
       DB2..0  : 010
    */
    NSMod      n_s_mode;                
    MuxOut     mux_out;                 
    bool       ref_doubler, ref_div_2;  
    int        r_cnt;                   
    bool       double_buff_r4;         
    double     charge_pump_current_mA;     
    PDPolarity pd_polarity;             
    bool       powerdown, cp_three_state, counter_reset;

    /* --------------------- R3 (DB31..DB0) ------------------------------------
       DB23    : Band-select clock mode (0=Low,1=High) — auto-resolved if requested
       DB22    : ABP (0=6ns Frac-N, 1=3ns Int-N)          ← we set from FRAC==0
       DB21    : Charge cancel (Int-N:1, Frac-N:0)        ← we set from FRAC==0
       DB18    : CSR (cycle slip reduction; requires RDIV2=1 for 50% duty) - default false
       DB16..15: Clock Divider Mode (00 Off, 01 FastLock, 10 Resync) - default Off
       DB14..3 : Clock divider value (1..4095), often ~150
       DB2..0  : 011
    */
    BSClkMode  bs_clk_mode;
    bool       charge_cancel;           
    bool       csr;                     
    ClkDivMode clk_div_mode;            
    int        clock_divider_value;     

    /* --------------------- R4 (DB31..DB0) ------------------------------------
       DB23    : Feedback select (0=Divider, 1=Fundamental/default) 
       DB22..20: Output divider select (log2 of output_divider) - calculated
       DB19..12: Band-select clock divider (1..255; ≤254 if High mode on 4351)
       DB11    : VCO power down - default false
       DB10    : Mute till lock detect - default false
       DB9     : AUX output select (0=Divided/default,1=Fundamental)
       DB8     : AUX output enable - default false
       DB7..6  : AUX output power code (0..3) → {-4,-1,+2,+5} dBm - default 0
       DB5     : RF OUT enable - default true
       DB4..3  : RF OUT power code (0..3) → {-4,-1,+2,+5} dBm - default 3
       DB2..0  : 100
    */
    FBSelect   fb_sel;                  
    int        output_divider;          
    int        bs_clk_div;              
    bool       vco_powerdown;           
    bool       mute_till_ld;            
    AuxOutSel  aux_output_select;       
    bool       aux_output_enable;       
    int        aux_output_power_dBm;    
    bool       output_enable;           
    int        output_power_dBm;        

    /* --------------------- R5 (DB31..DB0) ------------------------------------
       DB23..22: LD pin mode (00 Low, 01 Digital LD/default, 11 High)
       DB21..19: must be 011
       DB2..0  : 101
    */
    LDPinMode  ld_pin_mode;             
} PackParams;

/* Packs regs[0..5]. Returns 0 on success; nonzero on error with msg. */
int make_regs(const PackParams* p, uint32_t regs[6], char* err, int errlen);

#ifdef __cplusplus
}
#endif