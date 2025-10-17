# ADF4351 Controller

Arduino controller programs for the ADF4351 PLL frequency synthesizer with comprehensive register library following the ADF4351 datasheet. Includes manual control and automated sweep functionality.

## Repository Structure

This repository is structured as an Arduino library following the official Arduino library specification.

```
adf4351-controller/
├── README.md                  # This file
├── LICENSE                    # MIT License
├── library.properties         # Arduino library metadata
├── src/                       # Library source code
│   ├── ADF4351Controller.cpp  # ADF4351 register implementation
│   └── ADF4351Controller.h    # ADF4351 register header
└── examples/                  # Example sketches
    ├── ManualControl/         # Interactive serial control
    │   └── ManualControl.ino
    ├── BasicSweep/            # Configurable GPIO-controlled sweep
    │   └── BasicSweep.ino
    ├── SweepMixer/            # Band A sweep for mixer measurements
    │   └── SweepMixer.ino
    └── SweepFilter/           # Band B dual-power sweep for filter characterization
        └── SweepFilter.ino
```

## Examples

### 1. ManualControl
**File:** `examples/ManualControl/ManualControl.ino`

Interactive serial control program for manual frequency setting and testing.

#### Features:
- Manual frequency input via serial interface
- Real-time frequency adjustment
- Output power control (-4, -1, +2, +5 dBm)
- Lock detect monitoring
- Register calculation using ADF4351 datasheet specifications

#### Usage:
1. Upload to Arduino
2. Connect via USB
3. Open Serial Monitor (115200 baud)
4. Use commands to control the ADF4351

#### Serial Commands:
- `f <MHz>` - Set specific frequency (e.g., `f 742.4`)
- `p <dBm>` - Set output power: -4, -1, +2, or +5 dBm (e.g., `p 5`)
- `n` - Advance to next frequency step
- `a` - Toggle between bands (A: 650-850 MHz / B: 900-960 MHz)
- `r` - Reset to band start
- `m 0|1` - Set MUXOUT: 0=ThreeState, 1=DigitalLockDetect
- `t` - Toggle Mute-Till-Lock detect
- `s` - Show current status
- `?` - Show help menu

---

### 2. BasicSweep
**File:** `examples/BasicSweep/BasicSweep.ino`

General-purpose GPIO-controlled sweep with configurable frequency bands.

#### Features:
- Simple configurable sweep implementation
- VCO power control via GPIO
- Choice of two preset bands (A or B)
- Easy to modify for custom frequency ranges

#### Configuration:
Set `BAND_SELECT` to `'A'` or `'B'`, or modify frequency parameters directly:
```cpp
#define BAND_SELECT 'A'  // 'A' or 'B'

// Band A: 650-850 MHz, 2.0 MHz step @ +5 dBm
// Band B: 900-960 MHz, 0.2 MHz step @ +5 dBm
```

#### GPIO Control (Raspberry Pi):
- **Pin 6 (LOSET)**: Falling edge = program frequency, Rising edge = advance
- **Pin 7 (RESET)**: LOW = reset to band start
- **Pin 8 (VCO_CTRL)**: HIGH = VCO on (sweep active), LOW = VCO off

#### Usage:
Good starting point for custom sweep applications. Modify frequency range and output power as needed.

---

### 3. SweepMixer
**File:** `examples/SweepMixer/SweepMixer.ino`

Purpose-built sweep for **mixer characterization measurements**.

#### Specifications:
- **Frequency Range**: 650-850 MHz (Band A)
- **Step Size**: 2.0 MHz
- **Output Power**: +5 dBm (fixed)
- **Use Case**: Mixer response measurements

#### Features:
- Fixed high output power for mixer testing
- VCO power control for sweep enable/disable
- Optimized for coarse frequency sweeps

#### GPIO Control (Raspberry Pi):
- **Pin 6 (LOSET)**: Program/advance frequency
- **Pin 7 (RESET)**: Reset to 650 MHz
- **Pin 8 (VCO_CTRL)**: Enable/disable sweep

---

### 4. SweepFilter
**File:** `examples/SweepFilter/SweepFilter.ino`

Purpose-built sweep for **filter characterization with signal injection**.

#### Specifications:
- **Frequency Range**: 900-960 MHz (Band B)
- **Step Size**: 0.2 MHz (fine resolution)
- **Output Power**: Dual sweep (+5 dBm → -4 dBm)
- **Use Case**: Filter/cavity response measurements

#### Features:
- **Automatic power toggling**: Each RESET toggles between +5 dBm and -4 dBm
- **Dual-sweep protocol**: 
  1. First sweep at +5 dBm (high power)
  2. Second sweep at -4 dBm (low power for signal injection)
- Fine frequency resolution for detailed filter characterization

#### GPIO Control (Raspberry Pi):
- **Pin 6 (LOSET)**: Program/advance frequency
- **Pin 7 (RESET)**: Reset to 900 MHz + toggle power level
- **Pin 8 (PD_CTRL)**: LO power-down control (HIGH=on, LOW=off)

#### Power Control:
- Uses **register 2 power-down** for clean on/off switching
- When LOW: Complete power-down (no signal leakage)
- When HIGH: LO powered and sweep enabled

#### Sweep Sequence:
1. Start at 900 MHz, +5 dBm
2. Sweep to 960 MHz
3. Pi sends RESET → frequency resets to 900 MHz, power switches to -4 dBm
4. Sweep to 960 MHz at lower power
5. Next RESET toggles back to +5 dBm

## Hardware Setup

### Required Components:
- Arduino (Uno, Nano, or compatible)
- ADF4351 Local Oscillator module
- SPI connections between Arduino and ADF4351
- Raspberry Pi (for automated sweep control)

### Wiring:

#### Arduino to ADF4351 Module (SPI):
All examples use these connections:
- **MOSI** → ADF4351 DATA pin
- **SCK** → ADF4351 CLK pin  
- **Pin 10** → ADF4351 LE pin (Latch Enable)
- **Pin 2** → ADF4351 CE pin (Chip Enable, kept HIGH)
- **GND** → ADF4351 GND
- **VCC** → ADF4351 VCC (3.3V or 5V depending on module)

#### Arduino to Raspberry Pi (GPIO - for sweep examples):
Sweep examples (BasicSweep, SweepMixer, SweepFilter):
- **Pin 6** → Raspberry Pi GPIO (LOSET - Program/Advance signal)
- **Pin 7** → Raspberry Pi GPIO (RESET - Reset to band start)
- **Pin 8** → Raspberry Pi GPIO (VCO_CTRL - VCO power control)
- **GND** → Raspberry Pi GND (common ground)

#### Arduino to Laptop:
All examples support serial monitoring:
- **USB Cable** → For serial communication, monitoring, and power

## ADF4351Controller Library

The core ADF4351 register calculation library is located in the `src/` directory.

### Key Features:
- Full ADF4351 datasheet compliance
- Comprehensive register calculation
- Error checking and validation
- Support for both integer-N and fractional-N modes
- Configurable reference frequency and dividers

### Library Files:
- `src/ADF4351Controller.h` - Header with data structures and function declarations
- `src/ADF4351Controller.cpp` - Implementation following ADF4351 specifications
- `library.properties` - Arduino library metadata

## Installation

### Method 1: Clone into Arduino Libraries (Recommended)
1. Clone this repository directly into your Arduino libraries directory:
   ```bash
   cd ~/Documents/Arduino/libraries/  # macOS/Linux
   # or cd Documents\Arduino\libraries\  # Windows
   git clone https://github.com/alhosani-abdulla/adf4351-controller.git
   ```
2. Restart Arduino IDE
3. The library will appear as "ADF4351Controller" in **Sketch → Include Library**

### Method 2: ZIP Installation
1. Download the repository as ZIP from GitHub
2. In Arduino IDE: **Sketch → Include Library → Add .ZIP Library**
3. Select the downloaded ZIP file

### Using Example Sketches
1. In Arduino IDE: **File → Examples → ADF4351Controller**
2. Choose an example:
   - `ManualControl` - Interactive serial control
   - `BasicSweep` - Configurable GPIO sweep
   - `SweepMixer` - Mixer characterization (Band A)
   - `SweepFilter` - Filter characterization (Band B, dual power)
3. Configure your Arduino board:
   - **Tools → Board** → Select your Arduino model
   - **Tools → Processor** → Select your processor (if applicable)
   - **Tools → Port** → Select your USB port
   
   **Example for Arduino Nano:**
   - **Board:** "Arduino Nano"
   - **Processor:** "ATmega328P"
   - **Port:** "/dev/cu.usbserial-14110" (macOS/Linux) or "COM3" (Windows)
   
   > **Note:** The exact port name will vary. Look for `/dev/cu.usbserial-*` on macOS, `/dev/ttyUSB*` on Linux, or `COM*` on Windows.

4. Upload the sketch: **Sketch → Upload** or click the Upload button (→)

### Serial Monitor
- Open via **Tools → Serial Monitor** or click the magnifying glass icon
- Set baud rate to **115200** in the bottom-right dropdown
- All examples provide serial output for monitoring and debugging

## Application Notes

### Choosing the Right Example:
- **Testing/Development** → Use `ManualControl` for interactive testing
- **Custom Applications** → Start with `BasicSweep` and modify as needed
- **Mixer Measurements** → Use `SweepMixer` (650-850 MHz at +5 dBm)
- **Filter/Cavity Testing** → Use `SweepFilter` (900-960 MHz, dual power)

### Dual-Board Setup:
For applications requiring both frequency bands simultaneously:
1. Use two Arduino boards with separate ADF4351 modules
2. Upload `SweepMixer` to one board (Band A)
3. Upload `SweepFilter` to another board (Band B)
4. Control both from Raspberry Pi via separate GPIO pins

## Troubleshooting

### Common Issues:
- **No serial output:** Check baud rate (should be 115200)
- **Frequency not changing:** Verify SPI wiring connections
- **Compilation errors:** Ensure library is properly installed in Arduino/libraries/
- **Invalid frequency:** Check that frequency is within ADF4351 operating range
- **GPIO not working:** Verify common ground between Arduino and Raspberry Pi

### ADF4351 Frequency Range:
- **Fundamental Output:** 2200-4400 MHz
- **Divided Output:** 137.5-4400 MHz (with output dividers)
- Examples use divided output ranges (650-960 MHz)

## Support

For issues or questions:
- Check serial monitor output for error messages
- Verify hardware connections
- Ensure frequency values are within valid ranges
- Review ADF4351 datasheet for additional specifications

## Version History

- **v2.0** - Updated with new ADF4351Controller library following ADF4351 datasheet
- **v1.0** - Original implementation (obsolete)