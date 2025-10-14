# Local Oscillator Controller

This directory contains Arduino programs for controlling the ADF4351 Local Oscillator using the updated LORegisterLibrary that follows the ADF4351 datasheet implementation.

## Programs

### 1. LO_Manual_Controller
**File:** `LO_Manual_Controller/LO_Manual_Controller.ino`

Manual frequency control program that allows you to set specific frequencies for the Local Oscillator.

#### Features:
- Manual frequency input via serial interface
- Real-time frequency adjustment
- Register calculation using ADF4351 datasheet specifications
- Serial monitoring for debugging and control

#### Usage:
1. Upload the program to your Arduino
2. Connect Arduino to your laptop via USB cable
3. Open Serial Monitor (115200 baud rate)
4. Use the available commands to control the Local Oscillator
5. Monitor output for confirmation and debugging information

#### Serial Commands:
- `f <MHz>` - Program specific frequency (e.g., `f 742.4`)
- `n` - Advance to next step in current band and program it
- `a` - Toggle between bands (A: 650–850 MHz, step 1.0) / (B: 900–960 MHz, step 0.2)
- `r` - Reset to band start frequency
- `m 0|1` - Set MUXOUT mode: 0=ThreeState, 1=DigitalLockDetect
- `t` - Toggle Mute-Till-Lock detect
- `s` - Show current status
- `?` - Show help menu

### 2. LO_Sweep_Controller  
**File:** `LO_Sweep_Controller/LO_Sweep_Controller.ino`

Automated frequency sweep program that steps through a range of frequencies.

#### Features:
- Automated frequency sweeping across defined range
- Configurable start/stop frequencies and step size
- Configurable dwell time at each frequency
- Serial output for sweep progress monitoring

#### Usage:
1. Upload the program to your Arduino
2. Connect Arduino to Raspberry Pi via GPIO pins (see GPIO Control section below)
3. Connect Arduino to your laptop via USB cable for serial monitoring
4. Open Serial Monitor (115200 baud rate) to observe sweep progress
5. Control the sweep using GPIO pins from the Raspberry Pi

#### GPIO Control:
The sweep is controlled via GPIO pins connected to the Raspberry Pi:
- **PIN_LOSET (Pin 6)**: Program/advance to next frequency step
- **PIN_RESET (Pin 7)**: Reset sweep to band start frequency  
- **PIN_CALIB (Pin 8)**: Select band (LOW=Band A, HIGH=Band B)

#### Frequency Bands:
The sweep operates on two predefined frequency bands:
- **Band A**: 650.0 - 850.0 MHz, step size: 2.0 MHz
- **Band B**: 902.6 - 957.6 MHz, step size: 0.2 MHz

#### Configuration:
Edit these parameters in the code to customize frequency bands:
```cpp
static const double A_MIN  = 650.0, A_MAX  = 850.0, A_STEP  = 2.0;
static const double B_MIN  = 902.6, B_MAX  = 957.6, B_STEP  = 0.2;
```

## Hardware Setup

### Required Components:
- Arduino (Uno, Nano, or compatible)
- ADF4351 Local Oscillator module
- SPI connections between Arduino and ADF4351

### Wiring:

#### Arduino to ADF4351 Module (SPI):
- **MOSI** → ADF4351 DATA pin
- **SCK** → ADF4351 CLK pin  
- **SS** (Pin 10) → ADF4351 LE pin
- **Pin 2** → ADF4351 CE pin (Chip Enable)
- **GND** → ADF4351 GND
- **VCC** → ADF4351 VCC (3.3V or 5V depending on module)

#### Arduino to Raspberry Pi (GPIO - for Sweep Controller only):
- **Pin 6** → Raspberry Pi GPIO (LOSET - Program/Advance signal)
- **Pin 7** → Raspberry Pi GPIO (RESET - Reset to band start)
- **Pin 8** → Raspberry Pi GPIO (CALIB - Band select)
- **GND** → Raspberry Pi GND

#### Arduino to Laptop:
- **USB Cable** → For serial communication and power

## LORegisterLibrary

Both programs use the updated `LORegisterLibrary` located in `CLibraries/LORegisterLibrary/`:

### Key Features:
- Full ADF4351 datasheet compliance
- Comprehensive register calculation
- Error checking and validation
- Support for both integer-N and fractional-N modes
- Configurable reference frequency and dividers

### Library Files:
- `LORegisterLibrary.h` - Header with data structures and function declarations
- `LORegisterLibrary.cpp` - Implementation following ADF4351 specifications
- `library.properties` - Arduino library metadata
- `LORegisterLibrary.zip` - Compiled library package

## Installation

1. **Install the Library:**
   - Copy the `LORegisterLibrary` folder to your Arduino libraries directory, or
   - Install the `LORegisterLibrary.zip` file through Arduino IDE Library Manager

2. **Upload Programs:**
   - Open either Arduino program in Arduino IDE
   - Select your board and port
   - Upload the program

3. **Serial Monitor:**
   - Set baud rate to 115200
   - Use for program interaction and monitoring

## Troubleshooting

### Common Issues:
- **No serial output:** Check baud rate (should be 115200)
- **Frequency not changing:** Verify SPI wiring connections
- **Compilation errors:** Ensure LORegisterLibrary is properly installed
- **Invalid frequency:** Check that frequency is within ADF4351 operating range

### ADF4351 Frequency Range:
- **Fundamental Output:** 2200-4400 MHz
- **Divided Output:** 137.5-4400 MHz (with output dividers)

## Support

For issues or questions:
- Check serial monitor output for error messages
- Verify hardware connections
- Ensure frequency values are within valid ranges
- Review ADF4351 datasheet for additional specifications

## Version History

- **v2.0** - Updated with new LORegisterLibrary following ADF4351 datasheet
- **v1.0** - Original implementation (obsolete)