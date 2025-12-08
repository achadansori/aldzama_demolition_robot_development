# Aldzama Demolition Robot Development

Development project for a demolition robot control system using LoRa communication for long-range applications. This system consists of two main components: transmitter (controller) and control (robot).

## 📋 Project Description

This system is designed to control a demolition robot wirelessly using LoRa technology, enabling safe and reliable remote operation. The project implements two-way communication between the controller unit and the robot with various control modes.

## 🏗️ Project Structure

### 1. Transmitter Demolition Robot
**Platform:** STM32F401CEU6  
**Description:** Controller/remote control unit that sends commands to the robot.

#### Main Features:
- **Joystick Control**: Analog input for robot movement control
- **Switch Interface**: Buttons for special operation modes
- **LoRa Communication**: Long-range wireless communication module
- **USB CDC**: USB interface for debugging and configuration
- **ADC Input**: Analog value reading from joystick

#### Software Components:
- `joystick.c/h` - Joystick driver and reading logic
- `switch.c/h` - Button input handler
- `lora.c/h` - LoRa communication driver
- `usb.c/h` - USB CDC interface
- `adc.c/h` - Analog to Digital Converter for joystick
- `var.c/h` - Global variables and definitions

#### Used Peripherals:
- ADC - Joystick analog reading
- USART - Serial communication with LoRa module
- USB Device (CDC) - Debugging interface
- GPIO - Digital input/output
- TIM - Timer for control timing

---

### 2. Control Demolition Robot
**Platform:** STM32F407VGT6  
**Description:** Control unit on the robot that receives commands and drives actuators.

#### Main Features:
- **LoRa Receiver**: Receives commands from transmitter
- **PWM Control**: Drives motors and actuators
- **USB CDC**: Monitoring and debugging
- **SPI Interface**: Communication with additional sensors

#### Software Components:
- `lora.c/h` - LoRa receiver driver
- `control.c` - Motor and actuator control logic
- `pwm.c` - PWM signal generator for motors
- `spi.c/h` - SPI interface
- `tim.c/h` - Timer for PWM and scheduling

#### Used Peripherals:
- USART - Communication with LoRa module
- TIM (PWM Mode) - Motor control
- SPI - Sensor/actuator interface
- USB Device (CDC) - Debugging
- GPIO - Control signals

---

## 🔧 Hardware Specifications

### Transmitter
- **Microcontroller**: STM32F401CEU6 (ARM Cortex-M4, 84 MHz)
- **Flash**: 512 KB
- **RAM**: 96 KB
- **LoRa Module**: EBYTE E220-900T22D (Via UART)
- **Antenna**: 900 MHz SMA antenna
- **Input**: Analog Joystick (dual-axis), Multiple switches
- **Interface**: USB Type-C (CDC Virtual COM Port)

### Control Unit (Robot)
- **Microcontroller**: STM32F407VGT6 (ARM Cortex-M4, 168 MHz)
- **Flash**: 1 MB
- **RAM**: 192 KB
- **LoRa Module**: EBYTE E220-900T22D (Via UART)
- **Antenna**: 900 MHz SMA antenna
- **Motor Driver**: PWM output (multiple channels)
- **Interface**: USB Type-C, SPI peripherals

---

## 📡 LoRa Communication

### LoRa Module: EBYTE E220-900T22D

#### Module Specifications:
- **Model**: EBYTE E220-900T22D
- **Chipset**: Semtech LLCC68
- **Frequency**: 850.125 ~ 930.125 MHz
- **Transmission Power**: 22 dBm (160 mW)
- **Sensitivity**: -136 dBm @ SF12, BW125kHz
- **Interface**: UART (TTL 3.3V)
- **Antenna**: SMA connector
- **Range**: Up to 5 km (line of sight, open area)

#### Communication Parameters:
- **UART Baud Rate**: 9600 bps (default) / configurable
- **Air Data Rate**: 0.3 ~ 62.5 kbps
- **Bandwidth**: 125/250/500 kHz
- **Spreading Factor**: SF5 ~ SF12
- **Coding Rate**: 4/5, 4/6, 4/7, 4/8
- **Mode**: Transparent/Fixed transmission

#### Pin Configuration:
- **M0, M1**: Mode selection pins
  - M0=0, M1=0: Normal mode (transmission)
  - M0=1, M1=0: WOR transmitting
  - M0=0, M1=1: WOR receiving
  - M0=1, M1=1: Configuration mode
- **RXD, TXD**: UART communication
- **AUX**: Status indicator

### Data Protocol:
Data packet format is designed for efficiency and reliability with error checking and acknowledgment.

---

## 🚀 Getting Started

### Build & Flash

#### Using STM32CubeIDE:
1. Import both projects into workspace
2. Build project (Ctrl+B or Project → Build All)
3. Connect STM32 via ST-Link
4. Flash binary (Run → Debug or F11)

#### Using Command Line (Make):
```bash
# Build Transmitter
cd transmitter_demolition_robot/Debug
make all

# Build Control
cd control_demoliiton_robot/Debug
make all
```

#### Flash using st-flash:
```bash
# Flash Transmitter
st-flash write transmitter_demolition_robot.bin 0x8000000

# Flash Control
st-flash write control_demoliiton_robot.bin 0x8000000
```

---

## 🛠️ Development Setup

### Prerequisites:
- **STM32CubeIDE** v1.9.0 or newer
- **STM32CubeMX** (optional for reconfiguration)
- **ARM GCC Toolchain**
- **ST-Link Driver & Utilities**
- **USB CDC Driver** (for Windows)

### Library Dependencies:
- STM32F4xx HAL Driver
- STM32 USB Device Library (CDC Class)
- CMSIS Core & Device Support

---

## 📝 Configuration

### Transmitter Configuration:
Edit `Core/Inc/var.h` or `Core/Inc/lora_config.h` for:
- Joystick calibration values
- LoRa frequency and power
- Control sensitivity
- Switch mapping

### Control Configuration:
Edit `Core/Inc/lora.h` for:
- LoRa reception parameters
- PWM frequency and duty cycle limits
- Motor direction mapping
- Safety limits

---

## 🔍 Debugging

### Serial Monitor (USB CDC):
Both units provide debug output via USB CDC:
- **Windows**: COMx port
- **Linux**: /dev/ttyACMx
- **Baud Rate**: 115200 (virtual, does not affect USB)

### Monitoring:
```bash
# Linux
screen /dev/ttyACM0 115200

# or using minicom
minicom -D /dev/ttyACM0
```

---

## ⚠️ Safety Features

- Automatic timeout on communication loss
- Emergency stop via dedicated switch
- Motor current limiting
- Watchdog timer
- Signal validation and checksum

---

## 📄 Important Files

### Transmitter:
- `Core/Src/main.c` - Main program
- `Core/Src/joystick.c` - Joystick logic
- `Core/Src/lora.c` - LoRa transmission
- `.ioc` - STM32CubeMX configuration file

### Control:
- `Core/Src/main.c` - Main program
- `Core/Src/control.c` - Motor control logic
- `Core/Src/lora.c` - LoRa reception
- `Core/Src/pwm.c` - PWM generation
- `.ioc` - STM32CubeMX configuration file

---

## 🐛 Troubleshooting

### LoRa not connecting:
- Check frequency configuration (850-930 MHz)
- Ensure LoRa parameters (SF, BW, CR) match on both devices
- Check mode pins M0 and M1 (should be in Normal Mode for communication)
- Verify AUX pin status (HIGH = ready, LOW = busy)
- Check UART connection to E220 module
- Ensure UART baud rate matches module configuration
- Check module power supply (3.3V, minimum 500mA during transmit)

### Joystick not responsive:
- Re-calibrate ADC values
- Check analog input connections
- Verify threshold values in configuration

### Motors not moving:
- Verify PWM signals with oscilloscope
- Check motor driver power supply
- Verify motor driver enable pin connections

---

## 📚 References

- [STM32F401 Datasheet](https://www.st.com/resource/en/datasheet/stm32f401ce.pdf)
- [STM32F407 Datasheet](https://www.st.com/resource/en/datasheet/stm32f407vg.pdf)
- [STM32 HAL Documentation](https://www.st.com/resource/en/user_manual/um1725-description-of-stm32f4-hal-and-lowlayer-drivers-stmicroelectronics.pdf)
- [EBYTE E220-900T22D Datasheet](https://www.ebyte.com/en/product-view-news.html?id=939)
- [EBYTE E220 User Manual](https://www.ebyte.com/en/pdf-down.html?id=939)
- LLCC68 LoRa Transceiver Datasheet (Semtech)

---

## 👥 Contributors

- **Riset and Development Demolition Team**
---

## 📜 License

See `LICENSE` files in respective driver directories for third-party component license information.

---

## 📞 Support

For questions and support, contact Demolition

---

**Version**: 1.0  
**Last Updated**: 2025  
**Status**: Active Development
