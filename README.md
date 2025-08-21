# STM32F103CB Basic Blink Project

## Description

This repository contains a simple embedded project for the STM32F103CBT6 microcontroller (WeAct version of the Blue Pill board). The project blinks the integrated LED on pin PB2, allows changing the blinking pace via the integrated KEY button on pin PA0, and uses UART to send debugging information, including LED status and current blink pace.

The project uses a bare-metal approach with custom clock and UART implementations, leveraging submodules for STM32 base configurations.

## Features

- LED blinking on pin PB2 (WeAct Blue Pill integrated LED).
- Integrated KEY button on PA0 to change blinking speed.
- UART for debugging, sending LED status and blink pace.

## Repository Structure

- **src/**: Source code directory
  - `main.c`: Main program logic for initialization and blinking loop.
  - `clock.c` / `clock.h`: Clock configuration and timing functions.
  - `uart.c` / `uart.h`: UART initialization and debugging output functions.
- `STM32-base`: Submodule providing base STM32 templates and makefiles.
- `STM32-base-STM32Cube`: Submodule for STM32Cube-related configurations.

## Requirements

- STM32F103CBT6 development board (WeAct Blue Pill).
- ARM GCC toolchain (arm-none-eabi-gcc).
- Make utility.
- A flashing tool (e.g., `stm32flash`).
- A UART communication program (e.g., Minicom) configured with the correct baud rate (here `115200`).

## Setup

1. Clone the repository with submodules:

   ```
   git clone --recursive https://github.com/YerimB/STM32F103CB-blink.git
   ```

   Or, if already cloned:

   ```
   git submodule update --init --recursive
   ```

## Building

Run the following command in the project root:

```
make
```

This will compile the source code using the settings from the submodules and produce firmware binaries (e.g., .elf, .bin).

## Flashing

Use a flashing tool like `stm32flash` to flash the board. For example:

```
sudo stm32flash -v -w bin/stm32_bin_image.bin /dev/ttyUSB0
```

Replace `/dev/ttyUSB0` with the appropriate serial port for your setup (e.g., `/dev/ttyACM0` or equivalent).

## Usage

1. Flash the firmware to the microcontroller using your flashing tool.
2. Power on the WeAct Blue Pill board.
3. The LED on pin PB2 should start blinking at a default pace.
4. Press the integrated KEY button to change the blinking speed.
5. Connect to UART (baud rate 115200, pins PA9-TX, PA10-RX) using a UART communication program like Minicom to view LED status and blink pace. For example:

   ```
   minicom -D /dev/ttyUSB0 -b 115200
   ```

   Ensure the serial port and baud rate match your setup.

## Configuration

- Clock: Configured in `clock.c` for system clock setup.
- UART: Implemented in `uart.c`, using USART1 for debugging output (sends LED status and blink pace, no command reception).
- GPIO: LED on PB2 and KEY button defined in `main.c`.

For customizations, edit the source files in `src/` and rebuild.

## Acknowledgments

- Based on STM32-base for build system and templates.
- STM32-base-STM32Cube for Cube integrations.

For more details, refer to the source code comments or submodule documentation.
