# embedded_exercise
# STM32F103 Nucleo Board Project

## Overview
This project is designed for the STM32F103 microcontroller on a Nucleo development board. It implements an embedded system to manage temperature and voltage monitoring with thresholds and controls via UART communication. The system uses ADC readings to monitor and control hardware behavior, and it provides UART feedback to a connected terminal or device.

### Features
- Temperature and voltage monitoring using ADC.
- Real-time control over temperature and voltage thresholds.
- UART communication for receiving commands and sending responses.
- Timer-based periodic updates and PWM control.

### Hardware Used
- **Microcontroller**: STM32F103 (on Nucleo board).
- **Peripherals**:
  - **ADC1**: Used for temperature and voltage measurements.
  - **Timers**: TIM1, TIM2, and TIM3 are used for control and periodic tasks.
  - **UART2**: Communication interface for receiving commands and sending responses.
  
### Software Components
- **STM32 HAL**: Used for peripheral initialization and management.
- **ADC DMA**: Direct Memory Access (DMA) is used for ADC data transfer.
- **Timers**: Configured for periodic interrupts to handle real-time tasks.
- **UART**: Used for command input and response output.

## Setup

1. **Hardware Setup**:
   - Nucleo board with STM32F103.
   - Connect the UART interface to a PC or terminal for command input/output.
   - The temperature sensor should be connected to the ADC channels (CHANNEL_1).
   - The voltage sensor should be connected to the ADC channels (CHANNEL_4).

2. **Software Requirements**:
   - STM32CubeIDE or a compatible IDE.
   - STM32 HAL libraries installed.
   - A terminal program (e.g., PuTTY, Tera Term) to communicate with the board.

## Building the Project

1. **Clone the repository** (if applicable):
    ```bash
    git clone <your-repository-url>
    ```

2. **Open the project in STM32CubeIDE**:
   - Open STM32CubeIDE and import the project into the workspace.

3. **Build the Project**:
   - Click on the build icon in STM32CubeIDE to compile the project.

4. **Flash the Program**:
   - Connect the Nucleo board to your PC using the USB cable.
   - Use STM32CubeIDE to flash the program onto the board.

## Command Set

The following commands can be sent via UART to control the system:

### Temperature Control
- **TEMPERATURE_ON**: Turns on temperature monitoring.
- **TEMPERATURE_OFF**: Turns off temperature monitoring.
- **TEMPERATURE_SET_THRESHOLD=<value>**: Sets the temperature threshold in Celsius (e.g., `TEMPERATURE_SET_THRESHOLD=73.00`).

### Voltage Control
- **VOLTAGE_ON**: Turns on voltage monitoring.
- **VOLTAGE_OFF**: Turns off voltage monitoring.
- **VOLTAGE_SET_THRESHOLD=<value>**: Sets the voltage threshold in Volts (e.g., `VOLTAGE_SET_THRESHOLD=8.35`).

### UART Response
- **CMD_OK**: Sent when a command is successfully processed.
- **CMD_ERROR**: Sent when an invalid command is received or there is an error processing the command.

### Example Commands:
- `TEMPERATURE_ON`
- `TEMPERATURE_SET_THRESHOLD=73.00`
- `VOLTAGE_ON`
- `VOLTAGE_SET_THRESHOLD=8.35`

## Code Walkthrough

- **main.c**:
    - Initializes peripherals including ADC, UART, and timers.
    - The `HAL_UART_RxCpltCallback` function processes commands from the UART input.
    - The `HAL_TIM_PeriodElapsedCallback` manages periodic updates for monitoring temperature and voltage.
    - The ADC readings are smoothed and processed to trigger alerts if the thresholds are exceeded.

- **ADC Handling**:
    - The temperature and voltage values are read from the ADC channels and smoothed using a filter coefficient to ensure stability.
    - The raw ADC values are then scaled to their corresponding units (Celsius for temperature, Volts for voltage).

- **Timer Handling**:
    - Timer interrupts (TIM1, TIM2, and TIM3) are used for periodic updates and PWM control.
    - The temperature and voltage statuses are updated periodically and sent over UART.

## Notes
- Ensure that the Nucleo board's power and connections are correctly set up before flashing the firmware.
- The UART baud rate is set to 115200, so ensure your terminal program is configured accordingly.

## Troubleshooting
- **No response from the board**: Ensure the board is correctly connected via USB and that the UART settings (baud rate, word length, stop bits) match the configuration in `main.c`.
- **Invalid command errors**: Double-check the command syntax, ensuring it matches exactly as defined in the "Command Set" section.

## License

This project is licensed under the terms of the MIT License.

