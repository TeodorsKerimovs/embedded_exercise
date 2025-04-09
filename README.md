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


