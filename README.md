# STM32 Multi-Sensor Anomaly Detection System

Embedded system built on STM32 that integrates multiple sensors to detect
environmental anomalies based on configurable thresholds and signal patterns.

---

## Overview
This project demonstrates low-level embedded firmware development, sensor
integration, and system-level design using an STM32 microcontroller.

The system reads data from multiple sensors and performs real-time evaluation
to flag abnormal conditions.

---

## Hardware Components
- STM32 microcontroller
- Environmental sensors (e.g. temperature, humidity, motion, EM/analog inputs)
- Communication interfaces (I2C / ADC / GPIO)
- On-board indicators (LED / buzzer / UART output)

---

## System Architecture
- Sensor data acquisition via I2C / ADC
- Threshold-based detection logic
- Event indication via GPIO / serial output

*(Block diagram in /docs)*

---

## Firmware Details
- Bare-metal / HAL-based STM32 firmware
- Modular sensor drivers
- Main loop with periodic sampling
- Interrupts and timers (if used)

---

## Results & Testing
- Verified sensor readings via serial logs
- Tested detection thresholds under controlled conditions
- Logged anomaly events for validation

---

## Future Improvements
- Sensor fusion / filtering
- Adaptive thresholds
- Data logging to external storage
- Wireless communication

---

## Skills Demonstrated
- Embedded C
- STM32 peripherals
- Sensor integration
- System design and validation
