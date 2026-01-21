# STM32 Multi-Sensor Anomaly Detector (B-L475E-IOT01A)

STM32L475 IoT Node project that reads onboard sensors and performs threshold-based anomaly detection with UART telemetry and real-time alerts (LED blink rate + buzzer). Includes an HT16K33 I2C LED matrix display demo.

> Built originally as a “ghost detector” demo — presented here as a multi-sensor embedded anomaly detection system.

---

## Hardware
- **Board:** B-L475E-IOT01A (STM32L475)
- **Sensors (BSP):** accelerometer, gyroscope, magnetometer, temperature, humidity, pressure
- **Outputs:** UART (115200), LED (PB14), buzzer (PD14)
- **Display:** HT16K33 LED matrix via **I2C1 (PB8/PB9)**

---

## Features
- **Mode 0 (Normal):** prints sensor magnitudes over UART every 1s
- **Mode 1 (Detection):**
  - magnetometer magnitude controls LED blink rate
  - buzzer triggers at high magnetic threshold
  - periodic threshold checks for temperature/humidity/pressure/IMU events
- **User button (PC13):**
  - double press toggles mode
  - single press in detection mode toggles active state and logs “event captured”

---

## How it works
1. Initialize HAL, SysTick, UART, GPIO, buzzer, I2C, HT16K33
2. Initialize sensor BSP drivers
3. Main loop runs mode logic + periodic sampling using `uwTick`

---

## Build & Run
Recommended: STM32CubeIDE.
1. Open project
2. Build + flash to B-L475E-IOT01A
3. Open serial monitor at **115200 8N1**

---

## Repo layout
- `firmware/Core/Src/main.c` — main application logic
- `docs/` — diagrams/logs (add screenshots or UART logs here)

---

## Skills demonstrated
Embedded C • STM32 HAL/BSP • UART • EXTI interrupts • I2C • sensor integration • real-time thresholding • validation via telemetry

