# STM32 Target Position Controller with LiDAR and PID

This project implements a target position controller using an STM32F103C8T6 microcontroller. It receives distance and angle data from a LiDAR sensor (LS02A), calculates the target position in Cartesian coordinates, measures the current position using an encoder, and uses a PID algorithm to control a motor to reach the desired target.

---

## 🔧 Hardware Configuration

- **MCU**: STM32F103C8T6 ("Blue Pill")
- **LiDAR**: LS02A (sends distance and angle via UART1)
- **Motor**: JGB37-545 DC 12V 107RPM with built-in encoder
- **Motor Driver**: TB6612FNG
- **UART Debug**: UART2 used for debugging/logging
- **PWM Output**: TIM3 (for motor speed control)
- **Encoder Input**: TIM1 (encoder mode)
- **PID Timer**: TIM2 (periodic interrupt for PID control loop)

---

## 📡 System Overview

1. **Receive** distance and angle data via UART1 (from LiDAR).
2. **Convert** polar coordinates `(r, θ)` to Cartesian `(x, y)`.
3. **Compute** distance to a predefined target position.
4. **Read** the current motor position using encoder via TIM1.
5. **Use PID** control (triggered by TIM2 interrupt) to calculate a PWM signal.
6. **Drive** the motor forward/backward using PWM via TIM3 and TB6612FNG.
7. **Debug** information is sent to UART2 for live monitoring.

---

## 🔌 Pin Mapping

| Peripheral    | Function                  | STM32 Pin |
|---------------|---------------------------|-----------|
| UART1         | LiDAR Input               | PA9 (TX), PA10 (RX) |
| UART2         | Debug Output              | PA2 (TX), PA3 (RX)  |
| TIM1          | Encoder Input             | PA8 (CH1), PA9 (CH2) |
| TIM2          | PID Loop Timer (interrupt)| -         |
| TIM3          | PWM Output to Motor       | PB0       |
| TB6612FNG     | Motor Driver Input        | PWM + DIR Pins (custom define) |

---

## 📁 Project Structure

| File                  | Description |
|-----------------------|-------------|
| `main.c`              | Main loop and initialization |
| `uartParser.c/h`      | Handles LiDAR UART data parsing |
| `PIDController.c/h`   | PID control logic |
| `motorController.c/h` | Motor direction and PWM control |
| `Makefile`            | Compilation script |
| `grad_app.ioc`        | STM32CubeMX project file |

---

## 🧠 Algorithm Description

```text
LiDAR Data --> [UART Parser] --> (x, y)
                               |
                           [Distance to Target]
                               |
                  Encoder <-- [PID Controller] --> PWM --> Motor
```

- **Position Conversion**:  
  `x = r * cos(θ)`, `y = r * sin(θ)`  
  Target distance = `sqrt((x - xt)^2 + (y - yt)^2)`

- **PID Output**:  
  Used to generate PWM signal for motor speed control. Direction determined by whether current distance is less than or greater than target.

---

## 🧪 Build & Flash Instructions

1. Install `arm-none-eabi-gcc` and `make`.
2. Connect the STM32 board via ST-Link or USB-UART.
3. Run the following command:
   ```bash
   make all
   ```
4. Flash the binary using ST-Link or `stm32flash`:
   ```bash
   st-flash --reset write build/grad_app.bin 0x8000000
   ```

---

## 📊 Debug Output

- Position, target, and PID output values are logged via UART2 at 115200 baud.
- Use serial terminal (e.g., PuTTY, CoolTerm) to monitor data.

---

## ✅ Future Improvements

- Implement Kalman/Median filtering for noisy data.
- Use circular buffer or DMA for UART RX.
- Tune PID parameters with auto-tuning or GUI.
- Add obstacle avoidance (if using 2D LiDAR scan).
- Migrate to FreeRTOS for better task scheduling.

---


## 🧑‍💻 Author

- **Name**: [Nguyen Phung Duc]
- **University**: [Hanoi University of Science and Technology]
- **Project**: Graduation Thesis / Final Year Project (2025)
