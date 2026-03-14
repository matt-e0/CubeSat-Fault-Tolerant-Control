# Electrical

Overview of the electronics and electrical systems.

<p align="center">
  <img src="/Resources/circuit_image.svg" width="500"/>
  <br/>
  <em>Circuit Diagram of Electronics</em>
</p>

Component selection and electrical design for the CubeSat ADCS hardware prototype.

## Bill of Materials

| Component | Part | Quantity | Reason for Selection |
|-----------|------|----------|----------------------|
| Microcontroller | Teensy 4.1 | 1 | 600MHz ARM Cortex-M7, hardware floating point, IntervalTimer for precise 100Hz control loop, built-in SD card slot |
| IMU | Bosch BNO055 | 1 | Outputs quaternions directly via I2C at 100Hz - onboard sensor fusion removes the need to run a separate attitude estimator on the Teensy |
| Motor driver | STM32 B-G431B ESC1 | 4 | STM32G4 with dedicated motor control hardware (advanced timers, op-amps for current sensing), supported by ST Motor Control SDK with FOC implementation |
| Motors | BLDC with magnetic encoder | 4 | Brushless for longevity and low friction, magnetic encoder provides quadrature A/B feedback to the ESC for closed-loop speed control |
| Power | 3S LiPo | 1 | 9.6–12.6V operating range compatible with ESC input voltage, high discharge rate suitable for motor transients |

## Power architecture

The 3S LiPo supplies the ESCs directly at battery voltage (9.6–12.6V). The Teensy is powered via a 5V regulator from the battery rail in standalone operation.

A 1200µF 50V electrolytic capacitor bank (2x 100µF, 1x 1000µF in parallel) is placed across the main power rail close to the ESC inputs to absorb inrush transients on connection and back-EMF spikes during direction changes.

ESC voltage fault thresholds are configured to match the 3S operating range:
- Over-voltage threshold: 15V
- Under-voltage threshold: 9V

## Communication

| Interface | Used for |
|-----------|----------|
| I2C (400kHz) | BNO055 IMU → Teensy |
| PWM (digital) | Teensy → each ESC (one pin per ESC) |
| SPI | Teensy → microSD card (built-in on Teensy 4.1) |
| USB Serial | Debugging and telemetry during development |

## PWM signal specification

The Teensy drives each ESC with a standard RC-style PWM signal generated in software:

| Pulse width | Command |
|-------------|---------|
| 1500µs | Neutral / arming |
| 1550–1940µs | Forward (1000–3000 RPM) |
| 1450–1060µs | Reverse (1000–3000 RPM) |
| 50Hz | Signal frequency |

Ideally the motor drivers would have been interfaced using the available UART ports or CAN bus interface. However, due to a lack of documentation and time limitations, a modified version of the existing PWM control mode was used. The modified firmware for the ESCs cannot be shared due to license agreements.

## Key design decisions

**BNO055 over MPU-6050** - the BNO055 runs its own sensor fusion algorithm onboard and outputs calibrated quaternions directly, offloading computation from the Teensy and ensuring the 100Hz control loop is not bottlenecked by attitude estimation. Ideally, sensors suited for space would be added to transform this project to a complete Attitude Determination and Control System (ADCS) since the IMU would lose functionality in a zero-g environment. However, for the purposes of demonstration it is adequate.

**Teensy 4.1 over Arduino/ other MCU** - the 600MHz clock and hardware FPU handle the floating point matrix math in the ASMC with comfortable headroom. The `IntervalTimer` peripheral fires the control loop interrupt precisely at 100Hz without drift.

**B-G431B ESC1 over generic ESC** - the STM32-based ESC allows direct firmware modification. Bidirectional control, arming sequence, fault thresholds, and stop duration were all customised for reaction wheel operation - none of which would be possible on a sealed commercial ESC. It also features encoder feedback for precise motor control.

**Brushless DC Motors with AS5047P Magnetic Angle Encoder** - the reaction wheels require high performance (moderate torque, high rpm) due to the volume constraints of the CubeSat design. Brushed DC motors seem an obvoius choice, due to their ease of use, however brushed motors are known to arc in the vacuum of space, rapidly degrading their brushes until mechanical failure. For this reason a 1404 size drone BLDC motor was chosen, as it offers high performance while being suitable for the space environment. - Similarly the encoder could not rely on traditional optical or physical sensing methods due to the high top speed of the fly wheel and the risk of interfering radiation or dust. A magnetic encoder navigates both of these issues.
