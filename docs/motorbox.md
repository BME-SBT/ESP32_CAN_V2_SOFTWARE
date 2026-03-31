# Motorbox Node Documentation

## 1. Overview
The `motorbox_node` acts as the primary hardware interface and safety controller between the `throttle_node` and the physical motor controller. It receives throttle commands and motor status data over the CAN bus, performs safety validations, and generates a corresponding PWM signal to drive the motor. The logic runs in a dedicated FreeRTOS task executing at 100Hz (10ms period).

## 2. Hardware Configuration
* **CAN TX Pin:** GPIO 2 (`PIN_CAN_TX`).
* **CAN RX Pin:** GPIO 3 (`PIN_CAN_RX`).
* **PWM Output Pin:** GPIO 0 (`PIN_GPIO_1`).
* **CAN Baud Rate:** 500 Kbits/s.

## 3. CAN Interface and Data Points
The node utilizes a thread-safe CAN manager to handle communication. It listens to and processes the following CAN IDs:
* **`Control_ID` (0x100):** Expected to contain the throttle value in its first byte.
* **`MotorTemps_ID` (0x200):** Contains the motor temperature in its first byte.
* **`MotorData_ID` (0x201):** Registered for general motor data.

## 4. PWM Signal Generation
The motor control signal is generated using the ESP32 MCPWM peripheral.
* **Timebase/Resolution:** 1 MHz (1 tick = 1 microsecond).
* **Frequency:** 50 Hz (20ms period, requiring 20000 ticks).
* **Scaling:** Throttle values between `MIN_THROTTLE` (0) and `MAX_THROTTLE` (255) are linearly scaled to the 0-20000 compare value range.

## 5. State Machine & Arming Sequence
The node implements a strict state machine to prevent unintentional motor activation.
* **`STARTUP_STATE`:** The default and error-recovery state. PWM output is forcefully set to 0%.
* **Neutral Interlock (Arming):** To transition out of `STARTUP_STATE`, all system faults must be cleared, and the incoming throttle command must be exactly 0 for a continuous duration of 2000 ms (`SAFE_STARTUP_WAIT_MS`). If the throttle is increased during this window, the timer resets.
* **`OPERATION_STATE`:** Active running state. The validated CAN throttle value is applied to the PWM output.
* **`CAN_OVER_UART_STATE`:** A fallback state that currently stops the motor (0% PWM).

## 6. Error Cases and Failsafes
The system continuously evaluates input validity and hardware status. If any of the following faults occur while in `OPERATION_STATE`, the system logs the error and immediately forces a transition to `STARTUP_STATE` (stopping the motor).

#### A. CAN Communication Timeout
* **Trigger:** No valid throttle message (`Control_ID`) is received for more than 500 ms (`WD_TIMEOUT_MS`).
* **Consequence:** The communication timeout flag is set, triggering a global system fault that forces the state machine to `STARTUP_STATE` and drops the PWM to 0%. The node will not re-arm until communication is restored and the 2000 ms neutral interlock is satisfied.

#### B. Invalid Throttle Value (Data Fault)
* **Trigger:** The received `Control_ID` frame has a data length less than 1, or the payload value falls outside the bounds of `MIN_THROTTLE` (0) and `MAX_THROTTLE` (255).
* **Consequence:** The frame is discarded, an error is logged, and a data fault flag is returned. This triggers a system fault, forces the node into `STARTUP_STATE`, and stops the motor.

#### C. Motor Overtemperature
* **Trigger:** The temperature value received via `MotorTemps_ID` exceeds `MAX_MOTOR_TEMP` (120 °C).
* **Consequence:** An overtemperature error is logged and the high-temp flag is set. The global fault evaluator detects this, kicks the system into `STARTUP_STATE`, and halts the motor to allow cooling.

