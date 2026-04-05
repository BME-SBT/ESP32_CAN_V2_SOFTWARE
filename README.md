# Throttle node

This is the throttle node for the ESP32_CAN_V2_SOFTWARE project. It reads the throttle sensor, checks that the signal looks sane, and sends the result over CAN.

### What it does

- Reads the throttle position from the ADC.
- Turns the raw sensor value into a 0 to 100 percent throttle value.
- Marks bad readings if the sensor looks disconnected or out of range.
- Sends the throttle data over CAN at 50 Hz.
- Uses a short watchdog timeout so the task does not hang around too long if something goes wrong.

### How it is put together

- `throttle_node/main/main.c` starts the application and launches the throttle task.
- `throttle_node/components/throttle_core/` has the throttle logic.
- `throttle_node/components/can_manager/` holds the shared pin definitions used by this node.
- `common/` holds shared CAN protocol definitions and the CAN wrapper.

### Main flow

1. The app boots in `main.c`.
2. `init_hardware()` sets up the ADC and CAN driver.
3. `read_and_validate_throttle()` reads the sensor and checks the value.
4. `can_transmit_task()` packages the data and sends it on the bus.
5. If CAN fails, the code keeps track of the error and tries recovery when the bus is off.

### Shared CAN data

The throttle node uses the shared CAN ID definitions from `common/can_protocol.h`, so the protocol names stay the same across the project.

### Build

This is an ESP-IDF project. Build it the usual way from the throttle node folder.

### Notes

- `can_error_counter` is kept for now, but it is not used yet.
- `read_and_validate_throttle()` is part of the throttle core API.