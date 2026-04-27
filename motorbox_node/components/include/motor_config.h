/**
 * @file motor_config.h
 * @brief Central configuration file for the Motor Box.
 * Bridges the global solar.h definitions with motor-specific settings.
 */

#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#include <stdint.h>

// --- MCPWM Hardware Configuration ---
#define SETUP_GROUP                  0          // Group ID of all components of MCPWM
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000    // 1MHz, 1 tick = 1 microsecond
#define SERVO_TIMEBASE_PERIOD        20000      // 20000 ticks = 20ms = 50Hz refresh rate
#define MOTOR_PWM_PIN                PIN_GPIO_1 // Selected from solar.h definitions

// --- Motor Parameters ---
#define MAX_THROTTLE                 255        // Maximum value for throttle
#define MIN_THROTTLE                 0          // Minimum value for throttle
#define MAX_MOTOR_TEMP               120        // Maximum allowed temperature for motor operation

// --- Safety & Timings ---
#define WD_TIMEOUT_MS                500        // Watchdog timeout threshold in milliseconds
#define SAFE_STARTUP_WAIT_MS         2000       // Milliseconds the throttle must be 0 before arming
#define CONTROL_LOOP_PERIOD_MS       10         // Main control loop delay (10ms = 100Hz)

#endif // MOTOR_CONFIG_H
