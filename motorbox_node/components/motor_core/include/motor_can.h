/**
 * @file motor_can.h
 * @brief Data layer for CAN bus communication and validation.
 */

#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initializes the CAN manager and registers all motor-related frames.
 */
void motor_can_init(void);

/**
 * @brief Fetches and validates the current throttle command from the CAN frame.
 * @param out_fault Pointer to a boolean set to true if a data fault is detected.
 * @return Validated throttle value (0 if a fault is active).
 */
uint8_t motor_can_get_throttle(bool *out_fault);

/**
 * @brief Checks if the motor temperature exceeds the maximum allowed limit.
 * @return True if the temperature is too high, false otherwise.
 */
bool motor_can_is_temp_high(void);

/**
 * @brief Evaluates if the CAN communication watchdog has timed out.
 * @return True if communication is lost, false if active.
 */
bool motor_can_is_timeout(void);

#endif // MOTOR_CAN_H
