#ifndef MOTOR_CAN_H
#define MOTOR_CAN_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initializes the CAN hardware via wrapper and spawns the RX task.
 */
void motor_can_init(void);

/**
 * @brief Fetches and validates current throttle from the CAN cache.
 * @param out_fault Pointer set to true if a data fault occurs.
 * @return uint8_t Validated throttle value (0 if a fault is active).
 */
uint8_t motor_can_get_throttle(bool *out_fault);

/**
 * @brief Fetches the system enable state (Deadman & Motor Switch).
 * @return bool True if system is enabled, false otherwise.
 */
bool motor_can_get_enable(void);

/**
 * @brief Checks if motor temperature exceeds the maximum allowed limit.
 * @return bool True if the temperature is too high, false otherwise.
 */
bool motor_can_is_temp_high(void);

/**
 * @brief Evaluates if the CAN communication watchdog has timed out.
 * @return bool True if communication is lost, false if active.
 */
bool motor_can_is_timeout(void);

/**
 * @brief Transmits motor telemetry data over the CAN bus.
 * @param rpm Current motor RPM.
 * @param current Motor controller current.
 * @param torque Motor torque.
 */
void motor_can_send_data(int16_t rpm, int16_t current, int16_t torque);

/**
 * @brief Transmits motor temperatures over the CAN bus.
 * @param m_temp Motor temperature.
 * @param c_temp Controller temperature.
 */
void motor_can_send_temps(int16_t m_temp, int16_t c_temp);

#endif // MOTOR_CAN_H
