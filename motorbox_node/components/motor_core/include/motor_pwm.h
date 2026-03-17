/**
 * @file motor_pwm.h
 * @brief Hardware Abstraction Layer for Motor PWM output.
 */

#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stdint.h>

/**
 * @brief Initializes the MCPWM peripheral, timers, operators, and generators.
 */
void motor_pwm_init(void);

/**
 * @brief Sets the motor throttle.
 * @param throttle_val Throttle value from MIN_THROTTLE to MAX_THROTTLE.
 */
void motor_pwm_set_throttle(uint8_t throttle_val);

/**
 * @brief Safely stops the motor by setting the PWM duty cycle to 0.
 */
void motor_pwm_stop(void);

#endif // MOTOR_PWM_H
