/**
 * @file motor_control.h
 * @brief High-level system logic and state machine controller.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

/**
 * @brief Creates the FreeRTOS task that runs the motor control loop.
 */
void motor_control_start_task(void);

#endif // MOTOR_CONTROL_H
