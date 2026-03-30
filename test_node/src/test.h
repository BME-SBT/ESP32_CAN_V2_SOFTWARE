#ifndef TEST_H
#define TEST_H

#include <stdint.h>

/**
 * @file test.h
 * @brief Test node - simulates throttle node with fault injection.
 *
 * Frame layout:
 *   data[0] = throttle percentage (0-100)
 *   data[1] = status byte (STATUS_VALID / STATUS_ADC_OUT_OF_RANGE / STATUS_CAN_ERROR)
 *   data[2..6] = reserved (zero)
 *   data[7] = rolling message counter
 *
 * NOTE: motor_can.c does not currently check data[1] or data[7],
 * so BAD_STATUS and WRONG_COUNTER modes will not trigger a motorbox fault.
 */
 
typedef enum {
    TEST_MODE_NORMAL = 0,       // Valid throttle at 50Hz
    TEST_MODE_INVALID_THROTTLE, // Throttle = 200 (out of 0-100 range, confirm with team)
    TEST_MODE_BAD_STATUS,       // status = 0, not validated by motorbox currently
    TEST_MODE_WRONG_COUNTER,    // counter jumps, not validated by motorbox currently
    TEST_MODE_TIMEOUT,          // Stop sending -> motorbox watchdog triggers after 500ms
    TEST_MODE_SLOW,             // 200ms period, still under watchdog limit
    TEST_MODE_JITTER,           // Random extra delay, may approach watchdog limit
    TEST_MODE_RANDOM,           // Fully random 8-byte payload
    TEST_MODE_COUNT             // Total number of modes - used for cycling
} test_mode_t;

// Initialize test node 
void test_init(void);

#endif
