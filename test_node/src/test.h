/**
 * @file test.h
 * @brief Test node - simulates throttle node with fault injection.
 *
 * === Frame layout (matches throttle.c) ===
 *   data[0] = throttle percentage (0-100)
 *   data[1] = status byte (STATUS_VALID / STATUS_ADC_OUT_OF_RANGE / STATUS_CAN_ERROR)
 *   data[2..6] = reserved (zero)
 *   data[7] = rolling message counter
 *
 * === Wokwi pin mapping ===
 *   CAN TX: PIN_CAN_TX = GPIO_NUM_2  (solar.h)
 *   CAN RX: PIN_CAN_RX = GPIO_NUM_3  (solar.h)
 *
 * === Known limitations ===
 *   - motor_can.c does not check data[1] (status) or data[7] (counter),
 *     so BAD_STATUS and WRONG_COUNTER modes will not trigger a motorbox fault.
 *   - Throttle range mismatch: throttle.c sends 0-100, motorbox MAX_THROTTLE=255,
 *     so INVALID_THROTTLE (value=200) may not trigger a fault.
 */

#ifndef TEST_H
#define TEST_H



// NOTE: The solar.h include path needs to be checked!
// #include "../../throttle_node/components/can_manager/include/solar.h"
#include "solar.h"

#include "esp_err.h"
#include <stdint.h>

// ---------------------------------------------------------------------------
// CAN ID - sourced from solar.h, which includes can_protocol.h, where Control_ID is defined
// Control_ID = 0x100 = 256
// Mirrors throttle.h: rename Control_ID to THROTTLE_CAN_ID for clarity
// ---------------------------------------------------------------------------
#ifndef THROTTLE_CAN_ID
#define THROTTLE_CAN_ID Control_ID
#endif // THROTTLE_CAN_ID


// Status bitmasks - match throttle.h
#define STATUS_VALID            0x01
#define STATUS_ADC_OUT_OF_RANGE 0x02
#define STATUS_CAN_ERROR        0x04

// Timing
#define TX_PERIOD_MS     20     // Normal send period: 20ms = 50Hz, matches real throttle node
#define MODE_DURATION_MS 20000  // 20 seconds per mode for Wokwi analysis

 
typedef enum {
    TEST_MODE_NORMAL = 0,       // Valid throttle at 50Hz
    TEST_MODE_INVALID_THROTTLE, // Throttle = 200 (out of 0-100 range sent by real throttle node)
    TEST_MODE_BAD_STATUS,       // status = 0, not validated by motorbox currently
    TEST_MODE_WRONG_COUNTER,    // counter jumps, not validated by motorbox currently
    TEST_MODE_TIMEOUT,          // Stop sending -> motorbox watchdog triggers after 500ms
    TEST_MODE_SLOW,             // 200ms period, still under watchdog limit
    TEST_MODE_JITTER,           // Random extra delay, may approach watchdog limit
    TEST_MODE_RANDOM,           // Fully random 8-byte payload
    TEST_MODE_ADC_SWEEP,        // Simulates ADC values across all ranges, validates mapping and status bits
    TEST_MODE_COUNT             // Total number of modes - used for cycling
} test_mode_t;

// Initialize test node 
void test_init(void);

#endif
