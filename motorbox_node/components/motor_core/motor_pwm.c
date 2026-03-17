/**
 * @file motor_pwm.c
 * @brief Implementation of the MCPWM hardware layer.
 */

#include "motor_pwm.h"
#include "motor_config.h"
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"

static const char* TAG = "MOTOR_PWM";

static mcpwm_cmpr_handle_t comparator;
static mcpwm_oper_handle_t operator_handle;

void motor_pwm_init(void) {
    ESP_LOGI(TAG, "Initializing MCPWM...");

    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t timer_conf = {
        .group_id = SETUP_GROUP,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_conf, &timer));

    mcpwm_operator_config_t oper_conf = {
        .group_id = SETUP_GROUP
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_conf, &operator_handle));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator_handle, timer));

    mcpwm_comparator_config_t cmpr_conf = {
        .flags.update_cmp_on_tez = true
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(operator_handle, &cmpr_conf, &comparator));

    mcpwm_gen_handle_t generator;
    mcpwm_generator_config_t gen_conf = {
        .gen_gpio_num = MOTOR_PWM_PIN 
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(operator_handle, &gen_conf, &generator));

    // PWM Logic: HIGH on timer start
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)
    ));

    // PWM Logic: LOW on compare match
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)
    ));

    // Start timer safely with 0 duty cycle
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}

void motor_pwm_set_throttle(uint8_t throttle_val) {
    // Scale 0-MAX_THROTTLE to 0-SERVO_TIMEBASE_PERIOD
    uint32_t compare_val = ((uint32_t)throttle_val * SERVO_TIMEBASE_PERIOD) / MAX_THROTTLE;
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, compare_val)); 
}

void motor_pwm_stop(void) {
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0));
}
