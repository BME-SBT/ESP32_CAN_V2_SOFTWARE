// Motor control, with fault, watchdog, and safe startup
// SolarBoat Electronics team -- Kovács Marcell 2026. 02. 17.

const static char* TAG = "MOTOR_BOX";

// start includes

#include "solar.h"
#include "can_manager.h"

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/mcpwm_prelude.h"

// end includes


// start definitions

#define SETUP_GROUP                  0        // group id of all components of mvpwm
#define MAX_THROTTLE                 255      // Maximum value for throttle
#define MIN_THROTTLE                 0        // Minimum value for throttle
#define SERVO_TIMEBASE_PERIOD        100      // TODO change this to not template value
#define SERVO_TIMEBASE_RESOLUTION_HZ 100      // TODO change this to not template value
#define MOTOR_PWM_GPIO               1        // TODO change this to not template value



#define WD_TIMEOUT_MS                500      // watchdog timeout threshold in miliseconds

#define MAX_MOTOR_TEMP               120      // Maximum allowed tempreture for motor operation (TODO set temp correctly, just placeholder for now)

#define SAFE_STARTUP_WAIT_MS         2000     // safe startup time in ms (throttle in 0 pos for this amount of time before engine can start)

// end definitions


// start Frame defines

// Motor controll frame
static twai_message_t controll_frame = {
      .identifier = Control_ID,
      .extd = 0,
      .rtr = 0,
      .data_length_code = 8,
      .data = {0}
};
// Motor temp frame
static twai_message_t motor_temps_frame = {
      .identifier = MotorTemps_ID,
      .extd = 0,
      .rtr = 0,
      .data_length_code = 8,
      .data = {0}
};

// Motor data frame
static twai_message_t motor_data_frame = {
      .identifier = MotorData_ID,
      .extd = 0,
      .rtr = 0,
      .data_length_code = 8,
      .data = {0}
};

// end Frame defines

// start Resource allocation and Initialization

mcpwm_cmpr_handle_t comparator;
mcpwm_oper_handle_t operator;

void setup_mcpwm() {

  // Timer setup
  mcpwm_timer_handle_t timer;
  mcpwm_timer_config_t timer_conf = {
    .group_id = SETUP_GROUP,
    .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
    .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
    .period_ticks = SERVO_TIMEBASE_PERIOD,
    .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_conf, &timer));

  // Operator setup
  mcpwm_operator_config_t oper_conf = {
    .group_id = SETUP_GROUP
  };
  ESP_ERROR_CHECK(mcpwm_new_operator(&oper_conf, &operator));

  // Comparator setup
  mcpwm_comparator_config_t cmpr_conf = {
    .flags.update_cmp_on_tez = true
  };
  ESP_ERROR_CHECK(mcpwm_new_comparator(operator, &cmpr_conf, &comparator));


  // Generator setup
  mcpwm_gen_handle_t generator;
  mcpwm_generator_config_t gen_conf = {
    .gen_gpio_num = MOTOR_PWM_GPIO 
  };
  ESP_ERROR_CHECK(mcpwm_new_generator(operator, &gen_conf, &generator));

  ESP_ERROR_CHECK(
      mcpwm_generator_set_action_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
          MCPWM_TIMER_DIRECTION_UP,
          MCPWM_TIMER_EVENT_EMPTY, 
          MCPWM_GEN_ACTION_HIGH
  )));

  ESP_ERROR_CHECK(
      mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
          MCPWM_TIMER_DIRECTION_UP,
          comparator,
          MCPWM_GEN_ACTION_LOW
  )));

  /* Not used in the end, might be useful someday...

  // Brake configuration
  mcpwm_soft_fault_config_t soft_fault_conf = {
    .flags.active_high = true
  };
  mcpwm_fault_handle_t fault_handle;
  ESP_ERROR_CHECK(mcpwm_new_soft_fault(&soft_fault_conf, &fault_handle));
  // On fault event force PWM output to LOW
  mcpwm_generator_set_action_on_brake_event(
      generator,
      MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_OST, MCPWM_GENERATOR_ACTION_LOW)
  )
  */
}

// end Resource allocation and Initialization

// Can reciever, validation
enum valid_errors {
  LOWER_BOUND, UPPER_BOUND, LENGHT, NONE
};
volatile enum valid_errors g_valid_fault = NONE;

uint8_t validate_throttle(twai_message_t *control_frame) {

  can_manager_lock_frame(control_frame); // lock frame while reading from it

  // Lenght validation
  if (control_frame->data_length_code < 8) {
    ESP_LOGE(TAG, "Throttle data lenght is less then expected, raising fault flag");
    g_valid_fault = LENGHT;
    return 0;
  }
  uint8_t raw_val = (uint8_t)control_frame->data[0];

  can_manager_unlock_frame(control_frame); // unlock frame

  // Range validation FIXME compiler says its always false
  // lower
  if (raw_val < (uint8_t)MIN_THROTTLE) {
    ESP_LOGE(TAG, "Throttle value is LESS then expected, raising fault flag");
    g_valid_fault = LOWER_BOUND;
    return 0;
  }
  // upper
  if (raw_val > (uint8_t)MAX_THROTTLE) {
    ESP_LOGE(TAG, "Throttle value is MORE then expected, raising fault flag");
    g_valid_fault = UPPER_BOUND;
    return 0;
  }

  // all is good, sending value from frame g->g
  ESP_LOGI(TAG, "THROTTLE FRAME: \t%u", raw_val);
  if (g_valid_fault) {
    g_valid_fault = NONE;
  }
  return raw_val;
}

// Watchdog
static uint32_t last_valid_msg_time = 0;
bool is_communication_timed_out() { // returns true if last_valid_msg_time is larger than WD_TIMEOUT_MS
  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
  return (now - last_valid_msg_time) > WD_TIMEOUT_MS;
}

// Motor tempreture controll
bool is_motor_temp_high(twai_message_t *motor_temps_frame) {
  can_manager_lock_frame(motor_temps_frame); // lock frame while reading from it

  // Lenght validation
  if (motor_temps_frame->data_length_code < 8) {
    ESP_LOGE(TAG, "Throttle data lenght is less then expected, raising fault flag");
  }
  uint8_t motor_temp = (uint8_t)motor_temps_frame->data[0];

  can_manager_unlock_frame(motor_temps_frame); // unlock frame

  if (motor_temp > MAX_MOTOR_TEMP) {
    ESP_LOGE(TAG, "Motor temp of %u is greater then the maximum %u", motor_temp, MAX_MOTOR_TEMP);
    return true;
  }
  return false;
}

/*
 * TODO
 * Read motor data
 */

// Main motor control task loop
void ts_motorControlLoop(void *pvParameters) {
  // frame registration
  ESP_ERROR_CHECK(can_manager_register_frame(&controll_frame, /*tx_enable=*/true, /*rx_enable=*/true));
  ESP_ERROR_CHECK(can_manager_register_frame(&motor_data_frame, /*tx_enable=*/true, /*rx_enable=*/true));
  ESP_ERROR_CHECK(can_manager_register_frame(&motor_temps_frame, /*tx_enable=*/true, /*rx_enable=*/true));

  // can manager initialization
  ESP_ERROR_CHECK(can_manager_init(&g_config, &t_config, &f_config, /*tx_period_ms=*/100, "MOTOR_CTRL"));

  enum states {
    STARTUP_STATE, OPERATION_STATE, CAN_OVER_UART_STATE
  };
  enum states state = STARTUP_STATE;

  uint8_t target_throttle = 1;
  bool controll_frame_recieved = false;
  bool temp_frame_recieved = false;
  bool is_system_cleared = false;
  bool waiting_for_zero = false;
  uint32_t zero_throttle_start_time = 0;

  setup_mcpwm();

  while (1) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    // motor control (throttle) frame reception
    controll_frame_recieved = (twai_receive(&controll_frame, 0) == ESP_OK);
    if (controll_frame_recieved && controll_frame.identifier == THROTTLE_CAN_ID) { // TODO real id of frame
      target_throttle = validate_throttle(&controll_frame);
      last_valid_msg_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
      switch (g_valid_fault) {
        case LOWER_BOUND:
          ESP_LOGE(TAG, "Data from throttle is LESS than accepted!");
          state = STARTUP_STATE;
          break;
        case UPPER_BOUND:
          ESP_LOGE(TAG, "Data from throttle is MORE than accepted!");
          state = STARTUP_STATE;
          break;
        case LENGHT:
          ESP_LOGE(TAG, "Data from throttle is SHORTER then expected!");
          state = STARTUP_STATE;
          break;
        case NONE:
          is_system_cleared = true;
          break;
      }
    }

    // motor temp frame reception
    temp_frame_recieved = (twai_receive(&motor_temps_frame, 0) == ESP_OK);
    if (temp_frame_recieved && motor_temps_frame.identifier == MOTOR_TEMP_ID) { // TODO real id of frame
      last_valid_msg_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
      if (is_motor_temp_high(&motor_temps_frame)) {
        state = STARTUP_STATE;
      } else {
        is_system_cleared = true;
      }
    }

    // watchdog error handling
    if (is_communication_timed_out()) {
      state = STARTUP_STATE; // TODO switch to CAN_OVER_UART_STATE not startup, COUS still needs to be implemented
      ESP_LOGE(TAG, "Watchdog: CAN Timeout");
    } else {
      is_system_cleared = true;
    }

    // state handling for safe startup
    switch (state) {

      case STARTUP_STATE: //startup, error state
        // only proceed to operation s. when throttle is in deadzone for SAFE_STARTUP_WAIT_MS
        // this part is kind of a clusterduck sorry if you have to maintain this : ^(
        if (target_throttle == 0 && is_system_cleared) {

          if (!waiting_for_zero) {
            // start "Timer", might be better if we use a real timer not this bs
            zero_throttle_start_time = now;
            waiting_for_zero = true;

          } else if (now - zero_throttle_start_time >= SAFE_STARTUP_WAIT_MS) {
            // change to oper state and reset
            state = OPERATION_STATE;
            waiting_for_zero = false;
            ESP_LOGI(TAG, "System armed, motor ready for operation");
          }

        } else {
          // reset if throttle changes from 0
          waiting_for_zero = false;
        }
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0)); // Motor kept on 0 while starting up
        break;

      case OPERATION_STATE: //normal operation
        if (!is_system_cleared) { // checking system faults
          state = STARTUP_STATE;
          ESP_LOGE(TAG, "Motor entering STARTUP_STATE due to previous error!");
          break;
        }

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, target_throttle)); // TODO: reverse is not programmed yet, need to know more about the motor controller
        break;

      case CAN_OVER_UART_STATE: // no can communication so switch over to uart
        // TODO the actual can over uart stuff
        break;
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // TODO adjust timings
  }
};


void app_main(void) {
  ESP_LOGI(TAG, "Motor Controller Starting...");

  BaseType_t xReturned = xTaskCreate(
      ts_motorControlLoop,    // Function that implements the task
      "motor_ctrl_task",      // Text name for the task
      4096,                   // Stack size in words
      NULL,                   // Parameter passed into the task
      5,                      // Priority
      NULL                    // Task handle
  );

  if (xReturned != pdPASS) {
      ESP_LOGE(TAG, "Failed to create motor control task!");
  }
}

