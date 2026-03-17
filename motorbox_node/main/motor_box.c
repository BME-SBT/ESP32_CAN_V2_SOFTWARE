// Motor control, with fault, watchdog, and safe startup
// SolarBoat Electronics team -- Kovács Marcell 2026. 02. 17.

const static char* TAG = "MOTOR_BOX";

// --- start includes ---
#include "solar.h"
#include "can_manager.h"

#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/mcpwm_prelude.h"
// --- end includes ---


// --- start definitions ---
#define SETUP_GROUP                  0        // group id of all components of mcpwm
#define MAX_THROTTLE                 255      // Maximum value for throttle
#define MIN_THROTTLE                 0        // Minimum value for throttle

// JAVÍTVA: Valós értékek standard PWM-hez (50Hz, 20ms periódus, 1us felbontás)
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1 tick = 1 microsecund
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks = 20ms = 50Hz frissítés
#define MOTOR_PWM_GPIO               1        // TODO: Állítsd be a valós GPIO pinre!

#define WD_TIMEOUT_MS                500      // watchdog timeout threshold in miliseconds
#define MAX_MOTOR_TEMP               120      // Maximum allowed tempreture for motor operation

#define SAFE_STARTUP_WAIT_MS         2000     // safe startup time in ms (throttle in 0 pos for this amount of time before engine can start)
// --- end definitions ---


// --- start Frame defines ---
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
// --- end Frame defines ---


// --- start Resource allocation and Initialization ---
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
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operator, timer));

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

  // PWM Logika: Számláló indulásakor HIGH
  ESP_ERROR_CHECK(
      mcpwm_generator_set_action_on_timer_event(
        generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(
          MCPWM_TIMER_DIRECTION_UP,
          MCPWM_TIMER_EVENT_EMPTY, 
          MCPWM_GEN_ACTION_HIGH
  )));

  // PWM Logika: Amikor eléri a comparator értékét, LOW
  ESP_ERROR_CHECK(
      mcpwm_generator_set_action_on_compare_event(
        generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
          MCPWM_TIMER_DIRECTION_UP,
          comparator,
          MCPWM_GEN_ACTION_LOW
  )));

  // JAVÍTVA: Timer elindítása (enélkül nem generál jelet!)
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
}
// --- end Resource allocation and Initialization ---


// --- Can reciever, validation ---
enum valid_errors {
  LOWER_BOUND, UPPER_BOUND, LENGHT, NONE
};
volatile enum valid_errors g_valid_fault = NONE;

uint8_t validate_throttle(twai_message_t *control_frame) {
  can_manager_lock_frame(control_frame);

  // Lenght validation
  if (control_frame->data_length_code < 1) { // 8 helyett 1-re javítva, ha csak 1 byte a gáz
    ESP_LOGE(TAG, "Throttle data lenght is less then expected, raising fault flag");
    g_valid_fault = LENGHT;
    can_manager_unlock_frame(control_frame);
    return 0;
  }
  
  uint8_t raw_val = control_frame->data[0];
  can_manager_unlock_frame(control_frame);

  // Range validation (A raw_val < MIN_THROTTLE felesleges, ha MIN_THROTTLE = 0 és uint8_t a típus, 
  // de bennehagyjuk általános esetre, int konverzióval a warning elkerülésére)
  if ((int16_t)raw_val < MIN_THROTTLE) {
    ESP_LOGE(TAG, "Throttle value is LESS then expected, raising fault flag");
    g_valid_fault = LOWER_BOUND;
    return 0;
  }
  
  if (raw_val > MAX_THROTTLE) {
    ESP_LOGE(TAG, "Throttle value is MORE then expected, raising fault flag");
    g_valid_fault = UPPER_BOUND;
    return 0;
  }

  // Minden rendben
  if (g_valid_fault != NONE) {
    g_valid_fault = NONE;
  }
  return raw_val;
}

// --- Watchdog ---
static uint32_t last_valid_msg_time = 0;
bool is_communication_timed_out() { 
  uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
  return (now - last_valid_msg_time) > WD_TIMEOUT_MS;
}

// --- Motor tempreture controll ---
bool is_motor_temp_high(twai_message_t *motor_temps_frame) {
  can_manager_lock_frame(motor_temps_frame);

  if (motor_temps_frame->data_length_code < 1) {
    ESP_LOGE(TAG, "Temp data lenght is less then expected");
    can_manager_unlock_frame(motor_temps_frame);
    return false; // vagy true, attól függően mennyire szigorú a hiba
  }
  
  uint8_t motor_temp = motor_temps_frame->data[0];
  can_manager_unlock_frame(motor_temps_frame);

  if (motor_temp > MAX_MOTOR_TEMP) {
    ESP_LOGE(TAG, "Motor temp of %u is greater then the maximum %u", motor_temp, MAX_MOTOR_TEMP);
    return true;
  }
  return false;
}

// --- Main motor control task loop ---
void ts_motorControlLoop(void *pvParameters) {
  // frame registration
  ESP_ERROR_CHECK(can_manager_register_frame(&controll_frame, true, true));
  ESP_ERROR_CHECK(can_manager_register_frame(&motor_data_frame, true, true));
  ESP_ERROR_CHECK(can_manager_register_frame(&motor_temps_frame, true, true));

  // can manager initialization
  ESP_ERROR_CHECK(can_manager_init(&g_config, &t_config, &f_config, 100, "MOTOR_CTRL"));

  enum states {
    STARTUP_STATE, OPERATION_STATE, CAN_OVER_UART_STATE
  };
  enum states state = STARTUP_STATE;

  uint8_t target_throttle = 0;
  bool waiting_for_zero = false;
  uint32_t zero_throttle_start_time = 0;

  setup_mcpwm();

  while (1) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;

    target_throttle = validate_throttle(&controll_frame);
    bool temp_is_high = is_motor_temp_high(&motor_temps_frame);

    // TODO: A last_valid_msg_time frissítését érdemes a can_manager RX callbackjébe tenni,
    // de ideiglenesen frissítjük, ha érvényes gázadatot látunk (vagy ha a can_manager ad rá API-t)
    if (g_valid_fault == NONE) {
       last_valid_msg_time = now; 
    }

    bool comms_timeout = is_communication_timed_out();

    bool system_fault_active = (g_valid_fault != NONE) || temp_is_high || comms_timeout;

    if (system_fault_active && state == OPERATION_STATE) {
      ESP_LOGE(TAG, "Motor entering STARTUP_STATE due to system fault!");
      state = STARTUP_STATE;
    }

    switch (state) {

      case STARTUP_STATE: // startup, error state
        // Motor biztonsági lekapcsolása (0% PWM)
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0)); 

        if (!system_fault_active) {
          if (target_throttle == 0) {
            if (!waiting_for_zero) {
              zero_throttle_start_time = now;
              waiting_for_zero = true;
            } else if (now - zero_throttle_start_time >= SAFE_STARTUP_WAIT_MS) {
              state = OPERATION_STATE;
              waiting_for_zero = false;
              ESP_LOGI(TAG, "System armed, motor ready for operation");
            }
          } else {
            waiting_for_zero = false; // Gázt adtak a várakozás alatt
          }
        } else {
          waiting_for_zero = false; // Hiba van jelen, nem indulhat a timer
        }
        break;

      case OPERATION_STATE: // normal operation
        // PWM kitöltési tényező kiszámítása (0-255 skálázása a Timer periódusra)
        // Megjegyzés: Ha standard RC jelet használsz, a "0 throttle" általában 1ms (1000 tick), 
        // a max pedig 2ms (2000 tick). Ha ez ipari motorvezérlő, akkor maradhat a 0-100% (0-20000 tick).
        uint32_t compare_val = ((uint32_t)target_throttle * SERVO_TIMEBASE_PERIOD) / MAX_THROTTLE;

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, compare_val)); 
        break;

      case CAN_OVER_UART_STATE: // no can communication so switch over to uart
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, 0)); // Motor kikapcs
        // TODO the actual can over uart stuff
        break;
    }

    // A ciklus futási sebessége (10ms = 100Hz frissítés a simább gázreakcióért)
    vTaskDelay(pdMS_TO_TICKS(10)); 
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
