#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#define Control_ID     0x100
#define MotorTemps_ID  0x200
#define MotorData_ID   0x201
#define BMSData_ID     0x300
#define BMSExtra_ID    0x301
#define GPS_ID         0x400
#define Position_ID    0x401

typedef struct {
    int8_t throttle;
    uint8_t enable;
} control_msg_t;

typedef struct {
    int16_t motor_temp;
    int16_t controller_temp;
} motor_temps_msg_t;

typedef struct {
    int16_t rpm;
    int16_t current;
    int16_t torque;
} motor_data_msg_t;

typedef struct {
    int16_t voltage;
    int16_t current;
    uint16_t soc;
} bms_data_msg_t;

typedef struct {
    int16_t temperature;
    int16_t mppt_current;
    int16_t fan_rpm;
} bms_extra_msg_t;

#endif
