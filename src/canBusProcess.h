#ifndef _CAN_BUS_PROCESS_H_
#define _CAN_BUS_PROCESS_H_

#include "stdint.h"
#include "stdbool.h"
#include "hal.h"
#include "string.h"

#define CHASSIS_MOTOR_NUM                4U

#define CHASSIS_MOTOR_CTRL_EID        0x200
#define CHASSIS_MOTOR_FL_EID          0x201
#define CHASSIS_MOTOR_FR_EID          0x202
#define CHASSIS_MOTOR_BR_EID          0x203
#define CHASSIS_MOTOR_BL_EID          0x204

#define CAN_ENCODER_RANGE              8192            // 0x2000

typedef enum {
    FL_WHEEL = 0,
    FR_WHEEL = 1,
    BR_WHEEL = 2,
    BL_WHEEL = 3
} wheel_id_t;

typedef struct {
    uint16_t angle_rotor_raw;
    int16_t speed_rpm;
    int16_t current_raw;
    uint8_t temperature;

    int32_t round_count;
    int32_t total_ecd;
    float radian_angle; // Continuous

    bool updated;
} Encoder_canStruct;

volatile Encoder_canStruct* can_getEncoder(void);

void can_processInit(void);
void can_motorSetCurrent(const uint16_t EID,
                         const int16_t cm1_iq,
                         const int16_t cm2_iq,
                         const int16_t cm3_iq,
                         const int16_t cm4_iq);

#endif
