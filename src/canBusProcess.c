/**
 * Edward ZHANG, 20171101
 * @file    canBusProcess.c
 * @brief   CAN driver configuration file
 * @reference   RM2017_Archive
 */
#include "ch.h"
#include "hal.h"

#include "canBusProcess.h"

volatile Encoder_canStruct _encoder[CHASSIS_MOTOR_NUM];

/*
 * 1M baud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 * TODO
 */
static const CANConfig cancfg = {
  CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
  CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
  CAN_BTR_TS1(8) | CAN_BTR_BRP(2)
};

#define CAN_FILTER_NUM 28U
static CANFilter canfilter[CAN_FILTER_NUM];

volatile Encoder_canStruct* can_getEncoder(void)
{
  return _encoder;
}


#define CAN_ENCODER_RADIAN_RATIO    7.669904e-4f    // 2*M_PI / 0x2000
static void can_processEncoder
  (volatile Encoder_canStruct* cm, const CANRxFrame* const rxmsg)
{
  uint16_t prev_angle = cm->angle_rotor_raw;

  chSysLock();
  cm->updated = true;
  cm->angle_rotor_raw = (uint16_t)(rxmsg->data8[0]) << 8 | rxmsg->data8[1];
  cm->speed_rpm       = (int16_t)(rxmsg->data8[2]) << 8 | rxmsg->data8[3];
  cm->current_raw     = (int16_t)(rxmsg->data8[4]) << 8 | rxmsg->data8[5];
  cm->temperature     = (uint8_t)rxmsg->data8[6];

  if      (cm->angle_rotor_raw - prev_angle >  CAN_ENCODER_RANGE / 2) cm->round_count--;
  else if (cm->angle_rotor_raw - prev_angle < -CAN_ENCODER_RANGE / 2) cm->round_count++;

  cm->total_ecd = cm->round_count * CAN_ENCODER_RANGE + cm->angle_rotor_raw;
  cm->radian_angle = cm->total_ecd * CAN_ENCODER_RADIAN_RATIO;

  chSysUnlock();
}

static void can_processEncoderMessage(const CANRxFrame* const rxmsg)
{
    switch(rxmsg->SID)
    {
        case CHASSIS_MOTOR_FL_EID:
            can_processEncoder(&_encoder[FL_WHEEL], rxmsg);
            break;
        case CHASSIS_MOTOR_FR_EID:
            can_processEncoder(&_encoder[FR_WHEEL], rxmsg);
            break;
        case CHASSIS_MOTOR_BR_EID:
            can_processEncoder(&_encoder[BR_WHEEL], rxmsg);
            break;
        case CHASSIS_MOTOR_BL_EID:
            can_processEncoder(&_encoder[BL_WHEEL], rxmsg);
            break;
        //TODO process the rest of motor encoder feedback

        default:break;
    }
}

/*
 * Receiver thread.
 */
static THD_WORKING_AREA(can_rx1_wa, 256);
static THD_FUNCTION(can_rx, p) {

  CANDriver* canp = (CANDriver*)p;
  event_listener_t el;
  CANRxFrame rxmsg;

  (void)p;
  chRegSetThreadName("can receiver");
  chEvtRegister(&canp->rxfull_event, &el, 0);
  while(!chThdShouldTerminateX())
  {
    if (chEvtWaitAnyTimeout(ALL_EVENTS, TIME_MS2I(100)) == 0)
      continue;
    while (canReceive(canp, CAN_ANY_MAILBOX,
                      &rxmsg, TIME_IMMEDIATE) == MSG_OK)
    {
      can_processEncoderMessage(&rxmsg);
    }
  }
  chEvtUnregister(&canp->rxfull_event, &el);
}

/*
 * @brief              Send motor current cmd using CAN driver
 * @param[in] cand     Pointer to CANDriver object we are currently using
 * @param[in] cmx_iq   Current (Torque) cmd of motor
 *
 * @notapi
 */
void can_motorSetCurrent(
  const uint16_t EID,
  const int16_t cm1_iq,
  const int16_t cm2_iq,
  const int16_t cm3_iq,
  const int16_t cm4_iq)
{
    CANTxFrame txmsg;

    txmsg.IDE = CAN_IDE_STD;
    txmsg.EID = EID;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    chSysLock();

    //TODO arange data bytes for motor current control
    txmsg.data8[0] = (uint8_t)(cm1_iq >> 8);
    txmsg.data8[1] = (uint8_t)cm1_iq;

    txmsg.data8[2] = (uint8_t)(cm2_iq >> 8);
    txmsg.data8[3] = (uint8_t)cm2_iq;

    txmsg.data8[4] = (uint8_t)(cm3_iq >> 8);
    txmsg.data8[5] = (uint8_t)cm3_iq;

    txmsg.data8[6] = (uint8_t)(cm4_iq >> 8);
    txmsg.data8[7] = (uint8_t)cm4_iq;

    chSysUnlock();

    canTransmit(&CAND1, CAN_ANY_MAILBOX, &txmsg, TIME_MS2I(100));
}


void can_processInit(void)
{
  memset((void *)_encoder, 0, sizeof(Encoder_canStruct)*CHASSIS_MOTOR_NUM);

  uint8_t i;
  for (i = 0; i < CAN_FILTER_NUM; i++)
  {
    canfilter[i].filter = i;
    canfilter[i].mode = 0; //CAN_FilterMode_IdMask
    canfilter[i].scale = 1; //CAN_FilterScale_32bit
    canfilter[i].assignment = 0;
    canfilter[i].register1 = 0;
    canfilter[i].register2 = 0;
  }
  canSTM32SetFilters(&CAND1, 14, CAN_FILTER_NUM, canfilter);

  canStart(&CAND1, &cancfg);
  /*
   * Starting the transmitter and receiver threads.
   */
  chThdCreateStatic(can_rx1_wa, sizeof(can_rx1_wa), NORMALPRIO + 7,
                    can_rx, (void *)&CAND1);
}
