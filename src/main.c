/*
    ChibiOS - Copyright (C) 2006..2018 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @brief This file is based on the MAPLE MINI example from ChibiOS
 *
 * @file main.cpp
 * @author Alex Au
 * @date 2018-09-11
 */

#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "canBusProcess.h"

#include <stdlib.h>

static RC_Ctl_t* rc;

static int16_t motor_output[4];        //Torque command for motors
static int16_t motor_speed_sp[4];      //speed set-point for motor

//These are the parameters of PID controller
const float chassis_kp = 0.5;     //Proportional
const float chassis_ki = 0.06;     //Integration
const float chassis_kd = 0.5;     //Derivative

static void drive_meccanum(const int16_t strafe, const int16_t drive, const int16_t rotation)
{
   motor_speed_sp[FL_WHEEL] =  strafe + drive + rotation;
   motor_speed_sp[FR_WHEEL] =  strafe - drive + rotation;
   motor_speed_sp[BR_WHEEL] = -strafe - drive + rotation;
   motor_speed_sp[BL_WHEEL] = -strafe + drive + rotation;




  /*
        TODO: drive the four meccanum wheel individually by using meccanum wheel kinematics
        eg. motor_speed_sp[FL_WHEEL] = ?? strafe ?? drive ?? rotation;
        + or -, depends on required speed direction
    */

}

static int16_t pid_control(const int16_t setPoint, const int16_t current, float* error_int, float* error_der, int16_t* previous_error)
{
    int16_t output = 0;
    int16_t error = 0;

    const int dt = 2;

    error = setPoint - current;
    *error_int += error*dt;
    *error_der = (error - *previous_error)/dt;

    output = chassis_kp * error + chassis_ki * *error_int + chassis_kd * *error_der;

    float MAX_ERROR_INT = 10000.0;

    if (*error_int > MAX_ERROR_INT)
         *error_int = MAX_ERROR_INT;
    if (*error_int < -MAX_ERROR_INT)
         *error_int = -MAX_ERROR_INT;


    *previous_error = error;

    if(output > 10000)
         output = 10000;
    else if(output < -10000)
         output = -10000;

    return output;


}
    /*
        NOTE: Limit your maximum integrated error is often useful
        if(*error_int > YOUR_MAX_ERROR_INT)
            *error_int = YOUR_MAX_ERROR_INT;
        else if(*error_int < -YOUR_MAX_ERROR_INT)
            *error_int = -YOUR_MAX_ERROR_INT;
    */

    //=============================================================
/*
    previous_error = error;

    if(output > 10000)
        output = 10000;
    else if(output < -10000)
        output = -10000;

    return output;
}
*/

static THD_WORKING_AREA(motor_ctrl_thread_wa,512);
static THD_FUNCTION(motor_ctrl_thread, p)
{
    (void) p;
	int16_t strafe = 0, drive = 0, rotation = 0;   //move direction for chassis
    Encoder_canStruct* encoder = can_getEncoder(); //Pointer to motor encoder feedbakc
    static float motor_error_int[4]; //error integrators for the four motors
    static float motor_error_der[4];
    static int16_t previous_error[4];

	while(true)
	{

            //NOTE: A special question for you: how we decide this value
            //"12000/1320"
	  if ((rc->channel0 - 1024) > -45 && (rc->channel0 -1024) < 45) rc->channel0 = 1024;
	  if ((rc->channel1 - 1024) > -45 && (rc->channel1 -1024) < 45) rc->channel1 = 1024;
	  if ((rc->channel2 - 1024) > -45 && (rc->channel2 -1024) < 45) rc->channel2 = 1024;

        strafe = (rc->channel0 - 1024)*12000.0f/1320.0f;
        drive = (rc->channel1 - 1024)*12000.0f/1320.0f;
        rotation = (rc->channel2 - 1024)*12000.0f/1320.0f;
        drive_meccanum(strafe,drive, rotation);



        /*
            TODO PID speed controller function for motor
        */
        for(int i = 0; i < 4; i++)
        {
            motor_output[i] = pid_control(motor_speed_sp[i],
                                          (encoder + i)->speed_rpm,
                                          &motor_error_int[i],
                                          &motor_error_der[i],
                                          &previous_error[i]);
        }
            /*
                TODO set motor current
            */
        can_motorSetCurrent(
              0x200,
              motor_speed_sp[FL_WHEEL] + motor_output[FL_WHEEL],
              motor_speed_sp[FR_WHEEL] + motor_output[FR_WHEEL],
              motor_speed_sp[BR_WHEEL] + motor_output[BR_WHEEL],
              motor_speed_sp[BL_WHEEL] + motor_output[BL_WHEEL]);



		chThdSleepMilliseconds(2);
	}
}

/*
 * Application entry point.
 */
int main(void)
{

    /*
    * System initializations.
    * - HAL initialization, this also initializes the configured device drivers
    *   and performs the board-specific initializations.
    * - Kernel initialization, the main() function becomes a thread and the
    *   RTOS is active.
    */
    halInit();
    chSysInit();

    RC_init();
    can_processInit();

    rc = RC_get();

    chThdCreateStatic(motor_ctrl_thread_wa, sizeof(motor_ctrl_thread_wa),
		  	  	  	 NORMALPRIO, motor_ctrl_thread, NULL);

    /*
    * Normal main() thread activity
    */
    while (true)
    {
        palTogglePad(GPIOA, GPIOA_LED);
        chThdSleepMilliseconds(500);
    }
}

