/**
 * Edward ZHANG, Terry ZENG, 20180304
 * @file    dbus.c
 * @brief   Dbus driver and decoder with keyboard and mouse support and safe lock
 */

#include "ch.h"
#include "hal.h"

#include "dbus.h"

static uint8_t rxbuf[DBUS_BUFFER_SIZE];
static RC_Ctl_t RC_Ctl;

static thread_reference_t uart_dbus_thread_handler = NULL;

static rc_state_t rc_state = RC_UNCONNECTED;

/**
 * @brief   Decode the received DBUS sequence and store it in RC_Ctl struct
 */
static void decryptDBUS(void)
{
	RC_Ctl.channel0 = ((rxbuf[0]) | (rxbuf[1]<<8)) & 0x07FF;
	RC_Ctl.channel1 = ((rxbuf[1]>>3) | (rxbuf[2]<<5)) & 0x07FF;
	RC_Ctl.channel2 = ((rxbuf[2]>>6) | (rxbuf[3]<<2) | ((uint32_t)rxbuf[4]	<<10)) & 0x07FF;
	RC_Ctl.channel3 = ((rxbuf[4]>>1) | (rxbuf[5]<<7)) & 0x07FF;
	RC_Ctl.s1 = ((rxbuf[5] >> 4)& 0x000C) >> 2;
	RC_Ctl.s2 = ((rxbuf[5] >> 4)& 0x0003);
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp)
{
    (void)uartp;
    chSysLockFromISR();
    chThdResumeI(&uart_dbus_thread_handler, MSG_OK);
    chSysUnlockFromISR();
}


/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg =
{
    NULL,NULL,rxend,NULL,NULL,	//Call-back functions
    100000, 					//Baudrate
    USART_CR1_PCE | USART_CR1_M,				//EVEN Parity bit
    USART_CR2_LBDL,
    0
};

/**
 * @brief   Return the RC_Ctl struct
 */
RC_Ctl_t* RC_get(void)
{
  return &RC_Ctl;
}

static void RC_reset(void)
{
    RC_Ctl.channel0 = 1024;
    RC_Ctl.channel1 = 1024;
    RC_Ctl.channel2 = 1024;
    RC_Ctl.channel3 = 1024;

    RC_Ctl.s1 =0;
    RC_Ctl.s2 = 0;
}

#define  DBUS_INIT_WAIT_TIME_MS      4U
#define  DBUS_WAIT_TIME_MS         100U
static THD_WORKING_AREA(uart_dbus_thread_wa, 512);
static THD_FUNCTION(uart_dbus_thread, p)
{
    (void)p;
    chRegSetThreadName("uart dbus receiver");

    msg_t rxmsg;
    systime_t timeout = TIME_MS2I(DBUS_INIT_WAIT_TIME_MS);
    uint32_t count = 0;

    while(!chThdShouldTerminateX())
    {
        uartStopReceive(UART_DBUS);
        uartStartReceive(UART_DBUS, DBUS_BUFFER_SIZE, rxbuf);

        chSysLock();
        rxmsg = chThdSuspendTimeoutS(&uart_dbus_thread_handler, TIME_INFINITE);
        chSysUnlock();

        if(rxmsg == MSG_OK)
        {
            if(!rc_state)
            {
                timeout = TIME_MS2I(DBUS_WAIT_TIME_MS);
                rc_state = RC_UNLOCKED;
            }
            else
            {
                chSysLock();
                decryptDBUS();
                chSysUnlock();
            }
        }
        else
        {
            rc_state = RC_UNCONNECTED;
            RC_reset();
            timeout = TIME_MS2I(DBUS_INIT_WAIT_TIME_MS);
        }
    }
}

/**
 * @brief   Initialize the RC receiver
 */
void RC_init(void)
{
    RC_reset();

    uartStart(UART_DBUS, &uart_cfg);
    dmaStreamRelease(*UART_DBUS.dmatx);

    chThdCreateStatic(uart_dbus_thread_wa, sizeof(uart_dbus_thread_wa),
                    NORMALPRIO + 7,
                    uart_dbus_thread, NULL);
}
