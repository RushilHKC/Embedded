//OK
//Author: Arnav Khanade
//Date:18/10/2024
#include "ti_msp_dl_config.h"
#include"MCANDRIVER.h"
#include<stdio.h>
#include <UART_Driver.h>


volatile bool error;
#define TXSIZE (1)
#define IDENTIFIER (0x04)

int main(void)
{
    DL_MCAN_TxBufElement txMsg;

    UART_INIT(UART0, GPIOA, GPIOA, DL_GPIO_PIN_11, DL_GPIO_PIN_10);
    UART_COMMUNICATION_CONFIG(DL_UART_MAIN_CLOCK_LFCLK, DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1, DL_UART_MAIN_PARITY_NONE, DL_UART_MAIN_WORD_LENGTH_8_BITS, DL_UART_MAIN_STOP_BITS_ONE);
    CAN_config();

    while (1) {
        gUartServiceInt = false;

        /* Waits until new sensor signal,to send the message*/
        while (gUartServiceInt == false)
            ;

        CAN_initMsgFrame(&txMsg,IDENTIFIER,TXSIZE,&gRXBuff[0]);
        CAN_transmitMsg(&txMsg);
        UART_flushBuffer();
        CAN_flushRxBuf();


    }
}

