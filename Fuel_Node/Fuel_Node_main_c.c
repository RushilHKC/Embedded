#include "ti_msp_dl_config.h"
#include "MCANDDF.h"
#include "UART_fuel_Driver.h"

#include <stdio.h>

/* Global variables */
static volatile bool g_is_tx_msg_ready;  // Flag indicating if the transmission message is ready

/* Constants */
#define TX_MSG_SIZE 6                // Size of the CAN and UART message
#define CAN_MSG_IDENTIFIER 0x04      // CAN identifier for the message
#define UART_FIFO_SIZE 6             // UART FIFO size for data transmission

/* Transmit Data */
static uint8_t g_tx_msg_data[TX_MSG_SIZE] = { 'F', 'U', 'E', 'L' };  // Transmit message data

/* Function Prototypes */
static void UART_Init(void);         // Function to initialize UART communication
static void CAN_Init(void);          // Function to initialize CAN communication
static void CAN_Transmit(void);      // Function to transmit data via CAN
static void process_interrupt(void); // Function to handle GPIO interrupts

/* Main function */
int main(void)
{
    DL_MCAN_TxBufElement tx_msg;     // Message structure for CAN transmission

    /* Initialize UART and CAN configurations */
    UART_Init();
    CAN_Init();

    /* Enable GPIO interrupts */
    NVIC_EnableIRQ(GPIO_SWITCHES_INT_IRQN);

    /* Main loop */
    while (1) {
        g_is_tx_msg_ready = false;    // Reset transmission flag

        /* Wait until the button is pressed to send the message */
        while (!g_is_tx_msg_ready) {
            __WFE();  /* Wait for event (put CPU in low-power mode) */
        }

        /* Print received data from buffer (assumed global variable) */
        for (int i = 0; i < bufferIndex; i++) {
            printf("%u\n", RxData[i]);  // Print each received data byte
        }

        /* Transmit data via UART and CAN */
        DL_UART_fillTXFIFO(UART0, &g_tx_msg_data[0], UART_FIFO_SIZE); // Fill UART FIFO with data
        CAN_flushRxBuf();   // Flush CAN receive buffer
        CAN_Transmit();     // Transmit the data via CAN
    }

    /* Return with success */
    return 0;
}

/* UART initialization function */
static void UART_Init(void)
{
    // Initialize UART with specified parameters
    UART_INIT(UART0, GPIOA, GPIOA, DL_GPIO_PIN_11, DL_GPIO_PIN_10);
    UART_COMMUNICATION_CONFIG(DL_UART_MAIN_CLOCK_LFCLK,
                              DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1,
                              DL_UART_MAIN_PARITY_NONE,
                              DL_UART_MAIN_WORD_LENGTH_8_BITS,
                              DL_UART_MAIN_STOP_BITS_ONE);
}

/* CAN initialization function */
static void CAN_Init(void)
{
    CAN_config();  // Configure CAN settings
    // Initialize CAN message frame (commented out for now)
    // CAN_initMsgFrame(&tx_msg, CAN_MSG_IDENTIFIER, TX_MSG_SIZE, &g_tx_msg_data[0]);
}

/* Interrupt handler function for GPIO */
void GROUP1_IRQHandler(void)
{
    // Check which interrupt is pending in group 1
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        case GPIO_SWITCHES_INT_IIDX:
            // Check the specific GPIO interrupt source
            switch (DL_GPIO_getPendingInterrupt(GPIO_SWITCHES_PORT)) {
                case DL_GPIO_IIDX_DIO21:      // Check for the specific switch
                    g_is_tx_msg_ready = true; // Set flag to indicate message readiness
                    break;
                default:
                    break; // Ignore other interrupts
            }
            break;
        default:
            break; // Ignore other groups
    }
}
