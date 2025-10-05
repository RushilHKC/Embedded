#include "ti_msp_dl_config.h"
#include "stdio.h"
#include <stdbool.h>
#include <string.h>

#define UART_TX_PACKET_SIZE (50)  // Size for sending UART data
#define EXPECTED_DLC_LED (1)  //Expected Data Length Code

/* CAN IDs for Fuel and Temperature Nodes */
#define CAN_ID_FUEL_NODE (0x4)
#define CAN_ID_TEMPERATURE_NODE (0x3)

/* Define thresholds */
#define FUEL_THRESHOLD (10)  // Fuel threshold in liters
#define TEMPERATURE_THRESHOLD (100)  // Temperature threshold in degrees Celsius

/* Define timing and retry limits */
#define MESSAGE_TIMEOUT_MS (1000)  // Timeout in ms for message reception
#define MAX_RETRY_ATTEMPTS (3)
#define TIMER_INTERVAL_MS (100)  // Timer interval in ms

/* Global Variables */
volatile bool gServiceInt = false;
volatile bool gCANMessageReceived = false;  // Flag indicating a new CAN message was received
volatile uint32_t messageElapsedTime = 0;  // Time elapsed since the last message received
DL_MCAN_RxBufElement gRxMsg;               // Global storage for the received CAN message

/* Error and warning messages */
#define FUEL_WARNING "Warning: Low Fuel Level!"
#define TEMP_WARNING "Warning: High Temperature!"
#define INVALID_DATA_WARNING "Warning: Invalid Data Received!"
#define CAN_ID_ERROR "Error: Unknown CAN ID!"
#define CAN_LENGTH_ERROR "Error: Invalid CAN Message Length!"
#define BUS_ERROR "Error: CAN Bus Error!"
#define MESSAGE_LOSS_ERROR "Error: Message Lost!"
#define RETRY_FAILED_ERROR "Error: Retry Failed!"
#define CORRUPT_DATA_ERROR "Error: Data Corruption Detected!"

/* Function Prototypes */
static void processCANMessage(DL_MCAN_RxBufElement *rxMsg);
static void sendDataOverUART(const char *message);
static bool validateCANMessage(DL_MCAN_RxBufElement *rxMsg);
static void handleTimeout(void);
static void retryTransmission(void);
void MCAN0_INST_IRQHandler(void);
void TIMER_0_INST_IRQHandler(void);
void receive(void);
int main(void);

/* Main function */
int main(void)
{
    /* System initialization */
    SYSCFG_DL_init();

    /* Enable CAN and Timer interrupts */
    NVIC_EnableIRQ(MCAN0_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);

    /* Initialize timer for message timeout tracking */
    DL_TimerA_startCounter(TIMER_0_INST);

    /* Main loop - process CAN message when flag is set */
    while (1)
    {
        /* Check for message timeout */
        handleTimeout();

        /* Process the CAN message if received */
        if (gCANMessageReceived)
        {
            /* Validate and process the CAN message */
            if (validateCANMessage(&gRxMsg))
            {
                processCANMessage(&gRxMsg);
                messageElapsedTime = 0;  // Reset elapsed time on valid message
            }

            /* Clear the flag after processing */
            gCANMessageReceived = false;
        }
    }
}

void MCAN0_INST_IRQHandler(void)
{
    DL_MCAN_RxFIFOStatus rxFS;

    /* Check if message received in RX FIFO 0 */
    if ((DL_MCAN_getIntrStatus(MCAN0_INST) & MCAN_IR_RF0N_MASK) == MCAN_IR_RF0N_MASK)
    {
        /* Read message from RX FIFO 0 */
        DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO, 0U, DL_MCAN_RX_FIFO_NUM_0, &gRxMsg);

        /* Acknowledge the reception */
        DL_MCAN_writeRxFIFOAck(MCAN0_INST, DL_MCAN_RX_FIFO_NUM_0, rxFS.getIdx);

        /* Set flag to indicate message is ready to be processed in the main loop */
        gCANMessageReceived = true;

        /* Clear interrupt flag */
        DL_MCAN_clearIntrStatus(MCAN0_INST, MCAN_IR_RF0N_MASK, DL_MCAN_INTR_SRC_MCAN_LINE_1);
    }

}


/* Timer interrupt handler for timeout checks */
void TIMER_0_INST_IRQHandler(void)
{
    // Increment the elapsed message time every timer interrupt
    messageElapsedTime++;
}


/* Validate CAN message data */
static bool validateCANMessage(DL_MCAN_RxBufElement *rxMsg)
{
    uint32_t canID = (rxMsg->id & 0x1FFC0000) >> 18;  // Extract 11-bit standard CAN ID

    /* Check for valid CAN ID */
    if (canID != CAN_ID_FUEL_NODE && canID != CAN_ID_TEMPERATURE_NODE)
    {
        sendDataOverUART(CAN_ID_ERROR);
        return false;
    }

    /* Validate CAN message length (expecting 1 byte of data) */
    if (rxMsg->dlc != EXPECTED_DLC_LED)
    {
        sendDataOverUART(CAN_LENGTH_ERROR);
        return false;
    }

    return true;
}

/* Process the CAN message for fuel level and temperature */
static void processCANMessage(DL_MCAN_RxBufElement *rxMsg)
{
    uint32_t canID = (rxMsg->id & 0x1FFC0000) >> 18;  // Extract 11-bit standard CAN ID
    char dataMessage[UART_TX_PACKET_SIZE];

    switch (canID)
    {
        case CAN_ID_FUEL_NODE:
        {
            uint8_t fuelLevel = rxMsg->data[0];  //fuel level byte
            snprintf(dataMessage, UART_TX_PACKET_SIZE, "Fuel Level: %d L\n", fuelLevel);
            sendDataOverUART(dataMessage);

            /* Send warning if fuel is below the threshold */
            if (fuelLevel < FUEL_THRESHOLD)
            {
                sendDataOverUART(FUEL_WARNING);
            }
            break;
        }
        case CAN_ID_TEMPERATURE_NODE:
        {
            int8_t temperature = rxMsg->data[0];  //temperature byte
            snprintf(dataMessage, UART_TX_PACKET_SIZE, "Temperature: %d \u00B0C\n", temperature);

            sendDataOverUART(dataMessage);

            /* Send warning if temperature is above the threshold */
            if (temperature > TEMPERATURE_THRESHOLD)
            {
                sendDataOverUART(TEMP_WARNING);
            }
            break;
        }
        default:
            /* Unknown CAN ID - Shouldn't happen as validation already checks this */
            break;
    }
}

/* Handle message timeout and display error if no message is received */
static void handleTimeout(void)
{
    if (messageElapsedTime >= (MESSAGE_TIMEOUT_MS / TIMER_INTERVAL_MS))  // Convert timeout to count of timer intervals
    {
        sendDataOverUART(MESSAGE_LOSS_ERROR);
        messageElapsedTime = 0;  // Reset the elapsed time to avoid continuous timeout messages
    }
}

/* Function to send data over UART */
static void sendDataOverUART(const char *message)
{
    size_t length = strlen(message);
    size_t chunkSize = 4;   // Send in 4-byte chunks
    size_t bytesSent = 0;

    while (bytesSent < length)
    {
        size_t bytesToSend = (length - bytesSent < chunkSize) ? (length - bytesSent) : chunkSize;

        /* Transmit the chunk via UART */
        DL_UART_fillTXFIFO(UART_0_INST, (uint8_t *)&message[bytesSent], bytesToSend);

        /* Wait until the TX FIFO is empty (all bytes have been transmitted) */
        while (DL_UART_isBusy(UART_0_INST));

        bytesSent += bytesToSend;
    }
}


