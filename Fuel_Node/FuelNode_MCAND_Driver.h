// Global variables: gInterruptLine1Status, gServiceInt, RxData, bufferIndex
#include "ti_msp_dl_config.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <ti/driverlib/dl_common.h>
#include <ti/driverlib/dl_mcan.h>

// Define constants for ID modes
#define ID_MODE_STANDARD (0x0U)
#define ID_MODE_EXTENDED (0x1U)

// Define a suitable buffer size for RxData
#define BUFFER_SIZE 256

// Global variables
volatile uint32_t gInterruptLine1Status = 0;  // Interrupt status for Line 1
volatile bool gServiceInt = false;              // Flag to indicate service interrupt
DL_MCAN_RxBufElement rxMsg;                     // Structure to hold received message
DL_MCAN_RxFIFOStatus rxFS;                      // Structure to hold RX FIFO status
uint8_t RxData[BUFFER_SIZE] = {0};              // Buffer to store received data
volatile uint32_t bufferIndex = 0;              // Index for the RxData buffer


#define POWER_STARTUP_DELAY                                                (16)


#define GPIO_HFXT_PORT                                                     GPIOA
#define GPIO_HFXIN_PIN                                             DL_GPIO_PIN_5
#define GPIO_HFXIN_IOMUX                                         (IOMUX_PINCM10)
#define GPIO_HFXOUT_PIN                                            DL_GPIO_PIN_6
#define GPIO_HFXOUT_IOMUX                                        (IOMUX_PINCM11)
#define CPUCLK_FREQ                                                     32000000


/* Port definition for Pin Group GPIO_SWITCHES */
#define GPIO_SWITCHES_PORT                                               (GPIOB)

/* Defines for USER_SWITCH_1: GPIOB.21 with pinCMx 49 on package pin 20 */
// pins affected by this interrupt request:["USER_SWITCH_1"]
#define GPIO_SWITCHES_INT_IRQN                                  (GPIOB_INT_IRQn)
#define GPIO_SWITCHES_INT_IIDX                  (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_SWITCHES_USER_SWITCH_1_IIDX                    (DL_GPIO_IIDX_DIO21)
#define GPIO_SWITCHES_USER_SWITCH_1_PIN                         (DL_GPIO_PIN_21)
#define GPIO_SWITCHES_USER_SWITCH_1_IOMUX                        (IOMUX_PINCM49)

/* Port definition for Pin Group GPIO_LEDS */
#define GPIO_LEDS_PORT                                                   (GPIOB)

/* Defines for USER_LED_1: GPIOB.22 with pinCMx 50 on package pin 21 */
#define GPIO_LEDS_USER_LED_1_PIN                                (DL_GPIO_PIN_22)
#define GPIO_LEDS_USER_LED_1_IOMUX                               (IOMUX_PINCM50)
/* Defines for USER_LED_2: GPIOB.26 with pinCMx 57 on package pin 28 */
#define GPIO_LEDS_USER_LED_2_PIN                                (DL_GPIO_PIN_26)
#define GPIO_LEDS_USER_LED_2_IOMUX                               (IOMUX_PINCM57)
/* Defines for PIN_2: GPIOB.13 with pinCMx 30 on package pin 1 */
#define GPIO_LEDS_PIN_2_PIN                                     (DL_GPIO_PIN_13)
#define GPIO_LEDS_PIN_2_IOMUX                                    (IOMUX_PINCM30)


/* Defines for MCAN0 */
#define MCAN0_INST                                                        CANFD0
#define GPIO_MCAN0_CAN_TX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_TX_PIN                                     DL_GPIO_PIN_12
#define GPIO_MCAN0_IOMUX_CAN_TX                                  (IOMUX_PINCM34)
#define GPIO_MCAN0_IOMUX_CAN_TX_FUNC               IOMUX_PINCM34_PF_CANFD0_CANTX
#define GPIO_MCAN0_CAN_RX_PORT                                             GPIOA
#define GPIO_MCAN0_CAN_RX_PIN                                     DL_GPIO_PIN_13
#define GPIO_MCAN0_IOMUX_CAN_RX                                  (IOMUX_PINCM35)
#define GPIO_MCAN0_IOMUX_CAN_RX_FUNC               IOMUX_PINCM35_PF_CANFD0_CANRX
#define MCAN0_INST_IRQHandler                                 CANFD0_IRQHandler
#define MCAN0_INST_INT_IRQN                                     CANFD0_INT_IRQn


/* Defines for MCAN0 MCAN RAM configuration */
#define MCAN0_INST_MCAN_STD_ID_FILT_START_ADDR     (0)
#define MCAN0_INST_MCAN_STD_ID_FILTER_NUM          (2)
#define MCAN0_INST_MCAN_EXT_ID_FILT_START_ADDR     (0)
#define MCAN0_INST_MCAN_EXT_ID_FILTER_NUM          (0)
#define MCAN0_INST_MCAN_TX_BUFF_START_ADDR         (148)
#define MCAN0_INST_MCAN_TX_BUFF_SIZE               (2)
#define MCAN0_INST_MCAN_FIFO_1_START_ADDR          (192)
#define MCAN0_INST_MCAN_FIFO_1_NUM                 (2)
#define MCAN0_INST_MCAN_TX_EVENT_START_ADDR        (164)
#define MCAN0_INST_MCAN_TX_EVENT_SIZE              (2)
#define MCAN0_INST_MCAN_EXT_ID_AND_MASK            (0x1FFFFFFFU)
#define MCAN0_INST_MCAN_RX_BUFF_START_ADDR         (208)
#define MCAN0_INST_MCAN_FIFO_0_START_ADDR          (172)
#define MCAN0_INST_MCAN_FIFO_0_NUM                 (3)

#define MCAN0_INST_MCAN_INTERRUPTS (DL_MCAN_INTERRUPT_ARA | \
                        DL_MCAN_INTERRUPT_BEU | \
                        DL_MCAN_INTERRUPT_BO | \
                        DL_MCAN_INTERRUPT_DRX | \
                        DL_MCAN_INTERRUPT_ELO | \
                        DL_MCAN_INTERRUPT_EP | \
                        DL_MCAN_INTERRUPT_EW | \
                        DL_MCAN_INTERRUPT_MRAF | \
                        DL_MCAN_INTERRUPT_PEA | \
                        DL_MCAN_INTERRUPT_PED | \
                        DL_MCAN_INTERRUPT_RF0N | \
                        DL_MCAN_INTERRUPT_TC | \
                        DL_MCAN_INTERRUPT_TEFN | \
                        DL_MCAN_INTERRUPT_TOO | \
                        DL_MCAN_INTERRUPT_TSW | \
                        DL_MCAN_INTERRUPT_WDI)



void CAN_config(void);
void CAN_initMsgFrame(DL_MCAN_TxBufElement* txMsg,uint32_t id,uint32_t size,uint32_t* data);
void CAN_transmitMsg(DL_MCAN_TxBufElement* txMsg);
void CAN_receiveMsg();
void CAN_flushRxBuf();
void CANFD0_IRQHandler(void);


static const DL_MCAN_ClockConfig gMCAN0ClockConf = {
    .clockSel = DL_MCAN_FCLK_HFCLK,
    .divider  = DL_MCAN_FCLK_DIV_1,
};

static const DL_MCAN_InitParams gMCAN0InitParams= {

/* Initialize MCAN Init parameters.    */
    .fdMode            = true,
    .brsEnable         = true,
    .txpEnable         = false,
    .efbi              = false,
    .pxhddisable       = false,
    .darEnable         = false,
    .wkupReqEnable     = true,
    .autoWkupEnable    = true,
    .emulationEnable   = true,
    .tdcEnable         = true,
    .wdcPreload        = 255,

/* Transmitter Delay Compensation parameters. */
    .tdcConfig.tdcf    = 10,
    .tdcConfig.tdco    = 6,
};

static const DL_MCAN_ConfigParams gMCAN0ConfigParams={
    /* Initialize MCAN Config parameters. */
    .monEnable         = false,
    .asmEnable         = false,
    .tsPrescalar       = 15,
    .tsSelect          = 0,
    .timeoutSelect     = DL_MCAN_TIMEOUT_SELECT_CONT,
    .timeoutPreload    = 65535,
    .timeoutCntEnable  = false,
    .filterConfig.rrfs = true,
    .filterConfig.rrfe = true,
    .filterConfig.anfe = 1,
    .filterConfig.anfs = 1,
};

static const DL_MCAN_MsgRAMConfigParams gMCAN0MsgRAMConfigParams ={

    /* Standard ID Filter List Start Address. */
    .flssa                = MCAN0_INST_MCAN_STD_ID_FILT_START_ADDR,
    /* List Size: Standard ID. */
    .lss                  = MCAN0_INST_MCAN_STD_ID_FILTER_NUM,
    /* Extended ID Filter List Start Address. */
    .flesa                = MCAN0_INST_MCAN_EXT_ID_FILT_START_ADDR,
    /* List Size: Extended ID. */
    .lse                  = MCAN0_INST_MCAN_EXT_ID_FILTER_NUM,
    /* Tx Buffers Start Address. */
    .txStartAddr          = MCAN0_INST_MCAN_TX_BUFF_START_ADDR,
    /* Number of Dedicated Transmit Buffers. */
    .txBufNum             = MCAN0_INST_MCAN_TX_BUFF_SIZE,
    .txFIFOSize           = 0,
    /* Tx Buffer Element Size. */
    .txBufMode            = 0,
    .txBufElemSize        = DL_MCAN_ELEM_SIZE_64BYTES,
    /* Tx Event FIFO Start Address. */
    .txEventFIFOStartAddr = MCAN0_INST_MCAN_TX_EVENT_START_ADDR,
    /* Event FIFO Size. */
    .txEventFIFOSize      = MCAN0_INST_MCAN_TX_EVENT_SIZE,
    /* Level for Tx Event FIFO watermark interrupt. */
    .txEventFIFOWaterMark = 0,
    /* Rx FIFO0 Start Address. */
    .rxFIFO0startAddr     = MCAN0_INST_MCAN_FIFO_0_START_ADDR,
    /* Number of Rx FIFO elements. */
    .rxFIFO0size          = MCAN0_INST_MCAN_FIFO_0_NUM,
    /* Rx FIFO0 Watermark. */
    .rxFIFO0waterMark     = 0,
    .rxFIFO0OpMode        = 0,
    /* Rx FIFO1 Start Address. */
    .rxFIFO1startAddr     = MCAN0_INST_MCAN_FIFO_1_START_ADDR,
    /* Number of Rx FIFO elements. */
    .rxFIFO1size          = MCAN0_INST_MCAN_FIFO_1_NUM,
    /* Level for Rx FIFO 1 watermark interrupt. */
    .rxFIFO1waterMark     = 3,
    /* FIFO blocking mode. */
    .rxFIFO1OpMode        = 0,
    /* Rx Buffer Start Address. */
    .rxBufStartAddr       = MCAN0_INST_MCAN_RX_BUFF_START_ADDR,
    /* Rx Buffer Element Size. */
    .rxBufElemSize        = DL_MCAN_ELEM_SIZE_8BYTES,
    /* Rx FIFO0 Element Size. */
    .rxFIFO0ElemSize      = DL_MCAN_ELEM_SIZE_8BYTES,
    /* Rx FIFO1 Element Size. */
    .rxFIFO1ElemSize      = DL_MCAN_ELEM_SIZE_8BYTES,
};

static const DL_MCAN_StdMsgIDFilterElement gMCAN0StdFiltelem = {
    .sfec = 0x1,
    .sft  = 0x1,
    .sfid1 = 3,
    .sfid2 = 4,
};


static const DL_MCAN_BitTimingParams   gMCAN0BitTimes = {
    /* Arbitration Baud Rate Pre-scaler. */
    .nomRatePrescalar   = 3,
    /* Arbitration Time segment before sample point. */
    .nomTimeSeg1        = 30,
    /* Arbitration Time segment after sample point. */
    .nomTimeSeg2        = 7,
    /* Arbitration (Re)Synchronization Jump Width Range. */
    .nomSynchJumpWidth  = 7,
    /* Data Baud Rate Pre-scaler. */
    .dataRatePrescalar  = 3,
    /* Data Time segment before sample point. */
    .dataTimeSeg1       = 14,
    /* Data Time segment after sample point. */
    .dataTimeSeg2       = 3,
    /* Data (Re)Synchronization Jump Width.   */
    .dataSynchJumpWidth = 3,
};


void CAN_config(){
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_MCAN_reset(MCAN0_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_MCAN_enablePower(MCAN0_INST);
    delay_cycles(POWER_STARTUP_DELAY);


    ////

//Tx
    DL_GPIO_initDigitalInputFeatures(GPIO_SWITCHES_USER_SWITCH_1_IOMUX,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
         DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setUpperPinsPolarity(GPIO_SWITCHES_PORT, DL_GPIO_PIN_21_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(GPIO_SWITCHES_PORT, GPIO_SWITCHES_USER_SWITCH_1_PIN);
    DL_GPIO_enableInterrupt(GPIO_SWITCHES_PORT, GPIO_SWITCHES_USER_SWITCH_1_PIN);


//Rx

    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXIN_IOMUX);
    DL_GPIO_initPeripheralAnalogFunction(GPIO_HFXOUT_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_LEDS_USER_LED_1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_LEDS_USER_LED_2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_LEDS_PIN_2_IOMUX);

    DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN |
        GPIO_LEDS_USER_LED_2_PIN |
        GPIO_LEDS_PIN_2_PIN);
    DL_GPIO_enableOutput(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN |
        GPIO_LEDS_USER_LED_2_PIN |
        GPIO_LEDS_PIN_2_PIN);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_MCAN0_IOMUX_CAN_TX, GPIO_MCAN0_IOMUX_CAN_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_MCAN0_IOMUX_CAN_RX, GPIO_MCAN0_IOMUX_CAN_RX_FUNC);



    ///
    //Low Power Mode is configured to be SLEEP0
     DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);


     DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
     /* Set default configuration */
     DL_SYSCTL_disableHFXT();
     DL_SYSCTL_disableSYSPLL();
     DL_SYSCTL_setHFCLKSourceHFXTParams(DL_SYSCTL_HFXT_RANGE_32_48_MHZ,10, true);

    /////
     DL_MCAN_RevisionId revid_MCAN0;

     DL_MCAN_enableModuleClock(MCAN0_INST);

     DL_MCAN_setClockConfig(MCAN0_INST, (DL_MCAN_ClockConfig *) &gMCAN0ClockConf);

     /* Get MCANSS Revision ID. */
     DL_MCAN_getRevisionId(MCAN0_INST, &revid_MCAN0);

     /* Wait for Memory initialization to be completed. */
     while(false == DL_MCAN_isMemInitDone(MCAN0_INST));

     /* Put MCAN in SW initialization mode. */

     DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_SW_INIT);

     /* Wait till MCAN is not initialized. */
     while (DL_MCAN_OPERATION_MODE_SW_INIT != DL_MCAN_getOpMode(MCAN0_INST));

     /* Initialize MCAN module. */
     DL_MCAN_init(MCAN0_INST, (DL_MCAN_InitParams *) &gMCAN0InitParams);

     /* Configure MCAN module. */
     DL_MCAN_config(MCAN0_INST, (DL_MCAN_ConfigParams*) &gMCAN0ConfigParams);

     /* Configure Bit timings. */
     DL_MCAN_setBitTime(MCAN0_INST, (DL_MCAN_BitTimingParams*) &gMCAN0BitTimes);

     /* Configure Message RAM Sections */
     DL_MCAN_msgRAMConfig(MCAN0_INST, (DL_MCAN_MsgRAMConfigParams*) &gMCAN0MsgRAMConfigParams);

     /* Configure Standard ID filter element */
     DL_MCAN_addStdMsgIDFilter(MCAN0_INST, 0U, (DL_MCAN_StdMsgIDFilterElement *) &gMCAN0StdFiltelem);


     /* Set Extended ID Mask. */
     DL_MCAN_setExtIDAndMask(MCAN0_INST, MCAN0_INST_MCAN_EXT_ID_AND_MASK );

     /* Loopback mode */

     /* Take MCAN out of the SW initialization mode */
     DL_MCAN_setOpMode(MCAN0_INST, DL_MCAN_OPERATION_MODE_NORMAL);

     while (DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(MCAN0_INST));

     /* Enable MCAN mopdule Interrupts */
     DL_MCAN_enableIntr(MCAN0_INST, MCAN0_INST_MCAN_INTERRUPTS, 1U);

     DL_MCAN_selectIntrLine(MCAN0_INST, DL_MCAN_INTR_MASK_ALL, DL_MCAN_INTR_LINE_NUM_1);
     DL_MCAN_enableIntrLine(MCAN0_INST, DL_MCAN_INTR_LINE_NUM_1, 1U);

     /* Enable MSPM0 MCAN interrupt */
     DL_MCAN_clearInterruptStatus(MCAN0_INST,(DL_MCAN_MSP_INTERRUPT_LINE1));
     DL_MCAN_enableInterrupt(MCAN0_INST,(DL_MCAN_MSP_INTERRUPT_LINE1));
    //
   //////

     while ((DL_SYSCTL_getClockStatus() & (DL_SYSCTL_CLK_STATUS_HFCLK_GOOD
          | DL_SYSCTL_CLK_STATUS_LFOSC_GOOD))
            != (DL_SYSCTL_CLK_STATUS_HFCLK_GOOD
          | DL_SYSCTL_CLK_STATUS_LFOSC_GOOD))
     {
         /* Ensure that clocks are in default POR configuration before initialization.
         * Additionally once LFXT is enabled, the internal LFOSC is disabled, and cannot
         * be re-enabled other than by executing a BOOTRST. */
         ;
     }

         NVIC_EnableIRQ(CANFD0_INT_IRQn);
         while (DL_MCAN_OPERATION_MODE_NORMAL != DL_MCAN_getOpMode(MCAN0_INST))
             ;



}


void CAN_initMsgFrame(DL_MCAN_TxBufElement* txMsg,uint32_t id,uint32_t size,uint32_t* data){
    /* Initialize message to transmit. */
        /* Identifier Value. */
        txMsg->id = ((uint32_t)(id)) << 18U;
        /* Transmit data frame. */
        txMsg->rtr = 0U;
        /* 11-bit standard identifier. */
        txMsg->xtd = 0U;
        /* ESI bit in CAN FD format depends only on error passive flag. */
        txMsg->esi = 0U;
        /* Transmitting 4 bytes. */
        txMsg->dlc = size;
        /* CAN FD frames transmitted with bit rate switching. */
        txMsg->brs = 1U;
        /* Frame transmitted in CAN FD format. */
        txMsg->fdf = 0U;
        /* Store Tx events. */
        txMsg->efc = 1U;
        /* Message Marker. */
        txMsg->mm = 0xAAU;
        /* Data bytes. */
        for(int i=0;i<size;i++){
            txMsg->data[i] = data[i];
        }


}


void CAN_transmitMsg(DL_MCAN_TxBufElement* txMsg){
    /* Write Tx Message to the Message RAM. */
    DL_MCAN_writeMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_BUF, 0U, txMsg);

    /* Add request for transmission. */
    DL_MCAN_TXBufAddReq(MCAN0_INST, 0U);
}

void CAN_receiveMsg(){
    gServiceInt = false;  // Reset the service interrupt flag
    rxFS.fillLvl = 0;     // Initialize fill level for RX FIFO

    // Check if a message has been received on FIFO 0
    if ((gInterruptLine1Status & MCAN_IR_RF0N_MASK) == MCAN_IR_RF0N_MASK) {
        rxFS.num = DL_MCAN_RX_FIFO_NUM_0;

        // Wait for at least one message in the RX FIFO
        while (rxFS.fillLvl == 0) {
            DL_MCAN_getRxFIFOStatus(MCAN0_INST, &rxFS);
        }

        // Read the message from the RX FIFO
        DL_MCAN_readMsgRam(MCAN0_INST, DL_MCAN_MEM_TYPE_FIFO, 0U, rxFS.num, &rxMsg);
        DL_MCAN_writeRxFIFOAck(MCAN0_INST, rxFS.num, rxFS.getIdx);  // Acknowledge the message

        uint32_t idMode = rxMsg.xtd;  // Extract ID mode
        uint32_t id;                   // Variable to store the ID

        // Determine the ID based on the mode (extended or standard)
        if (ID_MODE_EXTENDED == idMode) {
            id = rxMsg.id;  // Use extended ID
        } else {
            // Assuming package is using 11-bit standard ID
            // When package uses standard ID, ID is stored in ID[28:18]
            id = ((rxMsg.id & (uint32_t)0x1FFC0000) >> (uint32_t)18);
        }

        // Loop through the data length and store data in RxData buffer
        for (int i = 0; i < rxMsg.dlc; i++) {
            if (bufferIndex < BUFFER_SIZE) {  // Check to prevent buffer overflow
                RxData[bufferIndex++] = rxMsg.data[i];
            } else {
                // Handle buffer overflow, e.g., log an error or reset bufferIndex
            }
        }

        gInterruptLine1Status &= ~(MCAN_IR_RF0N_MASK);  // Clear the received interrupt status
    }
}

void CAN_flushRxBuf(){
    bufferIndex = 0;  // Reset the buffer index
}

void CANFD0_IRQHandler(void){
    printf("Interrupt!\n");  // Debug print to indicate an interrupt has occurred
    switch (DL_MCAN_getPendingInterrupt(MCAN0_INST)) {
        case DL_MCAN_IIDX_LINE1:
            CAN_receiveMsg();  // Call the receive message function

            /* Check MCAN interrupts fired during TX/RX of CAN package */
            gInterruptLine1Status |= DL_MCAN_getIntrStatus(MCAN0_INST);  // Update interrupt status

            // Clear the interrupt status for line 1
            DL_MCAN_clearIntrStatus(MCAN0_INST, gInterruptLine1Status, DL_MCAN_INTR_SRC_MCAN_LINE_1);

            gServiceInt = true;  // Set the service interrupt flag
            break;
        default:
            break;  // Handle other cases if needed
    }
}


