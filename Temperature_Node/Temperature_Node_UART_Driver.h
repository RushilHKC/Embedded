#include "ti_msp_dl_config.h"

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

bool gUartServiceInt = false;

/* Defines for UART_0 */
#define UART_0_INST                UART0
#define UART_0_INST_IRQHandler      UART0_IRQHandler
#define UART_0_INST_INT_IRQN        UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT         GPIOA
#define GPIO_UART_0_TX_PORT         GPIOA
#define GPIO_UART_0_RX_PIN          DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN          DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX        IOMUX_PINCM22
#define GPIO_UART_0_IOMUX_TX        IOMUX_PINCM21
#define GPIO_UART_0_IOMUX_RX_FUNC   IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC   IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE            (9600)
#define UART_0_IBRD_32_kHZ_9600_BAUD (1)
#define UART_0_FBRD_32_kHZ_9600_BAUD (9)

typedef struct UART_INST {
    UART_Regs *uartModule;
    GPIO_Regs *gpioRxPort;
    GPIO_Regs *gpioTxPort;
    uint32_t gpioRxPin;
    uint32_t gpioTxPin;
    IOMUX_PINCM rxIoMux;
    IOMUX_PINCM txIoMux;
    uint32_t rxIoMuxFunc;
    uint32_t txIoMuxFunc;
} UART_INST;

UART_INST uart_instance = { 0 };

// Initialize UART module, configure its GPIOs, and enable power
void UART_INIT(UART_Regs *uartModule, GPIO_Regs *gpioRxPort,
               GPIO_Regs *gpioTxPort, uint32_t gpioRxPin, uint32_t gpioTxPin) {
    SYSCFG_DL_initPower();
    DL_UART_Main_reset(uartModule);
    DL_UART_Main_enablePower(uartModule);
    SYSCFG_DL_SYSCTL_init();

    IOMUX_PINCM rxIoMux;
    IOMUX_PINCM txIoMux;
    uint32_t rxIoMuxFunc;
    uint32_t txIoMuxFunc;

    if (uartModule == UART0 && gpioRxPort == GPIOA && gpioTxPort == GPIOA && gpioRxPin == DL_GPIO_PIN_1 && gpioTxPin == DL_GPIO_PIN_0) {
        rxIoMux = IOMUX_PINCM2;
        txIoMux = IOMUX_PINCM1;
        rxIoMuxFunc = IOMUX_PINCM2_PF_UART0_RX;
        txIoMuxFunc = IOMUX_PINCM1_PF_UART0_TX;
    } else if (uartModule == UART0 && gpioRxPort == GPIOA && gpioTxPort == GPIOA && gpioRxPin == DL_GPIO_PIN_11 && gpioTxPin == DL_GPIO_PIN_10) {
        rxIoMux = IOMUX_PINCM22;
        txIoMux = IOMUX_PINCM21;
        rxIoMuxFunc = IOMUX_PINCM22_PF_UART0_RX;
        txIoMuxFunc = IOMUX_PINCM21_PF_UART0_TX;
    } else if (uartModule == UART0 && gpioRxPort == GPIOA && gpioTxPort == GPIOA && gpioRxPin == DL_GPIO_PIN_31 && gpioTxPin == DL_GPIO_PIN_28) {
        rxIoMux = IOMUX_PINCM6;
        txIoMux = IOMUX_PINCM3;
        rxIoMuxFunc = IOMUX_PINCM6_PF_UART0_RX;
        txIoMuxFunc = IOMUX_PINCM3_PF_UART0_TX;
    } else if (uartModule == UART0 && gpioRxPort == GPIOB && gpioTxPort == GPIOB && gpioRxPin == DL_GPIO_PIN_1 && gpioTxPin == DL_GPIO_PIN_0) {
        rxIoMux = IOMUX_PINCM13;
        txIoMux = IOMUX_PINCM12;
        rxIoMuxFunc = IOMUX_PINCM13_PF_UART0_RX;
        txIoMuxFunc = IOMUX_PINCM12_PF_UART0_TX;
    }

    uart_instance.uartModule = uartModule;
    uart_instance.gpioRxPort = gpioRxPort;
    uart_instance.gpioTxPort = gpioTxPort;
    uart_instance.gpioRxPin = gpioRxPin;
    uart_instance.gpioTxPin = gpioTxPin;
    uart_instance.rxIoMux = rxIoMux;
    uart_instance.txIoMux = txIoMux;
    uart_instance.rxIoMuxFunc = rxIoMuxFunc;
    uart_instance.txIoMuxFunc = txIoMuxFunc;


    DL_GPIO_initPeripheralOutputFunction(uart_instance.txIoMux, uart_instance.txIoMuxFunc);
    DL_GPIO_initPeripheralInputFunction(uart_instance.rxIoMux, uart_instance.rxIoMuxFunc);

    // Enable UART interrupts
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
}

// Configure the UART communication parameters like clock, parity, word length, and stop bits
void UART_COMMUNICATION_CONFIG(DL_UART_CLOCK clockSel, DL_UART_CLOCK_DIVIDE_RATIO divideRatio,
                               DL_UART_PARITY parity, DL_UART_WORD_LENGTH wordLength,
                               DL_UART_STOP_BITS stopBits) {
    DL_UART_Main_ClockConfig clockConfig = { .clockSel = clockSel, .divideRatio = divideRatio };
    DL_UART_Main_Config mainConfig = { .mode = DL_UART_MAIN_MODE_NORMAL, .direction = DL_UART_MAIN_DIRECTION_TX_RX,
                                       .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE, .parity = parity,
                                       .wordLength = wordLength, .stopBits = stopBits };


    DL_UART_Main_setClockConfig(uart_instance.uartModule, &clockConfig);


    DL_UART_Main_init(uart_instance.uartModule, &mainConfig);


    DL_UART_Main_setOversampling(uart_instance.uartModule, DL_UART_OVERSAMPLING_RATE_3X);
    DL_UART_Main_setBaudRateDivisor(uart_instance.uartModule, UART_0_IBRD_32_kHZ_9600_BAUD, UART_0_FBRD_32_kHZ_9600_BAUD);


    DL_UART_Main_enableInterrupt(uart_instance.uartModule, DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                                     DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                                     DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                                     DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                                     DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                                     DL_UART_MAIN_INTERRUPT_RX | DL_UART_INTERRUPT_EOT_DONE);


    DL_UART_Main_enableFIFOs(uart_instance.uartModule);
    DL_UART_Main_setRXFIFOThreshold(uart_instance.uartModule, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(uart_instance.uartModule, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);


    DL_UART_Main_enable(uart_instance.uartModule);
}
#define RxBufferSize 256
uint16_t grecIndex = 0;
uint32_t gRXBuff[RxBufferSize];
uint32_t greceiveData;


// Function to receive a byte of data from UART
uint8_t UART_receiveData(UART_Regs *uart) {
    return ((uint8_t)(uart->RXDATA & UART_RXDATA_DATA_MASK));
}

// Function to transmit a byte of data via UART
void UART_transmitData(UART_Regs *uart, uint8_t data) {
    uart->TXDATA = data;
}

// Function to clear the UART buffer
void UART_flushBuffer() {
    grecIndex = 0;
}

// UART interrupt handler to handle received data
void UART_0_INST_IRQHandler(void) {
    switch (DL_UART_getPendingInterrupt(UART_0_INST)) {
    case DL_UART_IIDX_RX:
        greceiveData = UART_receiveData(UART_0_INST);
        gRXBuff[grecIndex++] = greceiveData;
        gUartServiceInt = true;
        break;
    default:
        break;
    }
}
