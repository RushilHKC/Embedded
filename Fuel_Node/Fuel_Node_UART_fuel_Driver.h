#include "ti_msp_dl_config.h"
#include <stdio.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

/* RX buffer definitions */
#define RX_BUFFER_SIZE 256
volatile uint16_t g_rec_index = 0;  // Global receive buffer index
volatile uint32_t g_rx_buffer[RX_BUFFER_SIZE];  // Global buffer for received data
volatile uint32_t g_receive_data;  // Global variable to store received byte


/* Defines for UART_0 */
#define UART_0_INSTANCE                                                       UART0
#define UART_0_INSTANCE_IRQHandler                              UART0_IRQHandler
#define UART_0_INSTANCE_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                  (9600)
#define UART_0_IBRD_32_kHZ_9600_BAUD                                         (1)
#define UART_0_FBRD_32_kHZ_9600_BAUD                                         (9)

/* Structure for UART instance */
typedef struct uart_instance_t {
    UART_Regs *uart_module;
    GPIO_Regs *gpio_rx_port;
    GPIO_Regs *gpio_tx_port;
    uint32_t gpio_rx_pin;
    uint32_t gpio_tx_pin;
    IOMUX_PINCM rx_iomux;
    IOMUX_PINCM tx_iomux;
    uint32_t rx_iomux_func;
    uint32_t tx_iomux_func;
} uart_instance_t;

/* Global UART instance */
uart_instance_t g_uart_instance = { 0 };

/* UART initialization function */
void UART_Init(UART_Regs *uart_module, GPIO_Regs *gpio_rx_port,
               GPIO_Regs *gpio_tx_port, uint32_t gpio_rx_pin, uint32_t gpio_tx_pin) {
    SYSCFG_DL_initPower();
    DL_UART_Main_reset(uart_module);
    DL_UART_Main_enablePower(uart_module);
    SYSCFG_DL_SYSCTL_init();

    IOMUX_PINCM rx_iomux;
    IOMUX_PINCM tx_iomux;
    uint32_t rx_iomux_func;
    uint32_t tx_iomux_func;

    // Configures UART0 with RX/TX on GPIOA pins
    if (uart_module == UART0 && gpio_rx_port == GPIOA && gpio_tx_port == GPIOA &&
        gpio_rx_pin == DL_GPIO_PIN_11 && gpio_tx_pin == DL_GPIO_PIN_10) {
        rx_iomux = IOMUX_PINCM22;
        tx_iomux = IOMUX_PINCM21;
        rx_iomux_func = IOMUX_PINCM22_PF_UART0_RX;
        tx_iomux_func = IOMUX_PINCM21_PF_UART0_TX;
    }

    g_uart_instance.uart_module = uart_module;
    g_uart_instance.gpio_rx_port = gpio_rx_port;
    g_uart_instance.gpio_tx_port = gpio_tx_port;
    g_uart_instance.gpio_rx_pin = gpio_rx_pin;
    g_uart_instance.gpio_tx_pin = gpio_tx_pin;
    g_uart_instance.rx_iomux = rx_iomux;
    g_uart_instance.tx_iomux = tx_iomux;
    g_uart_instance.rx_iomux_func = rx_iomux_func;
    g_uart_instance.tx_iomux_func = tx_iomux_func;

    DL_GPIO_initPeripheralOutputFunction(g_uart_instance.tx_iomux, g_uart_instance.tx_iomux_func);
    DL_GPIO_initPeripheralInputFunction(g_uart_instance.rx_iomux, g_uart_instance.rx_iomux_func);
    NVIC_EnableIRQ(UART_0_INSTANCE_INT_IRQN);
}

/* UART communication configuration */
void UART_Config(DL_UART_CLOCK clock_sel,
                 DL_UART_CLOCK_DIVIDE_RATIO divide_ratio,
                 DL_UART_PARITY parity,
                 DL_UART_WORD_LENGTH word_length,
                 DL_UART_STOP_BITS stop_bits) {
    DL_UART_Main_ClockConfig clock_config = { .clockSel = clock_sel, .divideRatio = divide_ratio };
    DL_UART_Main_Config main_config = {
        .mode = DL_UART_MAIN_MODE_NORMAL,
        .direction = DL_UART_MAIN_DIRECTION_TX_RX,
        .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
        .parity = parity,
        .wordLength = word_length,
        .stopBits = stop_bits
    };

    DL_UART_Main_setClockConfig(g_uart_instance.uart_module, &clock_config);
    DL_UART_Main_init(g_uart_instance.uart_module, &main_config);
    DL_UART_Main_setOversampling(g_uart_instance.uart_module, DL_UART_OVERSAMPLING_RATE_3X);
    DL_UART_Main_setBaudRateDivisor(g_uart_instance.uart_module, UART_0_IBRD_32_kHZ_9600_BAUD,
                                    UART_0_FBRD_32_kHZ_9600_BAUD);

    DL_UART_Main_enableInterrupt(g_uart_instance.uart_module,
                                 DL_UART_MAIN_INTERRUPT_BREAK_ERROR |
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_NOISE_ERROR |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_PARITY_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX |
                                 DL_UART_INTERRUPT_EOT_DONE);

    DL_UART_Main_enableFIFOs(g_uart_instance.uart_module);
    DL_UART_Main_setRXFIFOThreshold(g_uart_instance.uart_module, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(g_uart_instance.uart_module, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);

    DL_UART_Main_enable(g_uart_instance.uart_module);
}


/* Function to receive data via UART */
uint8_t UART_ReceiveData(UART_Regs *uart_module) {
    return ((uint8_t)(uart_module->RXDATA & UART_RXDATA_DATA_MASK));
}

/* Function to transmit data via UART */
void UART_TransmitData(UART_Regs *uart_module, uint8_t data) {
    uart_module->TXDATA = data;
}

/* Function to print received data */
void PrintReceivedData(uint8_t receive_data) {
    g_rx_buffer[g_rec_index++] = receive_data;

    // Print the received data stored in buffer
    for (uint8_t i = 0; i < g_rec_index; i++) {
        printf("%c\n ", g_rx_buffer[i]);
    }
    g_rec_index = 0;  // Reset buffer index after printing
}

/* UART interrupt handler to receive data */
void UART_0_INSTANCE_IRQHandler(void) {
    switch (DL_UART_getPendingInterrupt(UART_0_INSTANCE)) {
    case DL_UART_IIDX_RX:
        g_receive_data = UART_ReceiveData(UART_0_INSTANCE);  // Store received byte
        printf("Fuel Value:\n");
        g_rx_buffer[g_rec_index++] = g_receive_data;  // Add received byte to buffer
        break;
    default:
        break;
    }
}
