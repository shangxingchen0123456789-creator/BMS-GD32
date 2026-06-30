#include "bms_uart.h"

#include "bms_board_config.h"

#define BMS_UART_TX_BUFFER_SIZE                 512U
#define BMS_UART_RX_BUFFER_SIZE                 256U

/*
 * UART 封装。
 *
 * RX/TX 都采用环形缓冲 + 中断方式搬运字节，通信任务只做非阻塞读取和写入。
 * 这样上位机在很短时间内发完一帧命令时，字节会先被 USART 中断保存下来，
 * 不会因为通信任务 10ms 调度周期太慢而覆盖丢失。
 */
static uint8_t s_rx_buffer[BMS_UART_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static volatile uint32_t s_rx_dropped;

static uint8_t s_tx_buffer[BMS_UART_TX_BUFFER_SIZE];
static volatile uint16_t s_tx_head;
static volatile uint16_t s_tx_tail;
static volatile uint32_t s_tx_dropped;

static uint16_t Uart_Next_Index(uint16_t index, uint16_t size)
{
    index++;
    if(index >= size) {
        index = 0U;
    }

    return index;
}

static uint16_t Uart_Rx_Next_Index(uint16_t index)
{
    return Uart_Next_Index(index, BMS_UART_RX_BUFFER_SIZE);
}

static uint16_t Uart_Tx_Next_Index(uint16_t index)
{
    return Uart_Next_Index(index, BMS_UART_TX_BUFFER_SIZE);
}

static uint16_t Uart_Tx_Used_Unlocked(void)
{
    if(s_tx_head >= s_tx_tail) {
        return (uint16_t)(s_tx_head - s_tx_tail);
    }

    return (uint16_t)(BMS_UART_TX_BUFFER_SIZE - s_tx_tail + s_tx_head);
}

static uint16_t Uart_Tx_Free_Unlocked(void)
{
    return (uint16_t)(BMS_UART_TX_BUFFER_SIZE - 1U - Uart_Tx_Used_Unlocked());
}

static void Uart_Clear_Rx_Errors(void)
{
    /*
     * 上位机插拔或波特率误配时，USART 可能留下错误标志。
     * 接收中断中读出数据后清掉这些标志，避免后续真实数据被硬件错误状态挡住。
     */
    if(RESET != usart_flag_get(BMS_UART_PERIPH, USART_FLAG_PERR)) {
        usart_flag_clear(BMS_UART_PERIPH, USART_FLAG_PERR);
    }
    if(RESET != usart_flag_get(BMS_UART_PERIPH, USART_FLAG_FERR)) {
        usart_flag_clear(BMS_UART_PERIPH, USART_FLAG_FERR);
    }
    if(RESET != usart_flag_get(BMS_UART_PERIPH, USART_FLAG_NERR)) {
        usart_flag_clear(BMS_UART_PERIPH, USART_FLAG_NERR);
    }
    if(RESET != usart_flag_get(BMS_UART_PERIPH, USART_FLAG_ORERR)) {
        usart_flag_clear(BMS_UART_PERIPH, USART_FLAG_ORERR);
    }
}

static void Uart_Tx_Kick(void)
{
    if(s_tx_head != s_tx_tail) {
        usart_interrupt_enable(BMS_UART_PERIPH, USART_INT_TBE);
    }
}

void Bms_Uart_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_rx_dropped = 0U;

    s_tx_head = 0U;
    s_tx_tail = 0U;
    s_tx_dropped = 0U;

    rcu_periph_clock_enable(BMS_UART_GPIO_CLK);
    rcu_periph_clock_enable(BMS_UART_CLK);
    rcu_usart_clock_config(IDX_USART2, RCU_USARTSRC_APB);

    gpio_af_set(BMS_UART_GPIO_PORT, BMS_UART_AF, BMS_UART_TX_PIN);
    gpio_af_set(BMS_UART_GPIO_PORT, BMS_UART_AF, BMS_UART_RX_PIN);

    gpio_mode_set(BMS_UART_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BMS_UART_TX_PIN);
    gpio_output_options_set(BMS_UART_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_12MHZ, BMS_UART_TX_PIN);

    gpio_mode_set(BMS_UART_GPIO_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BMS_UART_RX_PIN);
    gpio_output_options_set(BMS_UART_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_12MHZ, BMS_UART_RX_PIN);

    usart_deinit(BMS_UART_PERIPH);
    USART_CTL0(BMS_UART_PERIPH) = 0U;
    USART_CTL1(BMS_UART_PERIPH) = 0U;
    USART_CTL2(BMS_UART_PERIPH) = 0U;
    usart_baudrate_set(BMS_UART_PERIPH, BMS_UART_BAUDRATE);
    usart_receive_config(BMS_UART_PERIPH, USART_RECEIVE_ENABLE);
    usart_transmit_config(BMS_UART_PERIPH, USART_TRANSMIT_ENABLE);
    usart_enable(BMS_UART_PERIPH);
    Uart_Clear_Rx_Errors();

    usart_interrupt_enable(BMS_UART_PERIPH, USART_INT_RBNE);
    nvic_irq_enable(BMS_UART_IRQn, 1U, 0U);
}

uint16_t Bms_Uart_Read(uint8_t *data, uint16_t max_length)
{
    uint16_t count;
    uint32_t primask;

    if(data == 0 || max_length == 0U) {
        return 0U;
    }

    /* 非阻塞读取：只搬走软件 RX 环形缓冲中已经收到的字节。 */
    count = 0U;
    while(count < max_length) {
        primask = __get_PRIMASK();
        __disable_irq();

        if(s_rx_head == s_rx_tail) {
            if(primask == 0U) {
                __enable_irq();
            }
            break;
        }

        data[count] = s_rx_buffer[s_rx_tail];
        s_rx_tail = Uart_Rx_Next_Index(s_rx_tail);
        if(primask == 0U) {
            __enable_irq();
        }

        count++;
    }

    return count;
}

void Bms_Uart_Send(const uint8_t *data, uint16_t length)
{
    uint16_t i;
    uint32_t primask;

    if(data == 0 || length == 0U) {
        return;
    }

    /*
     * 一帧必须完整进入 TX 缓冲。
     * 如果只写入半帧，上位机只能收到 CRC 错误的残帧，表现上更像“卡住”。
     */
    primask = __get_PRIMASK();
    __disable_irq();
    if(length > Uart_Tx_Free_Unlocked()) {
        s_tx_dropped += length;
        if(primask == 0U) {
            __enable_irq();
        }
        return;
    }

    for(i = 0U; i < length; i++) {
        s_tx_buffer[s_tx_head] = data[i];
        s_tx_head = Uart_Tx_Next_Index(s_tx_head);
    }
    if(primask == 0U) {
        __enable_irq();
    }

    Uart_Tx_Kick();
}

void Bms_Uart_Irq_Handler(void)
{
    uint8_t byte;
    uint16_t next;

    if(SET == usart_interrupt_flag_get(BMS_UART_PERIPH, USART_INT_FLAG_RBNE)) {
        byte = (uint8_t)(usart_data_receive(BMS_UART_PERIPH) & 0xFFU);
        next = Uart_Rx_Next_Index(s_rx_head);

        if(next == s_rx_tail) {
            s_rx_dropped++;
        } else {
            s_rx_buffer[s_rx_head] = byte;
            s_rx_head = next;
        }
    }

    Uart_Clear_Rx_Errors();

    if(SET == usart_interrupt_flag_get(BMS_UART_PERIPH, USART_INT_FLAG_TBE)) {
        if(s_tx_head == s_tx_tail) {
            usart_interrupt_disable(BMS_UART_PERIPH, USART_INT_TBE);
            return;
        }

        byte = s_tx_buffer[s_tx_tail];
        s_tx_tail = Uart_Tx_Next_Index(s_tx_tail);
        usart_data_transmit(BMS_UART_PERIPH, byte);
    }
}
