#include "afe_gd30bm2016_transport.h"

#include "bms_board_config.h"

#include "semphr.h"
#include "task.h"

#include <string.h>

#define AFE_I2C_DELAY_CYCLES                  5000U
#define AFE_1MS_DELAY_CYCLES                  120000U

static SemaphoreHandle_t s_i2c_mutex;

static void Afe_I2c_Delay_Cycles(uint32_t cycles)
{
    volatile uint32_t i;

    for(i = 0U; i < cycles; i++) {
        __NOP();
    }
}

static void Afe_I2c_Delay(void)
{
    Afe_I2c_Delay_Cycles(AFE_I2C_DELAY_CYCLES);
}

static void Afe_I2c_Scl_High(void)
{
    gpio_bit_set(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SCL_PIN);
}

static void Afe_I2c_Scl_Low(void)
{
    gpio_bit_reset(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SCL_PIN);
}

static void Afe_I2c_Sda_Release(void)
{
    gpio_bit_set(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SDA_PIN);
    gpio_mode_set(BMS_AFE_I2C_GPIO_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_PUPD_PULLUP,
                  BMS_AFE_I2C_SDA_PIN);
}

static void Afe_I2c_Sda_Drive_Low(void)
{
    gpio_bit_reset(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SDA_PIN);
    gpio_mode_set(BMS_AFE_I2C_GPIO_PORT,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  BMS_AFE_I2C_SDA_PIN);
    gpio_output_options_set(BMS_AFE_I2C_GPIO_PORT,
                            GPIO_OTYPE_OD,
                            GPIO_OSPEED_60MHZ,
                            BMS_AFE_I2C_SDA_PIN);
}

static void Afe_I2c_Sda_High(void)
{
    Afe_I2c_Sda_Release();
}

static void Afe_I2c_Sda_Low(void)
{
    Afe_I2c_Sda_Drive_Low();
}

static uint8_t Afe_I2c_Sda_Is_High(void)
{
    return (RESET != gpio_input_bit_get(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SDA_PIN)) ? 1U : 0U;
}

static void Afe_I2c_Start(void)
{
    Afe_I2c_Sda_High();
    Afe_I2c_Scl_High();
    Afe_I2c_Delay();
    Afe_I2c_Sda_Low();
    Afe_I2c_Delay();
    Afe_I2c_Scl_Low();
    Afe_I2c_Delay();
}

static void Afe_I2c_Stop(void)
{
    Afe_I2c_Sda_Low();
    Afe_I2c_Delay();
    Afe_I2c_Scl_High();
    Afe_I2c_Delay();
    Afe_I2c_Sda_High();
    Afe_I2c_Delay();
}

static void Afe_I2c_Bus_Recover(void)
{
    uint8_t i;

    Afe_I2c_Sda_High();
    for(i = 0U; i < 9U; i++) {
        Afe_I2c_Scl_Low();
        Afe_I2c_Delay();
        Afe_I2c_Scl_High();
        Afe_I2c_Delay();
    }
    Afe_I2c_Stop();
}

static uint8_t Afe_I2c_Write_Byte(uint8_t value)
{
    uint8_t i;
    uint8_t ack;

    for(i = 0U; i < 8U; i++) {
        if((value & 0x80U) != 0U) {
            Afe_I2c_Sda_High();
        } else {
            Afe_I2c_Sda_Low();
        }
        Afe_I2c_Delay();
        Afe_I2c_Scl_High();
        Afe_I2c_Delay();
        Afe_I2c_Scl_Low();
        Afe_I2c_Delay();
        value <<= 1U;
    }

    Afe_I2c_Sda_High();
    Afe_I2c_Delay();
    Afe_I2c_Scl_High();
    Afe_I2c_Delay();
    ack = (Afe_I2c_Sda_Is_High() == 0U) ? 1U : 0U;
    Afe_I2c_Scl_Low();
    Afe_I2c_Delay();
    Afe_I2c_Sda_High();
    Afe_I2c_Delay();

    return ack;
}

static uint8_t Afe_I2c_Read_Byte(uint8_t ack)
{
    uint8_t i;
    uint8_t value;

    value = 0U;
    Afe_I2c_Sda_High();
    for(i = 0U; i < 8U; i++) {
        value <<= 1U;
        Afe_I2c_Delay();
        Afe_I2c_Scl_High();
        Afe_I2c_Delay();
        if(Afe_I2c_Sda_Is_High() != 0U) {
            value |= 1U;
        }
        Afe_I2c_Scl_Low();
        Afe_I2c_Delay();
    }

    if(ack != 0U) {
        Afe_I2c_Sda_Low();
    } else {
        Afe_I2c_Sda_High();
    }
    Afe_I2c_Delay();
    Afe_I2c_Scl_High();
    Afe_I2c_Delay();
    Afe_I2c_Scl_Low();
    Afe_I2c_Delay();
    Afe_I2c_Sda_High();

    return value;
}

void Afe_Gd30bm2016_Transport_Init(void)
{
    if(s_i2c_mutex == 0) {
        s_i2c_mutex = xSemaphoreCreateMutex();
        configASSERT(s_i2c_mutex != 0);
    }
}

uint8_t Afe_Gd30bm2016_Transport_Lock(TickType_t timeout)
{
    if(s_i2c_mutex == 0) {
        return 1U;
    }
    if(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return 1U;
    }

    return (xSemaphoreTake(s_i2c_mutex, timeout) == pdTRUE) ? 1U : 0U;
}

void Afe_Gd30bm2016_Transport_Unlock(void)
{
    if((s_i2c_mutex != 0) &&
       (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) {
        (void)xSemaphoreGive(s_i2c_mutex);
    }
}

void Afe_Gd30bm2016_Transport_Delay_Ms(uint32_t ms)
{
    uint32_t i;

    if((ms != 0U) &&
       (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)) {
        vTaskDelay(pdMS_TO_TICKS(ms));
        return;
    }

    for(i = 0U; i < ms; i++) {
        Afe_I2c_Delay_Cycles(AFE_1MS_DELAY_CYCLES);
    }
}

void Afe_Gd30bm2016_Transport_Gpio_Init(void)
{
    rcu_periph_clock_enable(BMS_AFE_I2C_GPIO_CLK);
    rcu_periph_clock_enable(BMS_AFE_RST_GPIO_CLK);
    rcu_periph_clock_enable(BMS_AFE_ALERT_GPIO_CLK);
    rcu_periph_clock_enable(BMS_AFE_DDSG_DCHG_GPIO_CLK);
    rcu_periph_clock_enable(BMS_AFE_DFETOFF_GPIO_CLK);

    gpio_bit_reset(BMS_AFE_RST_GPIO_PORT, BMS_AFE_RST_PIN);
    gpio_mode_set(BMS_AFE_RST_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, BMS_AFE_RST_PIN);
    gpio_output_options_set(BMS_AFE_RST_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, BMS_AFE_RST_PIN);
    gpio_bit_reset(BMS_AFE_RST_GPIO_PORT, BMS_AFE_RST_PIN);
    Afe_Gd30bm2016_Transport_Delay_Ms(5U);

    gpio_bit_reset(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_ADDR0_PIN | BMS_AFE_I2C_ADDR1_PIN);
    gpio_mode_set(BMS_AFE_I2C_GPIO_PORT,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_PULLDOWN,
                  BMS_AFE_I2C_ADDR0_PIN | BMS_AFE_I2C_ADDR1_PIN);
    gpio_output_options_set(BMS_AFE_I2C_GPIO_PORT,
                            GPIO_OTYPE_PP,
                            GPIO_OSPEED_60MHZ,
                            BMS_AFE_I2C_ADDR0_PIN | BMS_AFE_I2C_ADDR1_PIN);

    gpio_bit_set(BMS_AFE_I2C_GPIO_PORT, BMS_AFE_I2C_SCL_PIN | BMS_AFE_I2C_SDA_PIN);
    gpio_mode_set(BMS_AFE_I2C_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, BMS_AFE_I2C_SCL_PIN);
    gpio_output_options_set(BMS_AFE_I2C_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, BMS_AFE_I2C_SCL_PIN);
    Afe_I2c_Sda_Release();
    Afe_I2c_Bus_Recover();

    gpio_mode_set(BMS_AFE_ALERT_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BMS_AFE_ALERT_PIN);
    gpio_mode_set(BMS_AFE_DDSG_DCHG_GPIO_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, BMS_AFE_DDSG_PIN | BMS_AFE_DCHG_PIN);
    gpio_bit_reset(BMS_AFE_DFETOFF_GPIO_PORT, BMS_AFE_DFETOFF_PIN);
    gpio_mode_set(BMS_AFE_DFETOFF_GPIO_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BMS_AFE_DFETOFF_PIN);
    gpio_output_options_set(BMS_AFE_DFETOFF_GPIO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_60MHZ, BMS_AFE_DFETOFF_PIN);
}

uint8_t Afe_Gd30bm2016_Transport_Write_Raw(uint8_t address,
                                           uint8_t reg_addr,
                                           const uint8_t *data,
                                           uint8_t length)
{
    uint8_t ok;

    if((data == 0) || (length == 0U)) {
        return 0U;
    }

    ok = 1U;
    Afe_I2c_Start();
    ok = Afe_I2c_Write_Byte(address);
    if(ok != 0U) {
        ok = Afe_I2c_Write_Byte(reg_addr);
    }
    while((length-- != 0U) && (ok != 0U)) {
        ok = Afe_I2c_Write_Byte(*data);
        data++;
    }
    Afe_I2c_Stop();

    return ok;
}

uint8_t Afe_Gd30bm2016_Transport_Read_Raw(uint8_t address,
                                          uint8_t reg_addr,
                                          uint8_t *data,
                                          uint8_t length)
{
    uint8_t ok;
    uint8_t i;

    if((data == 0) || (length == 0U)) {
        return 0U;
    }

    ok = 1U;
    Afe_I2c_Start();
    ok = Afe_I2c_Write_Byte(address);
    if(ok != 0U) {
        ok = Afe_I2c_Write_Byte(reg_addr);
    }
    if(ok != 0U) {
        Afe_I2c_Start();
        ok = Afe_I2c_Write_Byte((uint8_t)(address | 0x01U));
    }
    for(i = 0U; (i < length) && (ok != 0U); i++) {
        data[i] = Afe_I2c_Read_Byte((i + 1U) < length);
    }
    Afe_I2c_Stop();

    if(ok == 0U) {
        memset(data, 0, length);
    }

    return ok;
}
