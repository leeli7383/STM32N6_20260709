/**
 ****************************************************************************************************
 * @file        pca9685.c
 * @brief       PCA9685 software-I2C driver on PB7(SDA)/PB6(SCL).
 ****************************************************************************************************
 */

#include "pca9685.h"
#include <stdint.h>

#define PCA9685_SCL_PORT             GPIOB
#define PCA9685_SCL_PIN              GPIO_PIN_6
#define PCA9685_SDA_PORT             GPIOB
#define PCA9685_SDA_PIN              GPIO_PIN_7

#define PCA9685_MODE1                (0x00U)
#define PCA9685_MODE2                (0x01U)
#define PCA9685_LED0_ON_L            (0x06U)
#define PCA9685_PRESCALE             (0xFEU)

#define PCA9685_MODE1_RESTART        (0x80U)
#define PCA9685_MODE1_AI             (0x20U)
#define PCA9685_MODE1_SLEEP          (0x10U)
#define PCA9685_MODE2_OUTDRV         (0x04U)

#define PCA9685_OSC_CLOCK_HZ         (25000000UL)
#define PCA9685_I2C_WRITE_TIMEOUT    (256U)

static void pca9685_i2c_gpio_init(void);
static void pca9685_i2c_delay(void);
static void pca9685_scl_write(GPIO_PinState state);
static void pca9685_sda_write(GPIO_PinState state);
static GPIO_PinState pca9685_sda_read(void);
static void pca9685_i2c_start(void);
static void pca9685_i2c_stop(void);
static uint8_t pca9685_i2c_write_byte(uint8_t data);
static HAL_StatusTypeDef pca9685_write_reg(uint8_t address, uint8_t reg, uint8_t value);
static HAL_StatusTypeDef pca9685_write_regs(uint8_t address, uint8_t reg, const uint8_t *data, uint32_t len);

HAL_StatusTypeDef pca9685_init(uint8_t address, uint16_t pwm_freq_hz)
{
    uint32_t prescale;

    if (pwm_freq_hz == 0U)
    {
        return HAL_ERROR;
    }

    pca9685_i2c_gpio_init();

    if (pca9685_write_reg(address, PCA9685_MODE1, PCA9685_MODE1_SLEEP) != HAL_OK)
    {
        return HAL_ERROR;
    }

    prescale = ((PCA9685_OSC_CLOCK_HZ + (2048UL * (uint32_t)pwm_freq_hz)) /
                (4096UL * (uint32_t)pwm_freq_hz)) - 1UL;
    if (prescale > 255UL)
    {
        prescale = 255UL;
    }

    if (pca9685_write_reg(address, PCA9685_PRESCALE, (uint8_t)prescale) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (pca9685_write_reg(address, PCA9685_MODE2, PCA9685_MODE2_OUTDRV) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (pca9685_write_reg(address, PCA9685_MODE1, PCA9685_MODE1_AI) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(1);
    return pca9685_write_reg(address, PCA9685_MODE1, PCA9685_MODE1_RESTART | PCA9685_MODE1_AI);
}

HAL_StatusTypeDef pca9685_set_pwm(uint8_t address, uint8_t channel, uint16_t on, uint16_t off)
{
    uint8_t data[4];

    if (channel >= 16U)
    {
        return HAL_ERROR;
    }

    on &= 0x0FFFU;
    off &= 0x0FFFU;

    data[0] = (uint8_t)(on & 0xFFU);
    data[1] = (uint8_t)((on >> 8) & 0x0FU);
    data[2] = (uint8_t)(off & 0xFFU);
    data[3] = (uint8_t)((off >> 8) & 0x0FU);

    return pca9685_write_regs(address, (uint8_t)(PCA9685_LED0_ON_L + (4U * channel)), data, sizeof(data));
}

HAL_StatusTypeDef pca9685_set_servo_pulse(uint8_t address, uint8_t channel, uint16_t pulse)
{
    if (pulse < PCA9685_SERVO_MIN_PULSE)
    {
        pulse = PCA9685_SERVO_MIN_PULSE;
    }
    else if (pulse > PCA9685_SERVO_MAX_PULSE)
    {
        pulse = PCA9685_SERVO_MAX_PULSE;
    }

    return pca9685_set_pwm(address, channel, 0U, pulse);
}

static void pca9685_i2c_gpio_init(void)
{
    GPIO_InitTypeDef gpio_init = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(PCA9685_SCL_PORT, PCA9685_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(PCA9685_SDA_PORT, PCA9685_SDA_PIN, GPIO_PIN_SET);

    gpio_init.Pin = PCA9685_SCL_PIN | PCA9685_SDA_PIN;
    gpio_init.Mode = GPIO_MODE_OUTPUT_OD;
    gpio_init.Pull = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio_init);
}

static void pca9685_i2c_delay(void)
{
    volatile uint32_t delay = 80U;

    while (delay-- != 0U)
    {
        __NOP();
    }
}

static void pca9685_scl_write(GPIO_PinState state)
{
    HAL_GPIO_WritePin(PCA9685_SCL_PORT, PCA9685_SCL_PIN, state);
    pca9685_i2c_delay();
}

static void pca9685_sda_write(GPIO_PinState state)
{
    HAL_GPIO_WritePin(PCA9685_SDA_PORT, PCA9685_SDA_PIN, state);
    pca9685_i2c_delay();
}

static GPIO_PinState pca9685_sda_read(void)
{
    pca9685_i2c_delay();
    return HAL_GPIO_ReadPin(PCA9685_SDA_PORT, PCA9685_SDA_PIN);
}

static void pca9685_i2c_start(void)
{
    pca9685_sda_write(GPIO_PIN_SET);
    pca9685_scl_write(GPIO_PIN_SET);
    pca9685_sda_write(GPIO_PIN_RESET);
    pca9685_scl_write(GPIO_PIN_RESET);
}

static void pca9685_i2c_stop(void)
{
    pca9685_sda_write(GPIO_PIN_RESET);
    pca9685_scl_write(GPIO_PIN_SET);
    pca9685_sda_write(GPIO_PIN_SET);
}

static uint8_t pca9685_i2c_write_byte(uint8_t data)
{
    uint32_t timeout;

    for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1)
    {
        pca9685_sda_write((data & mask) != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
        pca9685_scl_write(GPIO_PIN_SET);
        pca9685_scl_write(GPIO_PIN_RESET);
    }

    pca9685_sda_write(GPIO_PIN_SET);
    pca9685_scl_write(GPIO_PIN_SET);

    timeout = PCA9685_I2C_WRITE_TIMEOUT;
    while ((pca9685_sda_read() == GPIO_PIN_SET) && (timeout-- != 0U))
    {
    }

    pca9685_scl_write(GPIO_PIN_RESET);
    return timeout != 0U;
}

static HAL_StatusTypeDef pca9685_write_reg(uint8_t address, uint8_t reg, uint8_t value)
{
    return pca9685_write_regs(address, reg, &value, 1U);
}

static HAL_StatusTypeDef pca9685_write_regs(uint8_t address, uint8_t reg, const uint8_t *data, uint32_t len)
{
    pca9685_i2c_start();

    if ((pca9685_i2c_write_byte((uint8_t)(address << 1)) == 0U) ||
        (pca9685_i2c_write_byte(reg) == 0U))
    {
        pca9685_i2c_stop();
        return HAL_ERROR;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        if (pca9685_i2c_write_byte(data[i]) == 0U)
        {
            pca9685_i2c_stop();
            return HAL_ERROR;
        }
    }

    pca9685_i2c_stop();
    return HAL_OK;
}
