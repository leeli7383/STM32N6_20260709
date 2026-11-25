/**
 ****************************************************************************************************
 * @file        pca9685.h
 * @brief       PCA9685 software-I2C driver on PB7(SDA)/PB6(SCL).
 ****************************************************************************************************
 */

#ifndef __PCA9685_H
#define __PCA9685_H

#include "stm32n6xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PCA9685_DEFAULT_ADDRESS      (0x40U)
#define PCA9685_PWM_FREQ_HZ          (60U)
#define PCA9685_SERVO_MIN_PULSE      (150U)
#define PCA9685_SERVO_MAX_PULSE      (600U)

HAL_StatusTypeDef pca9685_init(uint8_t address, uint16_t pwm_freq_hz);
HAL_StatusTypeDef pca9685_set_pwm(uint8_t address, uint8_t channel, uint16_t on, uint16_t off);
HAL_StatusTypeDef pca9685_set_servo_pulse(uint8_t address, uint8_t channel, uint16_t pulse);

#ifdef __cplusplus
}
#endif

#endif /* __PCA9685_H */
