/**
 ****************************************************************************************************
 * @file        reject_servo.h
 * @brief       Bottle-cap reject event scheduler and servo worker.
 ****************************************************************************************************
 */

#ifndef __REJECT_SERVO_H
#define __REJECT_SERVO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void reject_servo_init(void);
void reject_servo_request(void);

#ifdef __cplusplus
}
#endif

#endif /* __REJECT_SERVO_H */
