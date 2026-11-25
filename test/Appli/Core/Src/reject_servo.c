/**
 ****************************************************************************************************
 * @file        reject_servo.c
 * @brief       Reject event queue, scheduler thread, and PCA9685 servo worker.
 ****************************************************************************************************
 */

#include "reject_servo.h"
#include "pca9685.h"
#include "tx_api.h"
#include "stm32n6xx_hal.h"
#include <stdint.h>

#define REJECT_SERVO_CHANNEL             (1U)
#define REJECT_SERVO_HOME_DEG            (-80)
#define REJECT_SERVO_REJECT_DEG          (90)

#define REJECT_EVENT_QUEUE_DEPTH         (8U)
#define REJECT_ACTION_QUEUE_DEPTH        (2U)

#define REJECT_DETECTION_DELAY_MS        (1400U)
#define REJECT_DUPLICATE_WINDOW_MS       (250U)
#define REJECT_COOLDOWN_MS               (1200U)
#define REJECT_SWEEP_HOLD_MS             (1500U)
#define REJECT_HOME_HOLD_MS              (1500U)
#define REJECT_PCA_RETRY_MS              (500U)

#define REJECT_THREAD_STACK_SIZE         (1024U)
#define REJECT_THREAD_PRIORITY           (TX_MAX_PRIORITIES - 1U)

#define MS_TO_TICKS(ms)                  (((ms) * TX_TIMER_TICKS_PER_SECOND + 999U) / 1000U)

static TX_QUEUE reject_event_queue;
static TX_QUEUE reject_action_queue;
static ULONG reject_event_queue_storage[REJECT_EVENT_QUEUE_DEPTH];
static ULONG reject_action_queue_storage[REJECT_ACTION_QUEUE_DEPTH];

static TX_THREAD reject_scheduler_thread;
static TX_THREAD reject_servo_thread;
static UCHAR reject_scheduler_stack[REJECT_THREAD_STACK_SIZE];
static UCHAR reject_servo_stack[REJECT_THREAD_STACK_SIZE];

static uint8_t reject_initialized;

static VOID reject_scheduler_entry(ULONG id);
static VOID reject_servo_entry(ULONG id);
static uint32_t reject_elapsed_ms(uint32_t now, uint32_t before);
static void reject_sleep_ms(uint32_t ms);
static uint16_t reject_angle_to_pulse(int16_t angle);
static void reject_servo_write_angle(int16_t angle);
static void reject_servo_execute(void);

void reject_servo_init(void)
{
    UINT status;

    if (reject_initialized != 0U)
    {
        return;
    }

    status = tx_queue_create(&reject_event_queue, "Reject Event Queue", TX_1_ULONG,
                             reject_event_queue_storage, sizeof(reject_event_queue_storage));
    status |= tx_queue_create(&reject_action_queue, "Reject Action Queue", TX_1_ULONG,
                              reject_action_queue_storage, sizeof(reject_action_queue_storage));

    status |= tx_thread_create(&reject_scheduler_thread, "Reject Scheduler", reject_scheduler_entry, 0,
                               reject_scheduler_stack, sizeof(reject_scheduler_stack),
                               REJECT_THREAD_PRIORITY, REJECT_THREAD_PRIORITY, 10, TX_AUTO_START);
    status |= tx_thread_create(&reject_servo_thread, "Reject Servo", reject_servo_entry, 0,
                               reject_servo_stack, sizeof(reject_servo_stack),
                               REJECT_THREAD_PRIORITY, REJECT_THREAD_PRIORITY, 10, TX_AUTO_START);

    if (status == TX_SUCCESS)
    {
        reject_initialized = 1U;
    }
}

void reject_servo_request(void)
{
    ULONG event_tick = HAL_GetTick();

    if (reject_initialized == 0U)
    {
        return;
    }

    (void)tx_queue_send(&reject_event_queue, &event_tick, TX_NO_WAIT);
}

static VOID reject_scheduler_entry(ULONG id)
{
    ULONG event_tick;
    ULONG action = 1U;
    uint32_t last_event_tick = 0U;
    uint32_t last_action_tick = 0U;

    (void)id;

    while (1)
    {
        if (tx_queue_receive(&reject_event_queue, &event_tick, TX_WAIT_FOREVER) != TX_SUCCESS)
        {
            continue;
        }

        if ((last_event_tick != 0U) &&
            (reject_elapsed_ms((uint32_t)event_tick, last_event_tick) < REJECT_DUPLICATE_WINDOW_MS))
        {
            continue;
        }
        last_event_tick = (uint32_t)event_tick;

        reject_sleep_ms(REJECT_DETECTION_DELAY_MS);

        if ((last_action_tick != 0U) &&
            (reject_elapsed_ms(HAL_GetTick(), last_action_tick) < REJECT_COOLDOWN_MS))
        {
            continue;
        }

        if (tx_queue_send(&reject_action_queue, &action, TX_NO_WAIT) == TX_SUCCESS)
        {
            last_action_tick = HAL_GetTick();
        }
    }
}

static VOID reject_servo_entry(ULONG id)
{
    ULONG action;
    uint8_t pca_ready = 0U;

    (void)id;

    while (pca_ready == 0U)
    {
        if (pca9685_init(PCA9685_DEFAULT_ADDRESS, PCA9685_PWM_FREQ_HZ) == HAL_OK)
        {
            reject_servo_write_angle(REJECT_SERVO_HOME_DEG);
            pca_ready = 1U;
        }
        else
        {
            reject_sleep_ms(REJECT_PCA_RETRY_MS);
        }
    }

    while (1)
    {
        if (tx_queue_receive(&reject_action_queue, &action, TX_WAIT_FOREVER) != TX_SUCCESS)
        {
            continue;
        }

        (void)action;
        reject_servo_execute();
    }
}

static uint32_t reject_elapsed_ms(uint32_t now, uint32_t before)
{
    return now - before;
}

static void reject_sleep_ms(uint32_t ms)
{
    ULONG ticks = MS_TO_TICKS(ms);

    if (ticks == 0U)
    {
        ticks = 1U;
    }

    tx_thread_sleep(ticks);
}

static uint16_t reject_angle_to_pulse(int16_t angle)
{
    uint32_t span = PCA9685_SERVO_MAX_PULSE - PCA9685_SERVO_MIN_PULSE;
    int32_t normalized_angle;

    if (angle < -180)
    {
        angle = -180;
    }
    else if (angle > 180)
    {
        angle = 180;
    }

    normalized_angle = (int32_t)angle + 180;

    return (uint16_t)(PCA9685_SERVO_MIN_PULSE + ((span * (uint32_t)normalized_angle) / 360U));
}

static void reject_servo_write_angle(int16_t angle)
{
    (void)pca9685_set_servo_pulse(PCA9685_DEFAULT_ADDRESS, REJECT_SERVO_CHANNEL,
                                  reject_angle_to_pulse(angle));
}

static void reject_servo_execute(void)
{
    reject_servo_write_angle(REJECT_SERVO_REJECT_DEG);
    reject_sleep_ms(REJECT_SWEEP_HOLD_MS);
    reject_servo_write_angle(REJECT_SERVO_HOME_DEG);
    reject_sleep_ms(REJECT_HOME_HOLD_MS);
}
