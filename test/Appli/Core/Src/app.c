/**
 ****************************************************************************************************
 * @file        app.c
 * @brief       YOLO defect detection application for STM32N6.
 ****************************************************************************************************
 */

#include "app.h"
#include "app_config.h"
#include "app_utils.h"
#include "app_lcd.h"
#include "app_camera.h"
#include "app_bqueue.h"
#include "app_cpuload.h"
#include "reject_servo.h"
#include "tx_api.h"
#include "cmw_camera.h"
#include "ll_aton_runtime.h"
#include "network.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

#define YOLO_BOX_NB                 (2100U)
#define YOLO_CLASS_NB               (2U)
#define YOLO_SCORE_THRESHOLD        (0.35f)
#define YOLO_IOU_THRESHOLD          (0.45f)
#define YOLO_MAX_CANDIDATES         (40U)
#define YOLO_MAX_DETECTIONS         (8U)
#define YOLO_INPUT_ZERO_POINT       (-128)
#define YOLO_PAD_VALUE_U8           (114U)
#define YOLO_OUTPUT_SCALE           (0.0056975498f)
#define YOLO_OUTPUT_ZERO_POINT      (-128)

#define UI_PANEL_WIDTH              (LCD_FG_WIDTH / 2U)
#define UI_PANEL_HEIGHT             LCD_FG_HEIGHT
#define UI_CAMERA_X0                UI_PANEL_WIDTH
#define UI_PANEL_COLOR              0xFF101820U
#define UI_PANEL_HEADER_COLOR       0xFF1F6FEBU
#define UI_PANEL_CARD_COLOR         0xFF1E2A36U
#define UI_PANEL_BUTTON_COLOR       0xFF2D6CDFU
#define UI_PANEL_BUTTON_ALT_COLOR   0xFF384654U
#define UI_PANEL_TEXT_COLOR         UTIL_LCD_COLOR_WHITE
#define UI_PANEL_ACCENT_COLOR       0xFF53D769U
#define UI_PANEL_LINE_COLOR         0xFF6B7785U

typedef struct
{
    float x0;
    float y0;
    float x1;
    float y1;
    float score;
    uint32_t class_id;
} app_detection_t;

typedef struct
{
    int8_t *nn_in;
    uint32_t nn_in_len;
    int8_t *nn_out;
    uint32_t nn_out_len;
    float nn_out_scale;
    int32_t nn_out_zero_point;
} app_model_info_t;

typedef struct
{
    float nn_period_ms;
    uint32_t inference_ms;
    uint32_t preprocess_ms;
    uint32_t postprocess_ms;
    uint32_t display_ms;
    uint32_t detection_nb;
    float max_score;
    app_detection_t detections[YOLO_MAX_DETECTIONS];
} app_display_info_t;

typedef struct
{
    TX_SEMAPHORE update;
    TX_MUTEX lock;
    app_display_info_t info;
} app_display_t;

static TX_SEMAPHORE isp_semaphore;

static void app_camera_display_pipe_vsync_cb(void);
static void app_camera_display_pipe_frame_cb(void);
static void app_camera_nn_pipe_frame_cb(void);

static TX_THREAD nn_thread;
static UCHAR nn_thread_stack[4096];
static TX_THREAD dp_thread;
static UCHAR dp_thread_stack[4096];
static TX_THREAD isp_thread;
static UCHAR isp_thread_stack[4096];

static VOID nn_thread_entry(ULONG id);
static VOID dp_thread_entry(ULONG id);
static VOID isp_thread_entry(ULONG id);

static app_display_t display;

static uint8_t nn_input_buffers[2][NN_WIDTH * NN_HEIGHT * NN_BPP] __attribute__((aligned(32))) __attribute__((section(".EXTRAM")));
static app_bqueue_t nn_input_queue;

LL_ATON_DECLARE_NAMED_NN_INSTANCE_AND_INTERFACE(network);

static app_cpuload_t cpuload;

static const char *const yolo_class_labels[YOLO_CLASS_NB] = {
    "defect1",
    "defect2",
};

static void app_model_init(app_model_info_t *info);
static void app_model_prepare_input(uint8_t *buffer, app_model_info_t *info);
static uint32_t app_model_run(uint8_t *buffer, app_model_info_t *info, uint32_t *preprocess_ms, uint32_t *inference_ms,
                              uint32_t *postprocess_ms, app_detection_t detections[YOLO_MAX_DETECTIONS], float *max_score);
static uint32_t app_yolo_postprocess(app_model_info_t *info, app_detection_t detections[YOLO_MAX_DETECTIONS],
                                     float *max_score);
static float app_output_value(app_model_info_t *info, uint32_t index);
static void app_display_network_output(app_display_info_t *display_info);
static void app_draw_control_panel(void);
static void app_draw_control_button(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color,
                                    const char *label);
static void app_draw_detection(app_detection_t *det);
static float app_box_iou(app_detection_t *a, app_detection_t *b);
static float app_clampf(float value, float min_value, float max_value);

void app_run(void)
{
    app_lcd_init();
    app_bqueue_init(&nn_input_queue, 2, (uint8_t *[2]){nn_input_buffers[0], nn_input_buffers[1]});
    app_cpuload_init(&cpuload);
    app_camera_init(app_camera_display_pipe_vsync_cb, app_camera_display_pipe_frame_cb, NULL, app_camera_nn_pipe_frame_cb);

    tx_semaphore_create(&isp_semaphore, NULL, 0);
    tx_semaphore_create(&display.update, NULL, 0);
    tx_mutex_create(&display.lock, NULL, TX_INHERIT);
    reject_servo_init();

    app_camera_display_pipe_start(app_lcd_get_bg_buffer(), CMW_MODE_CONTINUOUS);

    tx_thread_create(&nn_thread, "NN Thread", nn_thread_entry, 0, nn_thread_stack, sizeof(nn_thread_stack),
                     TX_MAX_PRIORITIES - 3, TX_MAX_PRIORITIES - 3, 10, TX_AUTO_START);
    tx_thread_create(&dp_thread, "DP Thread", dp_thread_entry, 0, dp_thread_stack, sizeof(dp_thread_stack),
                     TX_MAX_PRIORITIES - 2, TX_MAX_PRIORITIES - 2, 10, TX_AUTO_START);
    tx_thread_create(&isp_thread, "ISP Thread", isp_thread_entry, 0, isp_thread_stack, sizeof(isp_thread_stack),
                     TX_MAX_PRIORITIES - 4, TX_MAX_PRIORITIES - 4, 10, TX_AUTO_START);
}

static void app_camera_display_pipe_vsync_cb(void)
{
    tx_semaphore_put(&isp_semaphore);
}

static void app_camera_display_pipe_frame_cb(void)
{
    app_lcd_switch_bg_buffer();
    app_camera_display_pipe_set_address(app_lcd_get_bg_buffer());
}

static void app_camera_nn_pipe_frame_cb(void)
{
    uint8_t *buffer;

    buffer = app_bqueue_get_free(&nn_input_queue, 0);
    if (buffer != NULL)
    {
        app_camera_nn_pipe_set_address(buffer);
        app_bqueue_put_ready(&nn_input_queue);
    }
}

static VOID nn_thread_entry(ULONG id)
{
    app_model_info_t model_info;
    uint8_t *nn_pipe_dst;
    uint8_t *capture_buffer;
    uint32_t nn_period[2];
    uint32_t nn_period_ms;
    float nn_period_filtered_ms = 0.0f;
    uint32_t preprocess_ms;
    uint32_t inference_ms;
    uint32_t postprocess_ms;
    uint32_t detection_nb;
    float max_score;
    app_detection_t detections[YOLO_MAX_DETECTIONS];

    (void)id;

    app_model_init(&model_info);

    nn_period[1] = HAL_GetTick();
    nn_pipe_dst = app_bqueue_get_free(&nn_input_queue, 0);
    app_camera_nn_pipe_start(nn_pipe_dst, CMW_MODE_CONTINUOUS);

    while (1)
    {
        nn_period[0] = nn_period[1];
        nn_period[1] = HAL_GetTick();
        nn_period_ms = nn_period[1] - nn_period[0];
        nn_period_filtered_ms = (15.0f * nn_period_filtered_ms + (float)nn_period_ms) / 16.0f;

        capture_buffer = app_bqueue_get_ready(&nn_input_queue);
        detection_nb = app_model_run(capture_buffer, &model_info, &preprocess_ms, &inference_ms, &postprocess_ms,
                                     detections, &max_score);
        app_bqueue_put_free(&nn_input_queue);

        if (detection_nb > 0U)
        {
            reject_servo_request();
        }

        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        display.info.nn_period_ms = nn_period_filtered_ms;
        display.info.preprocess_ms = preprocess_ms;
        display.info.inference_ms = inference_ms;
        display.info.postprocess_ms = postprocess_ms;
        display.info.detection_nb = detection_nb;
        display.info.max_score = max_score;
        memcpy(display.info.detections, detections, sizeof(display.info.detections));
        tx_mutex_put(&display.lock);

        tx_semaphore_ceiling_put(&display.update, 1);
    }
}

static VOID dp_thread_entry(ULONG id)
{
    uint32_t display_ms = 0;
    uint32_t time_stamp;
    app_display_info_t info;

    (void)id;

    while (1)
    {
        tx_semaphore_get(&display.update, TX_WAIT_FOREVER);

        tx_mutex_get(&display.lock, TX_WAIT_FOREVER);
        info = display.info;
        tx_mutex_put(&display.lock);
        info.display_ms = display_ms;

        time_stamp = HAL_GetTick();
        app_display_network_output(&info);
        display_ms = HAL_GetTick() - time_stamp;
    }
}

static VOID isp_thread_entry(ULONG id)
{
    (void)id;

    while (1)
    {
        tx_semaphore_get(&isp_semaphore, TX_WAIT_FOREVER);
        app_camera_isp_update();
    }
}

static void app_model_init(app_model_info_t *info)
{
    const LL_Buffer_InfoTypeDef *nn_in_info = LL_ATON_Input_Buffers_Info_network();
    const LL_Buffer_InfoTypeDef *nn_out_info = LL_ATON_Output_Buffers_Info_network();

    info->nn_in = (int8_t *)LL_Buffer_addr_start(&nn_in_info[0]);
    info->nn_in_len = LL_Buffer_len(&nn_in_info[0]);
    info->nn_out = (int8_t *)LL_Buffer_addr_start(&nn_out_info[0]);
    info->nn_out_len = LL_Buffer_len(&nn_out_info[0]);
    info->nn_out_scale = YOLO_OUTPUT_SCALE;
    info->nn_out_zero_point = YOLO_OUTPUT_ZERO_POINT;
}

static void app_model_prepare_input(uint8_t *buffer, app_model_info_t *info)
{
    const uint32_t src_w = NN_WIDTH;
    const uint32_t src_h = NN_CAPTURE_HEIGHT;
    const uint32_t dst_w = NN_WIDTH;
    const uint32_t dst_h = NN_HEIGHT;
    const uint32_t y_offset = (dst_h - src_h) / 2U;
    const int8_t pad_value = (int8_t)((int32_t)YOLO_PAD_VALUE_U8 + YOLO_INPUT_ZERO_POINT);
    int8_t *dst = info->nn_in;
    uint32_t x;
    uint32_t y;

    memset(dst, (uint8_t)pad_value, dst_w * dst_h * NN_BPP);

    for (y = 0; y < src_h; y++)
    {
        uint8_t *src = buffer + y * src_w * NN_BPP;
        int8_t *dst_row = dst + (y + y_offset) * dst_w * NN_BPP;

        for (x = 0; x < src_w; x++)
        {
            dst_row[x * NN_BPP + 0] = (int8_t)((int32_t)src[x * NN_BPP + 0] + YOLO_INPUT_ZERO_POINT);
            dst_row[x * NN_BPP + 1] = (int8_t)((int32_t)src[x * NN_BPP + 1] + YOLO_INPUT_ZERO_POINT);
            dst_row[x * NN_BPP + 2] = (int8_t)((int32_t)src[x * NN_BPP + 2] + YOLO_INPUT_ZERO_POINT);
        }
    }
}

static uint32_t app_model_run(uint8_t *buffer, app_model_info_t *info, uint32_t *preprocess_ms, uint32_t *inference_ms,
                              uint32_t *postprocess_ms, app_detection_t detections[YOLO_MAX_DETECTIONS], float *max_score)
{
    uint32_t time_stamp;
    uint32_t detection_nb;

    time_stamp = HAL_GetTick();
    app_model_prepare_input(buffer, info);
    SCB_CleanInvalidateDCache_by_Addr(info->nn_in, info->nn_in_len);
    *preprocess_ms = HAL_GetTick() - time_stamp;

    time_stamp = HAL_GetTick();
    LL_ATON_RT_Main(&NN_Instance_network);
    SCB_InvalidateDCache_by_Addr(info->nn_out, info->nn_out_len);
    *inference_ms = HAL_GetTick() - time_stamp;

    time_stamp = HAL_GetTick();
    detection_nb = app_yolo_postprocess(info, detections, max_score);
    *postprocess_ms = HAL_GetTick() - time_stamp;

    return detection_nb;
}

static uint32_t app_yolo_postprocess(app_model_info_t *info, app_detection_t detections[YOLO_MAX_DETECTIONS],
                                     float *max_score)
{
    app_detection_t candidates[YOLO_MAX_CANDIDATES];
    uint32_t candidate_nb = 0;
    uint32_t detection_nb = 0;
    const uint32_t y_offset = (NN_HEIGHT - NN_CAPTURE_HEIGHT) / 2U;

    *max_score = 0.0f;
    memset(detections, 0, sizeof(app_detection_t) * YOLO_MAX_DETECTIONS);

    for (uint32_t i = 0; i < YOLO_BOX_NB; i++)
    {
        float score = app_output_value(info, 4U * YOLO_BOX_NB + i);
        uint32_t class_id = 0U;
        float cx;
        float cy;
        float w;
        float h;
        app_detection_t det;

        for (uint32_t c = 1U; c < YOLO_CLASS_NB; c++)
        {
            float class_score = app_output_value(info, (4U + c) * YOLO_BOX_NB + i);
            if (class_score > score)
            {
                score = class_score;
                class_id = c;
            }
        }

        if ((score < YOLO_SCORE_THRESHOLD) || !isfinite(score))
        {
            continue;
        }

        cx = app_output_value(info, 0U * YOLO_BOX_NB + i) * (float)NN_WIDTH;
        cy = app_output_value(info, 1U * YOLO_BOX_NB + i) * (float)NN_HEIGHT;
        w = app_output_value(info, 2U * YOLO_BOX_NB + i) * (float)NN_WIDTH;
        h = app_output_value(info, 3U * YOLO_BOX_NB + i) * (float)NN_HEIGHT;

        if ((w <= 1.0f) || (h <= 1.0f) || !isfinite(cx) || !isfinite(cy) || !isfinite(w) || !isfinite(h))
        {
            continue;
        }

        det.x0 = app_clampf(cx - w * 0.5f, 0.0f, (float)(NN_WIDTH - 1U));
        det.y0 = app_clampf(cy - h * 0.5f, 0.0f, (float)(NN_HEIGHT - 1U));
        det.x1 = app_clampf(cx + w * 0.5f, 0.0f, (float)(NN_WIDTH - 1U));
        det.y1 = app_clampf(cy + h * 0.5f, 0.0f, (float)(NN_HEIGHT - 1U));
        det.score = score;
        det.class_id = class_id;

        if ((det.y1 <= (float)y_offset) || (det.y0 >= (float)(y_offset + NN_CAPTURE_HEIGHT)))
        {
            continue;
        }

        *max_score = MAX(*max_score, score);

        if (candidate_nb < YOLO_MAX_CANDIDATES)
        {
            candidates[candidate_nb++] = det;
        }
        else
        {
            uint32_t min_idx = 0;
            for (uint32_t j = 1; j < candidate_nb; j++)
            {
                if (candidates[j].score < candidates[min_idx].score)
                {
                    min_idx = j;
                }
            }
            if (score > candidates[min_idx].score)
            {
                candidates[min_idx] = det;
            }
        }
    }

    for (uint32_t i = 0; i < candidate_nb; i++)
    {
        for (uint32_t j = i + 1; j < candidate_nb; j++)
        {
            if (candidates[j].score > candidates[i].score)
            {
                app_detection_t tmp = candidates[i];
                candidates[i] = candidates[j];
                candidates[j] = tmp;
            }
        }
    }

    for (uint32_t i = 0; (i < candidate_nb) && (detection_nb < YOLO_MAX_DETECTIONS); i++)
    {
        uint8_t keep = 1;

        for (uint32_t j = 0; j < detection_nb; j++)
        {
            if ((candidates[i].class_id == detections[j].class_id) &&
                (app_box_iou(&candidates[i], &detections[j]) > YOLO_IOU_THRESHOLD))
            {
                keep = 0;
                break;
            }
        }

        if (keep != 0)
        {
            detections[detection_nb++] = candidates[i];
        }
    }

    return detection_nb;
}

static float app_output_value(app_model_info_t *info, uint32_t index)
{
    return ((float)((int32_t)info->nn_out[index] - info->nn_out_zero_point)) * info->nn_out_scale;
}

static void app_display_network_output(app_display_info_t *display_info)
{
    app_lcd_lock();
    app_lcd_draw_area_update();
    UTIL_LCD_FillRect(0, 0, LCD_FG_WIDTH, LCD_FG_HEIGHT, 0x00000000);
    app_draw_control_panel();

    for (uint32_t i = 0; i < display_info->detection_nb; i++)
    {
        app_draw_detection(&display_info->detections[i]);
    }

    app_lcd_draw_area_commit();
    app_lcd_unlock();
}

static void app_draw_control_panel(void)
{
    UTIL_LCD_FillRect(0, 0, UI_PANEL_WIDTH, UI_PANEL_HEIGHT, UI_PANEL_COLOR);
    UTIL_LCD_FillRect(0, 0, UI_PANEL_WIDTH, 64, UI_PANEL_HEADER_COLOR);
    UTIL_LCD_FillRect(UI_PANEL_WIDTH - 2U, 0, 2, UI_PANEL_HEIGHT, UI_PANEL_LINE_COLOR);

    UTIL_LCD_SetTextColor(UI_PANEL_TEXT_COLOR);
    UTIL_LCDEx_PrintfAt(22, 22, LEFT_MODE, "Defect Control");

    UTIL_LCD_FillRect(24, 88, UI_PANEL_WIDTH - 48U, 128, UI_PANEL_CARD_COLOR);
    UTIL_LCDEx_PrintfAt(44, 108, LEFT_MODE, "Conveyor: ON");
    UTIL_LCDEx_PrintfAt(44, 148, LEFT_MODE, "Gate:      OFF");
    UTIL_LCDEx_PrintfAt(44, 188, LEFT_MODE, "System:    RUN");

    UTIL_LCD_SetTextColor(UI_PANEL_ACCENT_COLOR);
    UTIL_LCDEx_PrintfAt(28, 242, LEFT_MODE, "Controls");
    UTIL_LCD_SetTextColor(UI_PANEL_TEXT_COLOR);

    app_draw_control_button(28, 276, 164, 52, UI_PANEL_BUTTON_COLOR, "Conveyor");
    app_draw_control_button(208, 276, 164, 52, UI_PANEL_BUTTON_ALT_COLOR, "Gate");
    app_draw_control_button(28, 352, 164, 52, UI_PANEL_BUTTON_COLOR, "Start");
    app_draw_control_button(208, 352, 164, 52, UI_PANEL_BUTTON_ALT_COLOR, "Stop");
}

static void app_draw_control_button(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint32_t color,
                                    const char *label)
{
    UTIL_LCD_FillRect(x, y, w, h, color);
    UTIL_LCD_DrawLine(x, y, x + w - 1U, y, UI_PANEL_LINE_COLOR);
    UTIL_LCD_DrawLine(x + w - 1U, y, x + w - 1U, y + h - 1U, UI_PANEL_LINE_COLOR);
    UTIL_LCD_DrawLine(x + w - 1U, y + h - 1U, x, y + h - 1U, UI_PANEL_LINE_COLOR);
    UTIL_LCD_DrawLine(x, y + h - 1U, x, y, UI_PANEL_LINE_COLOR);
    UTIL_LCD_SetTextColor(UI_PANEL_TEXT_COLOR);
    UTIL_LCDEx_PrintfAt(x + 24U, y + 16U, LEFT_MODE, label);
}

static void app_draw_detection(app_detection_t *det)
{
    const float y_offset = (float)((NN_HEIGHT - NN_CAPTURE_HEIGHT) / 2U);
    const char *label = det->class_id < YOLO_CLASS_NB ? yolo_class_labels[det->class_id] : "defect";
    int x0 = (int)(det->x0 * (float)LCD_BG_WIDTH / (float)NN_WIDTH);
    int x1 = (int)(det->x1 * (float)LCD_BG_WIDTH / (float)NN_WIDTH);
    int y0 = (int)((det->y0 - y_offset) * (float)LCD_BG_HEIGHT / (float)NN_CAPTURE_HEIGHT);
    int y1 = (int)((det->y1 - y_offset) * (float)LCD_BG_HEIGHT / (float)NN_CAPTURE_HEIGHT);

    x0 = (int)app_clampf((float)x0, 0.0f, (float)(LCD_BG_WIDTH - 1U));
    x1 = (int)app_clampf((float)x1, 0.0f, (float)(LCD_BG_WIDTH - 1U));
    y0 = (int)app_clampf((float)y0, 0.0f, (float)(LCD_BG_HEIGHT - 1U));
    y1 = (int)app_clampf((float)y1, 0.0f, (float)(LCD_BG_HEIGHT - 1U));

    if (x1 < (int)UI_CAMERA_X0)
    {
        return;
    }

    x0 = MAX(x0, (int)UI_CAMERA_X0);

    UTIL_LCD_DrawLine(x0, y0, x1, y0, UTIL_LCD_COLOR_RED);
    UTIL_LCD_DrawLine(x1, y0, x1, y1, UTIL_LCD_COLOR_RED);
    UTIL_LCD_DrawLine(x1, y1, x0, y1, UTIL_LCD_COLOR_RED);
    UTIL_LCD_DrawLine(x0, y1, x0, y0, UTIL_LCD_COLOR_RED);

    UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_RED);
    UTIL_LCDEx_PrintfAt(x0, MAX(y0 - 20, 0), LEFT_MODE, "%s %.2f", label, det->score);
    UTIL_LCD_SetTextColor(UTIL_LCD_COLOR_WHITE);
}

static float app_box_iou(app_detection_t *a, app_detection_t *b)
{
    float x0 = MAX(a->x0, b->x0);
    float y0 = MAX(a->y0, b->y0);
    float x1 = MIN(a->x1, b->x1);
    float y1 = MIN(a->y1, b->y1);
    float inter_w = MAX(0.0f, x1 - x0);
    float inter_h = MAX(0.0f, y1 - y0);
    float inter = inter_w * inter_h;
    float area_a = MAX(0.0f, a->x1 - a->x0) * MAX(0.0f, a->y1 - a->y0);
    float area_b = MAX(0.0f, b->x1 - b->x0) * MAX(0.0f, b->y1 - b->y0);
    float denom = area_a + area_b - inter;

    return denom > 0.0f ? inter / denom : 0.0f;
}

static float app_clampf(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}
