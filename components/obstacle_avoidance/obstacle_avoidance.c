/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2022 Agora Lab, Inc (http://www.agora.io/)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/**
 * @file obstacle_avoidance.c
 * @brief HC-SR04 ultrasonic obstacle avoidance for two-wheeled balance car.
 *
 * Implementation notes
 * --------------------
 * Each HC-SR04 sensor is triggered by a ≥10 µs HIGH pulse on TRIG.
 * The sensor then raises ECHO proportional to the time-of-flight.
 * Distance (cm) = echo_duration_µs / 58.
 *
 * Limitations of a single ESP32
 * ------------------------------
 * While a single ESP32 is adequate for threshold-based obstacle avoidance
 * with up to four ultrasonic sensors, it is **not** sufficient for full
 * environment modelling (SLAM) because:
 *   - The ESP32's dual cores are already occupied by the Wi-Fi/BT stack and
 *     motor-control loops.
 *   - SLAM algorithms (e.g. GMapping, Hector SLAM) require floating-point
 *     matrix operations that benefit from a Linux-class CPU.
 *   - Dense LiDAR data streams (e.g. 8,000 points/s from RPLIDAR A1) need
 *     faster processing than the ESP32 can sustain alongside its other tasks.
 *
 * For full environment modelling the recommended architecture is:
 *   ESP32  – real-time motor control + balance PID + Wi-Fi telemetry
 *   + RPLIDAR A1 (UART) for 360° range data
 *   + MPU-6050 / MPU-9250 (I2C) for IMU fusion
 *   + Motor encoders for odometry
 *   + Co-processor (Raspberry Pi or second ESP32-S3 with PSRAM) running
 *     a lightweight SLAM implementation (e.g. Cartographer Lite or
 *     a custom occupancy grid updated by this env_mapping component).
 */

#include "obstacle_avoidance.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"

#define TAG "obstacle"

/** Background task stack size (bytes). */
#define OA_TASK_STACK   2048
/** Background task priority. */
#define OA_TASK_PRIO    5
/** Measurement interval in milliseconds. */
#define OA_MEASURE_MS   100
/** Maximum echo wait time in microseconds (≈ 400 cm × 58 µs/cm). */
#define OA_ECHO_TIMEOUT_US  23200

/* -------------------------------------------------------------------------- */
/* Internal context                                                            */
/* -------------------------------------------------------------------------- */

typedef struct {
    obstacle_avoidance_cfg_t cfg;
    uint32_t                 distances_cm[OBSTACLE_SENSOR_MAX];
    bool                     obstacle_active;
    SemaphoreHandle_t        lock;
    TaskHandle_t             task;
} oa_ctx_t;

/* -------------------------------------------------------------------------- */
/* HC-SR04 low-level helpers                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Fire one measurement on a single HC-SR04 and return distance in cm.
 *
 * The function blocks for at most OA_ECHO_TIMEOUT_US microseconds.
 * Returns OBSTACLE_OUT_OF_RANGE when no echo is received in time.
 */
static uint32_t hcsr04_measure_cm(gpio_num_t trig, gpio_num_t echo)
{
    /* Send 10 µs trigger pulse */
    gpio_set_level(trig, 0);
    esp_rom_delay_us(2);
    gpio_set_level(trig, 1);
    esp_rom_delay_us(10);
    gpio_set_level(trig, 0);

    /* Wait for ECHO to go HIGH (measurement start) */
    int64_t t_start = esp_timer_get_time();
    while (gpio_get_level(echo) == 0) {
        if ((esp_timer_get_time() - t_start) > OA_ECHO_TIMEOUT_US) {
            return OBSTACLE_OUT_OF_RANGE;
        }
    }

    /* Measure HIGH duration */
    int64_t t_echo_start = esp_timer_get_time();
    while (gpio_get_level(echo) == 1) {
        if ((esp_timer_get_time() - t_echo_start) > OA_ECHO_TIMEOUT_US) {
            return OBSTACLE_OUT_OF_RANGE;
        }
    }
    int64_t echo_pulse_us = esp_timer_get_time() - t_echo_start;

    /* Distance in cm = echo_pulse_us / 58 */
    uint32_t dist_cm = (uint32_t)(echo_pulse_us / 58);
    if (dist_cm > OBSTACLE_OUT_OF_RANGE) {
        dist_cm = OBSTACLE_OUT_OF_RANGE;
    }
    return dist_cm;
}

/* -------------------------------------------------------------------------- */
/* Avoidance logic                                                             */
/* -------------------------------------------------------------------------- */

/**
 * @brief Given the current distance array, decide the best avoidance direction.
 */
static obstacle_dir_t compute_direction(const oa_ctx_t *ctx)
{
    bool front_blocked       = false;
    bool front_left_blocked  = false;
    bool front_right_blocked = false;

    for (uint8_t i = 0; i < ctx->cfg.sensor_count; i++) {
        uint32_t d = ctx->distances_cm[i];
        if (d >= ctx->cfg.safe_distance_cm) {
            continue;
        }
        switch (ctx->cfg.sensors[i].position) {
            case SENSOR_POS_FRONT:       front_blocked       = true; break;
            case SENSOR_POS_FRONT_LEFT:  front_left_blocked  = true; break;
            case SENSOR_POS_FRONT_RIGHT: front_right_blocked = true; break;
            default: break;
        }
    }

    if (!front_blocked && !front_left_blocked && !front_right_blocked) {
        return OBSTACLE_DIR_NONE;
    }
    if (front_blocked || (front_left_blocked && front_right_blocked)) {
        return OBSTACLE_DIR_BACK;
    }
    if (front_left_blocked) {
        return OBSTACLE_DIR_RIGHT;
    }
    if (front_right_blocked) {
        return OBSTACLE_DIR_LEFT;
    }
    return OBSTACLE_DIR_STOP;
}

/* -------------------------------------------------------------------------- */
/* Background measurement task                                                 */
/* -------------------------------------------------------------------------- */

static void oa_task(void *arg)
{
    oa_ctx_t *ctx = (oa_ctx_t *)arg;

    while (1) {
        uint32_t distances[OBSTACLE_SENSOR_MAX] = {0};

        for (uint8_t i = 0; i < ctx->cfg.sensor_count; i++) {
            distances[i] = hcsr04_measure_cm(ctx->cfg.sensors[i].trig_pin,
                                             ctx->cfg.sensors[i].echo_pin);
            /* Small inter-sensor delay prevents acoustic crosstalk */
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        xSemaphoreTake(ctx->lock, portMAX_DELAY);
        memcpy(ctx->distances_cm, distances, sizeof(uint32_t) * ctx->cfg.sensor_count);

        obstacle_dir_t dir = compute_direction(ctx);
        bool now_blocked = (dir != OBSTACLE_DIR_NONE);

        if (ctx->cfg.on_obstacle && (now_blocked != ctx->obstacle_active)) {
            /* Determine the closest triggering sensor distance for the callback */
            uint32_t closest = OBSTACLE_OUT_OF_RANGE;
            for (uint8_t i = 0; i < ctx->cfg.sensor_count; i++) {
                if (distances[i] < closest) {
                    closest = distances[i];
                }
            }
            ctx->cfg.on_obstacle(dir, closest, ctx->cfg.cb_ctx);
        }
        ctx->obstacle_active = now_blocked;
        xSemaphoreGive(ctx->lock);

        vTaskDelay(pdMS_TO_TICKS(OA_MEASURE_MS));
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

obstacle_handle_t obstacle_avoidance_init(const obstacle_avoidance_cfg_t *cfg)
{
    if (!cfg || cfg->sensor_count == 0 || cfg->sensor_count > OBSTACLE_SENSOR_MAX) {
        ESP_LOGE(TAG, "invalid config");
        return NULL;
    }

    oa_ctx_t *ctx = (oa_ctx_t *)calloc(1, sizeof(oa_ctx_t));
    if (!ctx) {
        ESP_LOGE(TAG, "no memory");
        return NULL;
    }
    memcpy(&ctx->cfg, cfg, sizeof(obstacle_avoidance_cfg_t));

    /* Apply safe-distance default */
    if (ctx->cfg.safe_distance_cm == 0) {
        ctx->cfg.safe_distance_cm = OBSTACLE_SAFE_DIST_CM;
    }

    /* Initialise distances to "clear" */
    for (uint8_t i = 0; i < OBSTACLE_SENSOR_MAX; i++) {
        ctx->distances_cm[i] = OBSTACLE_OUT_OF_RANGE;
    }

    /* Configure GPIO pins */
    for (uint8_t i = 0; i < cfg->sensor_count; i++) {
        gpio_config_t trig_conf = {
            .pin_bit_mask = (1ULL << cfg->sensors[i].trig_pin),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        gpio_config_t echo_conf = {
            .pin_bit_mask = (1ULL << cfg->sensors[i].echo_pin),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&trig_conf));
        ESP_ERROR_CHECK(gpio_config(&echo_conf));
        gpio_set_level(cfg->sensors[i].trig_pin, 0);
    }

    ctx->lock = xSemaphoreCreateMutex();
    if (!ctx->lock) {
        free(ctx);
        return NULL;
    }

    BaseType_t rc = xTaskCreate(oa_task, "oa_task", OA_TASK_STACK,
                                ctx, OA_TASK_PRIO, &ctx->task);
    if (rc != pdPASS) {
        vSemaphoreDelete(ctx->lock);
        free(ctx);
        return NULL;
    }

    ESP_LOGI(TAG, "obstacle avoidance ready (%d sensor(s), safe_dist=%lu cm)",
             cfg->sensor_count, (unsigned long)ctx->cfg.safe_distance_cm);
    return (obstacle_handle_t)ctx;
}

int obstacle_avoidance_measure(obstacle_handle_t handle, uint32_t *distances_cm)
{
    oa_ctx_t *ctx = (oa_ctx_t *)handle;
    if (!ctx || !distances_cm) {
        return -1;
    }
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    memcpy(distances_cm, ctx->distances_cm,
           sizeof(uint32_t) * ctx->cfg.sensor_count);
    xSemaphoreGive(ctx->lock);
    return 0;
}

obstacle_dir_t obstacle_avoidance_get_direction(obstacle_handle_t handle)
{
    oa_ctx_t *ctx = (oa_ctx_t *)handle;
    if (!ctx) {
        return OBSTACLE_DIR_STOP;
    }
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    obstacle_dir_t dir = compute_direction(ctx);
    xSemaphoreGive(ctx->lock);
    return dir;
}

uint32_t obstacle_avoidance_get_distance(obstacle_handle_t handle, uint8_t sensor_idx)
{
    oa_ctx_t *ctx = (oa_ctx_t *)handle;
    if (!ctx || sensor_idx >= ctx->cfg.sensor_count) {
        return OBSTACLE_OUT_OF_RANGE;
    }
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    uint32_t d = ctx->distances_cm[sensor_idx];
    xSemaphoreGive(ctx->lock);
    return d;
}

void obstacle_avoidance_deinit(obstacle_handle_t handle)
{
    oa_ctx_t *ctx = (oa_ctx_t *)handle;
    if (!ctx) {
        return;
    }
    if (ctx->task) {
        vTaskDelete(ctx->task);
    }
    if (ctx->lock) {
        vSemaphoreDelete(ctx->lock);
    }
    free(ctx);
}
