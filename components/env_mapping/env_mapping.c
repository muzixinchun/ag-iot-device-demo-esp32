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

#include "env_mapping.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TAG "env_map"

/* -------------------------------------------------------------------------- */
/* Internal context                                                            */
/* -------------------------------------------------------------------------- */

typedef struct {
    uint8_t           grid[ENV_MAP_HEIGHT][ENV_MAP_WIDTH]; /* occupancy values */
    env_pose_t        pose;
    SemaphoreHandle_t lock;
} env_ctx_t;

/* Robot starts at the centre cell */
#define ORIGIN_COL  (ENV_MAP_WIDTH  / 2)
#define ORIGIN_ROW  (ENV_MAP_HEIGHT / 2)

/* -------------------------------------------------------------------------- */
/* Coordinate conversion helpers                                               */
/* -------------------------------------------------------------------------- */

/* Convert world (cm) to grid column/row.  Returns false if out of bounds. */
static bool world_to_cell(float x_cm, float y_cm,
                           int *col_out, int *row_out)
{
    /* Map centre corresponds to world origin */
    int col = ORIGIN_COL + (int)(x_cm  / ENV_MAP_CELL_CM);
    int row = ORIGIN_ROW - (int)(y_cm  / ENV_MAP_CELL_CM); /* row 0 = top */

    if (col < 0 || col >= ENV_MAP_WIDTH || row < 0 || row >= ENV_MAP_HEIGHT) {
        return false;
    }
    *col_out = col;
    *row_out = row;
    return true;
}

/* -------------------------------------------------------------------------- */
/* Bresenham ray-cast                                                          */
/* -------------------------------------------------------------------------- */

/**
 * @brief Walk cells from (c0,r0) toward (c1,r1) marking them free, then mark
 *        (c1,r1) as occupied.  If endpoint is out of bounds or range exceeds
 *        max, only mark the traversed cells free (no endpoint).
 */
static void raycast(env_ctx_t *ctx,
                    int c0, int r0, int c1, int r1,
                    bool mark_endpoint)
{
    int dc = abs(c1 - c0);
    int dr = abs(r1 - r0);
    int sc = (c0 < c1) ? 1 : -1;
    int sr = (r0 < r1) ? 1 : -1;
    int err = dc - dr;

    int c = c0;
    int r = r0;

    while (1) {
        if (c < 0 || c >= ENV_MAP_WIDTH || r < 0 || r >= ENV_MAP_HEIGHT) {
            break;
        }

        bool is_endpoint = (c == c1 && r == r1);

        if (is_endpoint) {
            if (mark_endpoint) {
                ctx->grid[r][c] = ENV_CELL_OCCUPIED;
            }
            break;
        }

        /* Mark intermediate cells free */
        ctx->grid[r][c] = ENV_CELL_FREE;

        int e2 = 2 * err;
        if (e2 > -dr) { err -= dr; c += sc; }
        if (e2 <  dc) { err += dc; r += sr; }
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

env_map_handle_t env_mapping_create(void)
{
    env_ctx_t *ctx = (env_ctx_t *)calloc(1, sizeof(env_ctx_t));
    if (!ctx) {
        ESP_LOGE(TAG, "no memory for map");
        return NULL;
    }

    memset(ctx->grid, ENV_CELL_UNKNOWN, sizeof(ctx->grid));
    ctx->pose.x_cm    = 0.0f;
    ctx->pose.y_cm    = 0.0f;
    ctx->pose.heading = 0.0f;

    ctx->lock = xSemaphoreCreateMutex();
    if (!ctx->lock) {
        free(ctx);
        return NULL;
    }

    ESP_LOGI(TAG, "occupancy grid %dx%d created (%d cm/cell)",
             ENV_MAP_WIDTH, ENV_MAP_HEIGHT, ENV_MAP_CELL_CM);
    return (env_map_handle_t)ctx;
}

void env_mapping_reset(env_map_handle_t handle)
{
    env_ctx_t *ctx = (env_ctx_t *)handle;
    if (!ctx) return;

    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    memset(ctx->grid, ENV_CELL_UNKNOWN, sizeof(ctx->grid));
    ctx->pose.x_cm    = 0.0f;
    ctx->pose.y_cm    = 0.0f;
    ctx->pose.heading = 0.0f;
    xSemaphoreGive(ctx->lock);
}

void env_mapping_set_pose(env_map_handle_t handle, const env_pose_t *pose)
{
    env_ctx_t *ctx = (env_ctx_t *)handle;
    if (!ctx || !pose) return;

    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    ctx->pose = *pose;
    xSemaphoreGive(ctx->lock);
}

void env_mapping_get_pose(env_map_handle_t handle, env_pose_t *pose)
{
    env_ctx_t *ctx = (env_ctx_t *)handle;
    if (!ctx || !pose) return;

    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    *pose = ctx->pose;
    xSemaphoreGive(ctx->lock);
}

void env_mapping_insert_range(env_map_handle_t handle,
                              float sensor_angle_rad,
                              float range_cm,
                              float max_range_cm)
{
    env_ctx_t *ctx = (env_ctx_t *)handle;
    if (!ctx) return;

    xSemaphoreTake(ctx->lock, portMAX_DELAY);

    float total_angle = ctx->pose.heading + sensor_angle_rad;
    float ray_len     = (range_cm < max_range_cm) ? range_cm : max_range_cm;
    bool  hit         = (range_cm < max_range_cm);

    float end_x = ctx->pose.x_cm + ray_len * cosf(total_angle);
    float end_y = ctx->pose.y_cm + ray_len * sinf(total_angle);

    int c0, r0, c1, r1;
    if (!world_to_cell(ctx->pose.x_cm, ctx->pose.y_cm, &c0, &r0)) {
        /* Robot is outside the map */
        xSemaphoreGive(ctx->lock);
        return;
    }

    bool ep_in_bounds = world_to_cell(end_x, end_y, &c1, &r1);

    if (ep_in_bounds) {
        raycast(ctx, c0, r0, c1, r1, hit);
    } else {
        /* Ray exits the map – mark the portion inside the map as free */
        /* Clamp endpoint to map boundary along the ray direction */
        float step = (float)ENV_MAP_CELL_CM;
        float fx = ctx->pose.x_cm;
        float fy = ctx->pose.y_cm;
        float dx = cosf(total_angle) * step;
        float dy = sinf(total_angle) * step;
        float walked = 0.0f;
        while (walked < ray_len) {
            fx += dx; fy += dy; walked += step;
            int nc, nr;
            if (!world_to_cell(fx, fy, &nc, &nr)) break;
            ctx->grid[nr][nc] = ENV_CELL_FREE;
        }
    }

    xSemaphoreGive(ctx->lock);
}

uint8_t env_mapping_get_cell(env_map_handle_t handle, uint8_t col, uint8_t row)
{
    env_ctx_t *ctx = (env_ctx_t *)handle;
    if (!ctx || col >= ENV_MAP_WIDTH || row >= ENV_MAP_HEIGHT) {
        return ENV_CELL_UNKNOWN;
    }
    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    uint8_t val = ctx->grid[row][col];
    xSemaphoreGive(ctx->lock);
    return val;
}

bool env_mapping_path_is_clear(env_map_handle_t handle, float look_ahead_cm)
{
    env_ctx_t *ctx = (env_ctx_t *)handle;
    if (!ctx) return false;

    xSemaphoreTake(ctx->lock, portMAX_DELAY);

    float fx = ctx->pose.x_cm;
    float fy = ctx->pose.y_cm;
    float step = (float)ENV_MAP_CELL_CM;
    float dx = cosf(ctx->pose.heading) * step;
    float dy = sinf(ctx->pose.heading) * step;
    bool  clear = true;

    for (float d = 0.0f; d < look_ahead_cm; d += step) {
        int c, r;
        if (!world_to_cell(fx, fy, &c, &r)) {
            break; /* reached map boundary – treat as clear */
        }
        if (ctx->grid[r][c] == ENV_CELL_OCCUPIED) {
            clear = false;
            break;
        }
        fx += dx;
        fy += dy;
    }

    xSemaphoreGive(ctx->lock);
    return clear;
}

int env_mapping_serialize(env_map_handle_t handle, uint8_t *buf, int buf_len)
{
    env_ctx_t *ctx = (env_ctx_t *)handle;
    if (!ctx || !buf) return -1;

    /* 2 bits per cell, 4 cells per byte */
    int needed = (ENV_MAP_WIDTH * ENV_MAP_HEIGHT) / 4;
    if (buf_len < needed) {
        return -1;
    }

    xSemaphoreTake(ctx->lock, portMAX_DELAY);
    memset(buf, 0, needed);
    int idx = 0;
    for (int r = 0; r < ENV_MAP_HEIGHT; r++) {
        for (int c = 0; c < ENV_MAP_WIDTH; c++) {
            int bit_pos  = (idx % 4) * 2;
            buf[idx / 4] |= (ctx->grid[r][c] & 0x03) << bit_pos;
            idx++;
        }
    }
    xSemaphoreGive(ctx->lock);
    return needed;
}

void env_mapping_destroy(env_map_handle_t handle)
{
    env_ctx_t *ctx = (env_ctx_t *)handle;
    if (!ctx) return;

    if (ctx->lock) {
        vSemaphoreDelete(ctx->lock);
    }
    free(ctx);
}
