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
 * @file obstacle_avoidance.h
 * @brief Obstacle avoidance component for a two-wheeled balance car using HC-SR04
 *        ultrasonic sensors.
 *
 * Hardware requirements
 * ---------------------
 * A single ESP32 can handle basic obstacle *detection* and *avoidance* using
 * one or more HC-SR04 ultrasonic sensors.  Each sensor uses two GPIO lines:
 *   - TRIG  – output, 10 µs pulse triggers a measurement
 *   - ECHO  – input,  pulse width proportional to distance
 *
 * For full 360° environment *modelling* (SLAM / occupancy grid) a single ESP32
 * is not sufficient on its own.  The recommended additional hardware is:
 *   1. RPLIDAR A1 (or equivalent 2-D LiDAR) connected via UART for dense range
 *      measurements.
 *   2. MPU-6050 IMU (I2C) already present on most balance-car boards, used for
 *      wheel-odometry fusion.
 *   3. Motor encoders for accurate odometry.
 *   4. (Optional) A second MCU / co-processor (e.g. a Raspberry Pi or a second
 *      ESP32) dedicated to the SLAM computation so that the primary ESP32 can
 *      focus on real-time motor control and communication.
 *
 * This component implements:
 *   - HC-SR04 driver (single sensor or up to OBSTACLE_SENSOR_MAX sensors)
 *   - Threshold-based obstacle avoidance logic with configurable safe distance
 *   - Callback notification when an obstacle is detected or cleared
 */

#ifndef __OBSTACLE_AVOIDANCE_H__
#define __OBSTACLE_AVOIDANCE_H__

#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Maximum number of ultrasonic sensors supported simultaneously. */
#define OBSTACLE_SENSOR_MAX     4

/** Default safe distance in centimetres (stop / turn when closer than this). */
#define OBSTACLE_SAFE_DIST_CM   30

/** Measurement out-of-range sentinel value (cm). */
#define OBSTACLE_OUT_OF_RANGE   400

/**
 * @brief Direction hint returned by the avoidance algorithm.
 */
typedef enum {
    OBSTACLE_DIR_NONE   = 0,  /*!< No obstacle, continue forward */
    OBSTACLE_DIR_LEFT   = 1,  /*!< Turn left to avoid */
    OBSTACLE_DIR_RIGHT  = 2,  /*!< Turn right to avoid */
    OBSTACLE_DIR_BACK   = 3,  /*!< Reverse – blocked on both sides */
    OBSTACLE_DIR_STOP   = 4,  /*!< Cannot determine safe direction */
} obstacle_dir_t;

/**
 * @brief Sensor placement hint used to select avoidance direction.
 */
typedef enum {
    SENSOR_POS_FRONT        = 0,
    SENSOR_POS_FRONT_LEFT   = 1,
    SENSOR_POS_FRONT_RIGHT  = 2,
    SENSOR_POS_REAR         = 3,
} sensor_position_t;

/**
 * @brief Per-sensor configuration.
 */
typedef struct {
    gpio_num_t      trig_pin;   /*!< GPIO connected to TRIG of HC-SR04 */
    gpio_num_t      echo_pin;   /*!< GPIO connected to ECHO of HC-SR04 */
    sensor_position_t position; /*!< Physical placement on the vehicle */
} obstacle_sensor_cfg_t;

/**
 * @brief Module configuration passed to obstacle_avoidance_init().
 */
typedef struct {
    obstacle_sensor_cfg_t sensors[OBSTACLE_SENSOR_MAX]; /*!< Sensor array */
    uint8_t               sensor_count;                 /*!< Number of active sensors */
    uint32_t              safe_distance_cm;             /*!< Obstacle threshold (cm) */
    /** Optional callback invoked from the measurement task when an obstacle
     *  is newly detected or cleared.  May be NULL.
     *  @param dir      Suggested avoidance direction (OBSTACLE_DIR_NONE when cleared).
     *  @param dist_cm  Distance measured by the triggering sensor (cm).
     *  @param ctx      User-supplied context pointer.
     */
    void (*on_obstacle)(obstacle_dir_t dir, uint32_t dist_cm, void *ctx);
    void *cb_ctx; /*!< Context pointer forwarded to on_obstacle. */
} obstacle_avoidance_cfg_t;

/** Opaque handle returned by obstacle_avoidance_init(). */
typedef void *obstacle_handle_t;

/**
 * @brief Initialise the obstacle avoidance module.
 *
 * Configures the GPIO pins for every sensor, installs the RMT / MCPWM
 * capture peripheral for echo timing, and starts a background FreeRTOS
 * task that periodically fires all sensors and evaluates distances.
 *
 * @param cfg   Pointer to a fully populated obstacle_avoidance_cfg_t.
 * @return      Handle on success, NULL on failure.
 */
obstacle_handle_t obstacle_avoidance_init(const obstacle_avoidance_cfg_t *cfg);

/**
 * @brief Trigger a single synchronous measurement on all sensors.
 *
 * Safe to call from any task context.  Blocks for up to ~25 ms (one round-
 * trip at maximum range).  Results are written into @p distances_cm which
 * must be at least cfg->sensor_count elements long.
 *
 * @param handle        Handle from obstacle_avoidance_init().
 * @param distances_cm  Output array; OBSTACLE_OUT_OF_RANGE when no echo received.
 * @return 0 on success, negative errno on failure.
 */
int obstacle_avoidance_measure(obstacle_handle_t handle, uint32_t *distances_cm);

/**
 * @brief Evaluate the current sensor readings and return a steering suggestion.
 *
 * @param handle  Handle from obstacle_avoidance_init().
 * @return        Avoidance direction, or OBSTACLE_DIR_NONE if path is clear.
 */
obstacle_dir_t obstacle_avoidance_get_direction(obstacle_handle_t handle);

/**
 * @brief Read the most recent distance from a specific sensor.
 *
 * @param handle      Handle from obstacle_avoidance_init().
 * @param sensor_idx  Zero-based index into the sensor array.
 * @return            Distance in cm, or OBSTACLE_OUT_OF_RANGE.
 */
uint32_t obstacle_avoidance_get_distance(obstacle_handle_t handle, uint8_t sensor_idx);

/**
 * @brief Deinitialise the module and free all resources.
 *
 * @param handle  Handle from obstacle_avoidance_init().  Set to NULL afterwards.
 */
void obstacle_avoidance_deinit(obstacle_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* __OBSTACLE_AVOIDANCE_H__ */
