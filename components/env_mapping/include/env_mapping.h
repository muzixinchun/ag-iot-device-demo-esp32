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
 * @file env_mapping.h
 * @brief Lightweight 2-D occupancy-grid environment mapping for ESP32.
 *
 * Overview
 * --------
 * This module maintains a fixed-size binary occupancy grid in SRAM.  Each
 * cell stores whether that region of the floor is believed to be occupied
 * (obstacle present) or free.
 *
 * The robot's position is tracked as a floating-point (x, y, θ) pose in a
 * right-handed coordinate frame where x points forward and y points left.
 *
 * Sensor readings (range + heading) are inserted with env_mapping_insert_range().
 * The endpoint of the ray is marked occupied; all cells along the ray are
 * marked free (ray-casting).
 *
 * Why a single ESP32 is not sufficient for full SLAM
 * ---------------------------------------------------
 * This component provides a *local* occupancy grid suitable for short-range
 * obstacle avoidance.  Full SLAM (Simultaneous Localisation and Mapping) that
 * builds a globally-consistent map requires:
 *
 *   1. A dense range sensor – typically an RPLIDAR A1/A2 (2-D, UART, ~8 kpts/s)
 *      or a depth camera.  A single HC-SR04 only provides one range reading
 *      per measurement cycle and cannot build a meaningful 2-D map alone.
 *
 *   2. Odometry from wheel encoders or an IMU (e.g. MPU-6050) for pose
 *      propagation between sensor updates.
 *
 *   3. A scan-matching or particle-filter step that is computationally
 *      expensive.  On an ESP32-S3 running the balance-control PID loop
 *      and the Wi-Fi stack simultaneously, there is insufficient CPU budget
 *      for real-time SLAM.
 *
 * Recommended additional hardware
 * --------------------------------
 *   - RPLIDAR A1 or A2 – 360° 2-D LiDAR, connected to ESP32 UART
 *   - MPU-6050 or MPU-9250 – 6/9-axis IMU, I2C
 *   - Motor encoders – quadrature encoders on both drive wheels
 *   - Co-processor for SLAM (optional but recommended):
 *       * Raspberry Pi Zero 2 W  (runs Cartographer or Hector SLAM under ROS2)
 *       * Second ESP32-S3 with PSRAM (runs a custom lightweight grid SLAM)
 *     The co-processor receives range + odometry data from the primary ESP32
 *     over UART / SPI and sends back the updated pose and map.
 *
 * This component is deliberately minimal so that it fits in the ESP32's
 * internal SRAM with a 64×64 grid (4 096 bytes of map data).
 */

#ifndef __ENV_MAPPING_H__
#define __ENV_MAPPING_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Grid dimensions                                                             */
/* -------------------------------------------------------------------------- */

/** Width of the occupancy grid in cells.  Must be a power of two ≤ 128. */
#define ENV_MAP_WIDTH   64
/** Height of the occupancy grid in cells.  Must be a power of two ≤ 128. */
#define ENV_MAP_HEIGHT  64
/** Physical size of one cell in centimetres. */
#define ENV_MAP_CELL_CM 10

/* -------------------------------------------------------------------------- */
/* Cell occupancy values                                                       */
/* -------------------------------------------------------------------------- */
#define ENV_CELL_FREE     0   /*!< Cell is confirmed free        */
#define ENV_CELL_OCCUPIED 1   /*!< Cell is confirmed occupied    */
#define ENV_CELL_UNKNOWN  2   /*!< Cell has not been observed    */

/* -------------------------------------------------------------------------- */
/* Types                                                                       */
/* -------------------------------------------------------------------------- */

/**
 * @brief 2-D robot pose in the map frame.
 */
typedef struct {
    float x_cm;      /*!< X position in cm (positive = forward)      */
    float y_cm;      /*!< Y position in cm (positive = left)          */
    float heading;   /*!< Heading in radians, 0 = facing +X           */
} env_pose_t;

/**
 * @brief Opaque environment map handle.
 */
typedef void *env_map_handle_t;

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

/**
 * @brief Allocate and initialise an occupancy grid.
 *
 * All cells start as ENV_CELL_UNKNOWN.  The robot origin is placed at the
 * centre of the grid.
 *
 * @return Handle on success, NULL when out of memory.
 */
env_map_handle_t env_mapping_create(void);

/**
 * @brief Reset all cells to ENV_CELL_UNKNOWN and move robot to centre.
 *
 * @param handle Handle from env_mapping_create().
 */
void env_mapping_reset(env_map_handle_t handle);

/**
 * @brief Update the robot's pose.
 *
 * Call this whenever new odometry data arrives (encoder ticks + IMU heading).
 *
 * @param handle  Handle from env_mapping_create().
 * @param pose    New robot pose.
 */
void env_mapping_set_pose(env_map_handle_t handle, const env_pose_t *pose);

/**
 * @brief Get the current robot pose.
 *
 * @param handle  Handle from env_mapping_create().
 * @param pose    Output pose.
 */
void env_mapping_get_pose(env_map_handle_t handle, env_pose_t *pose);

/**
 * @brief Insert a range measurement into the map.
 *
 * Performs a simple ray-cast from the current robot pose in the direction
 * (pose.heading + sensor_angle_rad).  Cells along the ray are marked free;
 * the endpoint cell is marked occupied (if range_cm < max_range_cm).
 *
 * @param handle           Handle from env_mapping_create().
 * @param sensor_angle_rad Angle of the sensor beam relative to robot heading (rad).
 * @param range_cm         Measured distance in centimetres.
 * @param max_range_cm     Maximum reliable sensor range; readings at or above
 *                         this value are treated as "no obstacle detected".
 */
void env_mapping_insert_range(env_map_handle_t handle,
                              float sensor_angle_rad,
                              float range_cm,
                              float max_range_cm);

/**
 * @brief Read the occupancy value of a specific cell.
 *
 * @param handle  Handle from env_mapping_create().
 * @param col     Column index [0, ENV_MAP_WIDTH).
 * @param row     Row index    [0, ENV_MAP_HEIGHT).
 * @return        ENV_CELL_FREE, ENV_CELL_OCCUPIED, or ENV_CELL_UNKNOWN.
 */
uint8_t env_mapping_get_cell(env_map_handle_t handle, uint8_t col, uint8_t row);

/**
 * @brief Check whether the straight-line path ahead of the robot is free.
 *
 * Traces a ray of @p look_ahead_cm centimetres along the current robot
 * heading and returns false if any occupied cell is encountered.
 *
 * @param handle         Handle from env_mapping_create().
 * @param look_ahead_cm  How far ahead to check (cm).
 * @return true if the path is clear, false if an obstacle is in the way.
 */
bool env_mapping_path_is_clear(env_map_handle_t handle, float look_ahead_cm);

/**
 * @brief Serialise the map to a compact binary buffer for transmission.
 *
 * Each cell is packed as 2 bits so the full 64×64 grid fits in 1 024 bytes.
 * The caller must provide a buffer of at least
 * (ENV_MAP_WIDTH * ENV_MAP_HEIGHT / 4) bytes.
 *
 * @param handle  Handle from env_mapping_create().
 * @param buf     Output buffer.
 * @param buf_len Length of @p buf in bytes.
 * @return        Number of bytes written, or -1 on error.
 */
int env_mapping_serialize(env_map_handle_t handle, uint8_t *buf, int buf_len);

/**
 * @brief Free all resources associated with the map.
 *
 * @param handle  Handle from env_mapping_create().
 */
void env_mapping_destroy(env_map_handle_t handle);

#ifdef __cplusplus
}
#endif

#endif /* __ENV_MAPPING_H__ */
