# Agora Video Doorbell for ESP32

This page guides you on running the demo project of the ESP32-S3 solution.

> This free 90-day trial demo version is available for testing purposes. Contact sales@agora.io or add the Wechat account karstamu to learn how to purchase and activate a commercial license.

## Run the device-side sample project

### Set up the development environment

Make sure your development environment meets the following requirements.

#### Hardware envrionment

- ESP32-S3-Korvo-2 
- A connected camera and loudspeaker for video/audio communication
- Two A to Micro-B USB cables used for power supply and data connection
- PC running Windows, Linux, or macOS

#### Software envrionment

1. Clone the sample project.

    ```shell
    git clone git@github.com:AgoraIO-Community/AG-VideoDoorbell-esp32.git
    ```

2. Install the following software in your computer:

    - Espressif IoT Development Framework (ESP-IDF). See the official documentation for installation steps. After installation, select the v4.4 branch.
        
        ```shell
        cd $IDF_PATH
        git checkout release/v4.4
        git pull
        git submodule update --init --recursive
        ```
        
   - The FreeRTOS patch of ESP-IDF. Run the following commands to install the patch.
       
       ```shell
       cd $IDF_PATH
       git apply $ADF_PATH/idf_patches/idf_v4.4_freertos.patch
       ```
   - Espressif Systems Audio Development Framework (ESP-ADF). See the official documentation for installation steps.

### Run the sample project

The following steps take macOS or Linux as an example:

1. Copy the AG-VideoDoorbell-esp32 folder to the ~/esp folder. Run the following command to compile the project:

     ```shell
     cd ~/esp/agora-demo-for-esp32
     idf.py set-target esp32s3
     # Configure WiFi SSID 和 WiFi Password in the menuconfig interface
     idf.py menuconfig
     # Build the project
     idf.py build
     ```

2. Flash the firmware to the device:

     ```shell
     $ idf.py -p /dev/ttyUSB0 flash monitor
     ```

> If you encounter the /dev/ttyUSB0 permission issue, run sudo usermod -aG dialout $USER to get privileges.

After flashing the firmware, this demo runs automatically. When initialization is successful, you can see the following information from the device:

```text
Agora: Press [REC] key to ring the doorbell ...
```

The device is in low-power mode.

## Run the client-side sample project

### Set up the development environment

Make sure your development environment meets the following requirements.

#### Hardware envrionment

- Android device or simulator
- PC running Windows, Linux, or macOS
- USB cable compatible with the Android device (not necessary for simulators)

#### Software envrionment

1. Download the latest version of Android Studio.
2. Clone the sample project:
      
      ```shell
      git clone git@github.com:AgoraIO-Community/AG-VideoDoorbell-Android.git
      ```
      
### Run the sample project

1. Open the sample project with Android Studio, which automatically syncs the project with Gradle.
2. After project sync, connect the Android device to the computer. Build and run the project on the Android device.


## Two-Wheeled Balance Car: Obstacle Avoidance and Environment Modelling

### Is a single ESP32 sufficient?

| Capability | Single ESP32 | Notes |
|-----------|-------------|-------|
| Basic threshold-based obstacle detection | ✅ Yes | HC-SR04 ultrasonic sensor via GPIO |
| Multi-directional avoidance (up to 4 sensors) | ✅ Yes | Implemented in the `obstacle_avoidance` component |
| Local occupancy-grid mapping | ⚠️ Limited | Small grid only (64 × 64 cells); implemented in the `env_mapping` component |
| Full SLAM environment modelling | ❌ No | ESP32 dual cores are already consumed by the Wi-Fi/BT stack and motor-control PID; insufficient headroom for SLAM |

**Conclusion**: A single ESP32 can handle basic obstacle avoidance, but it is not sufficient to run full Simultaneous Localisation and Mapping (SLAM) on its own.

### Additional hardware required for full obstacle avoidance and environment modelling

| Hardware | Example model | Interface | Purpose |
|---------|--------------|-----------|---------|
| Ultrasonic sensor(s) | HC-SR04 × 1–4 | GPIO | Front / side obstacle distance, up to ~400 cm |
| 2-D LiDAR | RPLIDAR A1 / A2 | UART | 360° dense point cloud (~8 000 pts/s) for global mapping |
| IMU | MPU-6050 / MPU-9250 | I2C | Accelerometer + gyroscope for balance control and heading estimation |
| Wheel encoders | Hall-effect or optical | GPIO / PCNT | Accurate odometry for SLAM |
| Co-processor for SLAM | Raspberry Pi Zero 2 W or a second ESP32-S3 with PSRAM | UART / SPI | Runs Cartographer / Hector SLAM; sends updated pose and map back to the primary ESP32 |

### Recommended system architecture

```
┌─────────────────────────────────────────────┐
│  Primary ESP32 (this repository)            │
│  • Balance PID (MPU-6050 + motor driver)    │
│  • Obstacle avoidance (obstacle_avoidance)  │
│  • Local occupancy grid (env_mapping)       │
│  • Wi-Fi / Agora IoT telemetry              │
└──────────────┬──────────────────────────────┘
               │ UART (LiDAR data + odometry)
               ▼
┌─────────────────────────────────────────────┐
│  Co-processor (RPi / second ESP32-S3)       │
│  • RPLIDAR A1 driver                        │
│  • SLAM (Hector SLAM / Cartographer)        │
│  • Global map maintenance & path planning   │
└─────────────────────────────────────────────┘
```

### Quick start – obstacle_avoidance component

```c
#include "obstacle_avoidance.h"

static void on_obstacle(obstacle_dir_t dir, uint32_t dist_cm, void *ctx)
{
    printf("Obstacle! distance=%lu cm, suggested direction=%d\n",
           (unsigned long)dist_cm, dir);
}

void app_main(void)
{
    obstacle_avoidance_cfg_t cfg = {
        .sensors = {
            { .trig_pin = GPIO_NUM_5,  .echo_pin = GPIO_NUM_18, .position = SENSOR_POS_FRONT },
            { .trig_pin = GPIO_NUM_19, .echo_pin = GPIO_NUM_21, .position = SENSOR_POS_FRONT_LEFT },
        },
        .sensor_count     = 2,
        .safe_distance_cm = 30,
        .on_obstacle      = on_obstacle,
        .cb_ctx           = NULL,
    };
    obstacle_handle_t h = obstacle_avoidance_init(&cfg);
    // The module fires measurements every 100 ms in a background task
    // and calls on_obstacle whenever the obstacle state changes.
}
```

### Quick start – env_mapping component

```c
#include "env_mapping.h"

void app_main(void)
{
    env_map_handle_t map = env_mapping_create();

    // Update the robot pose from odometry
    env_pose_t pose = { .x_cm = 10.0f, .y_cm = 0.0f, .heading = 0.0f };
    env_mapping_set_pose(map, &pose);

    // Insert an ultrasonic reading (sensor faces forward, angle = 0 rad)
    env_mapping_insert_range(map, 0.0f, 80.0f, 400.0f);

    // Check whether the path ahead is clear
    if (!env_mapping_path_is_clear(map, 30.0f)) {
        printf("Obstacle ahead!\n");
    }

    env_mapping_destroy(map);
}
```

### Component directory structure

```
components/
├── obstacle_avoidance/         # HC-SR04 obstacle avoidance
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── obstacle_avoidance.h
│   └── obstacle_avoidance.c
└── env_mapping/                # Occupancy-grid environment modelling
    ├── CMakeLists.txt
    ├── include/
    │   └── env_mapping.h
    └── env_mapping.c
```

### Implement device-to-client communication 

Implement device-to-client communication with the following steps.

#### The device calls the client for a video chat

1. Open the **VideoDoorbell** app in the Android device.
2. Grant permission requests for microphone and local storage. Enter the **register/login** page.
3. Enter `jack` as the account, and click **Login** to enter the main interface.
4. On the ESP32-S3-Korvo-2 device, press [REC] to call user jack. The calling interface pops up in the **VideoDoorbell** app. You can receive the real-time video and audio from the device, but you cannot chat with the device.
5. Click **Answer** in the **VideoDoorbell** app to communicate with the device.
6. Click **Hang up** in the **VideoDoorbell** app to stop communicating with the client.

Before pressing [REC] to communicate with the client app, the doorbell is in low-power mode. The typical power consumption is below 800 μA. After calling, the device automatically switches to full-power mode.

#### The client wakes up the device for a video chat

Enter mydoorbell for the device account field. Click Call to see the real-time video from the device. You can also chat with the device.

