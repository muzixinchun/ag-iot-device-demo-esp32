# 声网 ESP32 视频门铃

*简体中文| [English](README.en.md)*

## 例程简介

本例程演示了如何通过乐鑫 ESP32-S3 Korvo V3 开发板和一个安卓手机，模拟一个典型的视频门铃场景，可以演示按门铃键呼叫手机 APP 端，APP 端接听；或是手机 APP 实时查看门铃端的摄像头画面。

### 文件结构
```
├── CMakeLists.txt
├── components                                  Agora iot sdk component
│   ├── agora_iot_sdk
│   │   ├── CMakeLists.txt
│   │   ├── include                             Agora iot sdk header files
│   │   │   ├── agora_iot_api.h
│   │   │   └── agora_iot_call.h
│   │   └── lib
│   │       ├── esp32
│   │       │   └── PLACEHOLDER
│   │       ├── esp32s2
│   │       │   └── PLACEHOLDER
│   │       └── esp32s3                         Agora iot sdk libraries
│   │           ├── libagora-cjson.a
│   │           ├── libagora-iot-solution.a
│   │           ├── libagora-mbedtls.a
│   │           ├── libagora-webclient.a
│   │           ├── libahpl.a
│   │           ├── libhttp-parser.a
│   │           ├── libiot-license.a
│   │           ├── libiot-utility.a
│   │           ├── libmedia-engine.a
│   │           ├── librtsa.a
│   │           └── PLACEHOLDER
│   ├── camera                                  Camera component
│   ├── obstacle_avoidance                      HC-SR04 超声波避障组件
│   │   ├── CMakeLists.txt
│   │   ├── include
│   │   │   └── obstacle_avoidance.h
│   │   └── obstacle_avoidance.c
│   └── env_mapping                             占用栅格环境建模组件
│       ├── CMakeLists.txt
│       ├── include
│       │   └── env_mapping.h
│       └── env_mapping.c
├── firmware
│   ├── ag_videodoorbell_esp32.bin
│   ├── bootloader.bin
│   └── partition-table.bin
├── main                                        Video doorbell Demo code
│   ├── app_config.h
│   ├── CMakeLists.txt
│   ├── Kconfig.projbuild
│   └── video_doorbell.c
├── partitions.csv                              partition table
├── README.en.md
├── README.md
├── sdkconfig.defaults
└── sdkconfig.defaults.esp32s3
```

## 环境配置

### 硬件要求

本例程目前仅支持`ESP32-S3-Korvo-2`开发板。

摄像头推荐采用OV5660，外接一个扬声器。摄像头需要具有自动对焦特性，否则二维码配网功能不可用，可以采用ESP32的蓝牙配网或者代码中固定wifi相关信息

## 编译和下载

### 默认 IDF 分支

乐鑫ESP物联网开发框架（ESP-IDF）。安装方法详见 官方文档(https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32/get-started/index.html#id8)。

本例程支持 IDF release/v[4.4] 及以后的分支，例程默认使用 IDF release/v[4.4] 分支（commit id: 00396a9d495514986fd4fbfbd51e60fc8e4960e3)

选择 IDF 分支的方法，如下所示：

```bash
cd $IDF_PATH
git checkout release/v4.4
git pull
git submodule update --init --recursive
```

乐鑫ESP音频应用开发框架（ESP-ADF）。安装方法详见乐鑫音频应用开发指南(https://docs.espressif.com/projects/esp-adf/zh_CN/latest/get-started/index.html)。

本例程支持 ADF master分支（commit id: 6a977ad9d252738ecf672b154450d450b5e01a70)


### 打上 IDF 补丁

本例程还需给 IDF 合入1个 patch， 合入命令如下：
cd $IDF_PATH
git apply $ADF_PATH/idf_patches/idf_v4.4_freertos.patch


### 编译固件

将本例程（agora-demo-for-esp32）目录拷贝至 ~/esp 目录下。请运行如下命令：
```bash
$ export ADF_PATH=~/esp/esp-adf
$ . $HOME/esp/esp-idf/export.sh
$ cd ~/esp/agora-demo-for-esp32
$ idf.py set-target esp32s3
$ idf.py menuconfig
$ idf.py build
```

### 下载固件

#### Linux 操作系统

请运行如下命令：
```bash
$ idf.py -p /dev/ttyUSB0 flash monitor
```
注意：遇到 /dev/ttyUSB0 权限问题，请执行 sudo usermod -aG dialout $USER

下载成功后，本例程会自动运行，待初始化完成后，可以看到串口打印："Agora: Press [REC] key to ring the doorbell ..."。此时设备处在 `低功耗保活状态`

#### Windows 操作系统
（TBD）

### 下载预编译的固件

您也可以跳过以上的步骤，直接下载本例程中预编译好的固件。

#### Linux 操作系统

请运行如下命令：

```bash
esptool.py --chip esp32s3 \
--port /dev/ttyUSB0 --baud 921600 \
--before default_reset \
--after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size detect \
0x0      ./firmware/bootloader.bin \
0x8000   ./firmware/partition-table.bin \
0x10000  ./firmware/ag_videodoorbell_esp32.bin
```
#### Windows 操作系统
（TBD）

## 如何使用例程

### 开通灵隼物联网服务并创建产品

详见《开通声网灵隼服务》。

### 获取 License

设备端 SDK 通过 License 对设备鉴权。License 与设备绑定，一个 License 在同一时间只能绑定一个设备。

声网为每位开发者发放 10 个有效期 6 个月的免费测试 License，你需要联系 iot@agora.io 申请免费 License，或直接购买商业 License。

### 快速构建自定义ESP32门铃Demo

你可以简单修改ESP32 S3 Turnkey Demo相关参数配置，使用自己创建产品参数配套Demo效果。

设备端Demo源码中，你只需要修改main/app_config.h中的以下参数即可：

#define CONFIG_AGORA_APP_ID "4b31f****************************3037"

#define CONFIG_CUSTOMER_KEY "8620f******************************7363"

#define CONFIG_CUSTOMER_SECRET "492c1****************************e802"

#define CONFIG_MASTER_SERVER_URL "https://app.agoralink-iot-cn.sd-rtn.com"

#define CONFIG_SLAVE_SERVER_URL "https://api.agora.io/agoralink/cn/api"

#define CONFIG_PRODUCT_KEY "EJIJ**********5lI4"

#define CONFIG_LICENSE_PID "00F8******************************D646"  /* 从市场销售处申请获取 */

以上参数你均可从灵隼管理平台应用配置>>开发者选项页面中获取。

参数修改完毕后，参考编译和烧录步骤构建自己的ESP32 Turnkey Demo，对应的APP端Demo开发方式请参考对应的Android端文档和IOS端文档。

### 添加设备

打开安装的自定义应用，进入注册/登录界面，输入账号和密码 -> 点击 `登录`。

点击‘添加设备’，进入扫码设备二维码界面。

扫码设备的二维码后，进入wifi用户名和密码输入界面。

输入wifi用户名和密码后点击完成，手机屏幕出现二维码。

设备对准手机的二维码进行扫码。

扫码成功，手机app中点击完成，即设备添加成功。

### Demo 1：门铃呼叫APP，并进行视频通话

打开安装的自定义应用，进入注册/登录界面，输入账号和密码 -> 点击 `登录`。

回到开发板（门铃端），再次确认上述的固件编译、下载步骤已经完成，并看到串口每隔几秒打印出："Agora: Press [REC] key to ring the doorbell ..."

按一下 `[REC]` 按键，对 APP对应的用户进行呼叫。

APP 会出现被呼界面，显示 `有人按门铃`，并等待接听。同时已经可以看到门铃端的实时画面，听到门铃端的声音，但此时无法和门铃端对话。

在 APP端点击 `接听`，随即就能和门铃端实时对讲了。

在 APP端点击 `挂断`，结束和门铃端的通话。

说明：在按 [REC] 键呼叫远端APP前，门铃处于低功耗保活状态，典型功耗在 800uA 以下。呼叫后门铃自动切换到全功耗模式

### Demo 2：APP远程唤醒门铃，并进行视频通话

打开安装的自定义应用，进入注册/登录界面，输入账号和密码 -> 点击 `登录`。

回到开发板（门铃端），再次确认上述的固件编译、下载步骤已经完成，并看到串口每隔几秒打印出："Agora: Press [REC] key to ring the doorbell ..."

在我的设备中看到当前账号添加的设备，点击待通话的设备。

随即就能看到门铃端的实时画面了，APP端出图时间通常在 1 秒左右。

点击APP的电话按钮，就可以和门铃端实时对讲。

## 两轮平衡车：避障与环境建模

### 一个 ESP32 是否足够？

| 功能 | 单个 ESP32 | 备注 |
|------|-----------|------|
| 基础避障（阈值判断） | ✅ 足够 | HC-SR04 超声波传感器 + GPIO |
| 多方向避障（≤4 个传感器） | ✅ 足够 | 本仓库 `obstacle_avoidance` 组件已实现 |
| 局部占用栅格地图 | ⚠️ 有限 | 仅支持小尺寸网格（64×64），本仓库 `env_mapping` 组件已实现 |
| 完整 SLAM 环境建模 | ❌ 不足 | ESP32 双核被 Wi-Fi/蓝牙栈和电机 PID 占用，剩余算力不足以运行 SLAM |

**结论**：单个 ESP32 能完成基础避障，但无法独立完成全局环境建模（SLAM）。

### 完整避障与环境建模所需的额外硬件

| 硬件 | 型号示例 | 接口 | 作用 |
|------|---------|------|------|
| 超声波传感器（避障） | HC-SR04 × 1~4 | GPIO | 前方/侧方障碍物距离检测，最远约 400 cm |
| 2-D 激光雷达（建图） | RPLIDAR A1 / A2 | UART | 360° 高密度点云（~8 000 点/秒），用于全局地图构建 |
| IMU（姿态与里程计） | MPU-6050 / MPU-9250 | I2C | 加速度计 + 陀螺仪，辅助平衡控制和航向估计 |
| 电机编码器（里程计） | 霍尔编码器或光电编码器 | GPIO/PCNT | 精确轮速与位移测量，是 SLAM 里程计的基础 |
| 协处理器（SLAM 计算） | Raspberry Pi Zero 2 W 或第二块 ESP32-S3（带 PSRAM） | UART / SPI | 承担 Cartographer / Hector SLAM 计算，将结果回传主控 ESP32 |

### 推荐系统架构

```
┌─────────────────────────────────────────────┐
│  主控 ESP32 (本仓库)                         │
│  • 平衡 PID 控制 (MPU-6050 + 电机驱动)       │
│  • 基础避障 (obstacle_avoidance 组件)         │
│  • 局部占用栅格 (env_mapping 组件)            │
│  • Wi-Fi / Agora IoT 数据上报               │
└──────────────┬──────────────────────────────┘
               │ UART (激光雷达数据 + 里程计)
               ▼
┌─────────────────────────────────────────────┐
│  协处理器 (Raspberry Pi / 第二块 ESP32-S3)   │
│  • RPLIDAR A1 驱动                          │
│  • SLAM 算法 (Hector SLAM / Cartographer)   │
│  • 全局地图维护与路径规划                     │
└─────────────────────────────────────────────┘
```

### 快速使用 obstacle_avoidance 组件

```c
#include "obstacle_avoidance.h"

static void on_obstacle(obstacle_dir_t dir, uint32_t dist_cm, void *ctx)
{
    printf("障碍物！距离 %lu cm，建议方向: %d\n", (unsigned long)dist_cm, dir);
}

void app_main(void)
{
    obstacle_avoidance_cfg_t cfg = {
        .sensors = {
            { .trig_pin = GPIO_NUM_5, .echo_pin = GPIO_NUM_18, .position = SENSOR_POS_FRONT },
            { .trig_pin = GPIO_NUM_19, .echo_pin = GPIO_NUM_21, .position = SENSOR_POS_FRONT_LEFT },
        },
        .sensor_count     = 2,
        .safe_distance_cm = 30,
        .on_obstacle      = on_obstacle,
        .cb_ctx           = NULL,
    };
    obstacle_handle_t h = obstacle_avoidance_init(&cfg);
    // 模块在后台任务中每 100 ms 自动触发测量并调用回调
}
```

### 快速使用 env_mapping 组件

```c
#include "env_mapping.h"

void update_map_from_sensor(env_map_handle_t map, float range_cm)
{
    // 传感器安装在正前方（相对航向 0 rad）
    env_mapping_insert_range(map, 0.0f, range_cm, 400.0f);
}

void app_main(void)
{
    env_map_handle_t map = env_mapping_create();

    // 更新机器人位姿（由里程计提供）
    env_pose_t pose = { .x_cm = 10.0f, .y_cm = 0.0f, .heading = 0.0f };
    env_mapping_set_pose(map, &pose);

    // 插入一次超声波测量值
    update_map_from_sensor(map, 80.0f);

    // 检查前方路径是否畅通
    if (!env_mapping_path_is_clear(map, 30.0f)) {
        printf("前方有障碍物！\n");
    }

    env_mapping_destroy(map);
}
```

### 组件目录结构

```
components/
├── obstacle_avoidance/         # HC-SR04 避障模块
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── obstacle_avoidance.h
│   └── obstacle_avoidance.c
└── env_mapping/                # 占用栅格环境建模模块
    ├── CMakeLists.txt
    ├── include/
    │   └── env_mapping.h
    └── env_mapping.c
```

## 关于声网

声网音视频物联网平台方案，依托声网自建的底层实时传输网络 Agora SD-RTN™ (Software Defined Real-time Network)，为所有支持网络功能的 Linux/RTOS 设备提供音视频码流在互联网实时传输的能力。该方案充分利用了声网全球全网节点和智能动态路由算法，与此同时支持了前向纠错、智能重传、带宽预测、码流平滑等多种组合抗弱网的策略，可以在设备所处的各种不确定网络环境下，仍然交付高连通、高实时和高稳定的最佳音视频网络体验。此外，该方案具有极小的包体积和内存占用，适合运行在任何资源受限的 IoT 设备上，包括乐鑫 ESP32 全系列产品。

## 技术支持

请按照下面的链接获取技术支持：

- 如果发现了示例代码的 bug，欢迎提交 [issue](https://github.com/AgoraIO-Community/AG-VideoDoorbell-esp32/issues)
- 如果有其他疑问，也可以直接联系门铃方案负责人（微信：JJ2dog）

我们会尽快回复。

