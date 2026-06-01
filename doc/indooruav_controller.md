# indooruav_controller

> 室内 UAV 硬件控制器 ROS 功能包（基于 DJI Payload SDK 低速数据通道）

将上游 ROS 节点的指令通过 PSDK ↔ MSDK 二进制帧协议转发到 DJI Mavic 3T，并将飞机端的执行完成事件回传给上游状态机。

---

## 一、概述

本包是 `indooruav_psdk_bridge` 的重构版本，核心职责保持不变，但架构由"两层管道"折叠为"单层服务"：

| 维度 | 旧包 `indooruav_psdk_bridge` | 新包 `indooruav_controller` |
|---|---|---|
| 节点数 | 1 个 | 1 个 |
| 内部话题中转 | `/drone/cmd`、`/drone/aux_light`、`/drone/gimbal/*`、`/drone/camera/*` 自发自收 | 全部删除 |
| 位置控制器 | 内置 PID（`PositionController`） | 移除，`/drone/cmd_vel` 由外部节点发布 |
| 上层接口 | 部分服务（仅 takeoff/land/灯/录像） | **全部 ROS 服务统一暴露** |
| 类组织 | `RosInterface` + `PositionController` + 散文件 | 单一 `ControllerHardware` 类 |
| 协议层 | `drone_comm_protocol.hpp` 独立头文件 | 内嵌于 `controller_hardware.hpp` 的 `drone_comm` 命名空间 |

整体数据通路：

```
上游节点 ──ROS 服务──▶ ControllerHardware ──二进制帧──▶ PSDK 低速通道 ──▶ MSDK App ──▶ DJI 飞机
上游节点 ◀──ROS 服务── ControllerHardware ◀──二进制帧── PSDK 低速通道 ◀── MSDK App ◀── DJI 飞机
其它节点 ──/drone/cmd_vel──▶ ControllerHardware（受门控）
```

---

## 二、文件结构

```text
indooruav_controller/
├── package.xml                                       # 包元数据（含 message_generation/runtime）
├── CMakeLists.txt                                    # 构建脚本（含 srv 生成、PSDK 静态库链接）
├── config/
│   └── config.yaml                                   # 服务名、话题名、cmd_vel 发送频率
├── launch/
│   └── bringup_controller_hardware.launch            # 启动入口
├── srv/                                              # 自定义服务（带连续参数）
│   ├── GimbalYawFollow.srv                           # 云台 Yaw Follow
│   ├── GimbalAngle.srv                               # 云台目标角度
│   ├── CameraVideoConfig.srv                         # 视频分辨率 + 帧率
│   └── CameraZoom.srv                                # 镜头切换 + 变焦倍数
├── include/indooruav_controller/
│   └── controller_hardware.hpp                       # 协议层 + 类声明（合并自原 drone_comm_protocol.hpp）
├── src/indooruav_controller/
│   └── controller_hardware.cpp                       # 类实现（合并自原 ros_interface + psdk_drone_controller）
└── node/
    └── controller_hardware_node.cpp                  # 极简 main：PSDK 初始化 + AsyncSpinner
```

---

## 三、ROS 接口

### 3.1 服务（入站，作为服务端）

按模块分组，共 **15 个**。

#### 飞控（`std_srvs/Empty` × 3）

| 服务名（默认） | 类型 | 行为 |
|---|---|---|
| `.../state_machine_action/takeoff` | `Empty` | 触发自动起飞，**关闭** cmd_vel 门 |
| `.../state_machine_action/land` | `Empty` | 触发自动降落，**关闭** cmd_vel 门 |
| `.../state_machine_action/hover` | `Empty` | 释放 VirtualStick 进入定点悬停，**关闭** cmd_vel 门 |

#### 补光灯（`std_srvs/Empty` × 3）

| 服务名（默认） | 类型 | 行为 |
|---|---|---|
| `.../notify_uav_open_light` | `Empty` | 下视补光灯打开 |
| `.../notify_uav_close_light` | `Empty` | 下视补光灯关闭 |
| `.../notify_uav_auto_light` | `Empty` | 下视补光灯自动 |

#### 相机（`std_srvs/Empty` × 5 + 自定义 × 2）

| 服务名（默认） | 类型 | 行为 |
|---|---|---|
| `.../camera/mode_photo` | `Empty` | 切换至照片模式 |
| `.../camera/mode_video` | `Empty` | 切换至视频模式 |
| `.../camera/shoot` | `Empty` | 单拍 |
| `.../notify_uav_video_recording_start` | `Empty` | 开始录像 |
| `.../notify_uav_video_recording_stop` | `Empty` | 停止录像 |
| `.../camera/video_config` | `CameraVideoConfig` | 配置分辨率 + 帧率 |
| `.../camera/zoom` | `CameraZoom` | 镜头切换（广角/变焦/红外）+ 变焦倍数 |

#### 云台（自定义 × 2）

| 服务名（默认） | 类型 | 行为 |
|---|---|---|
| `.../gimbal/yaw_follow` | `GimbalYawFollow` | Yaw Follow 模式下控制 pitch/roll |
| `.../gimbal/angle` | `GimbalAngle` | 绝对/相对角度模式控制 |

> 所有自定义服务的 response 均为空（与 `Empty` 风格统一）。语义为"发即忘"——服务回调仅在帧编码 + PSDK 发送后立即返回 true，**真实的执行结果通过 ACK 异步回报，不阻塞调用方**。

### 3.2 服务（出站，作为客户端）

| 服务名（默认） | 类型 | 触发时机 |
|---|---|---|
| `.../state_machine_event/takeoff_complete` | `Empty` | 收到 MSDK 的 `CMD_ACK_TAKEOFF_COMPLETE` |
| `.../state_machine_event/land_complete` | `Empty` | 收到 MSDK 的 `CMD_ACK_LAND_COMPLETE` |
| `.../state_machine_event/hover_complete` | `Empty` | 收到 MSDK 的 `CMD_ACK_HOVER_COMPLETE` |

### 3.3 话题（订阅）

| 话题（默认） | 类型 | 说明 |
|---|---|---|
| `/drone/cmd_vel` | `geometry_msgs/Twist` | 机体 FLU 系速度指令，受**门控**约束 |

---

## 四、协议帧格式

PSDK ↔ MSDK 通过 DJI 低速数据通道交换二进制帧（与 MSDK Kotlin 端共用同一套编解码）：

```
┌────────┬──────┬────────┬──────────────────┬──────────┐
│ 0xAA   │ CMD  │  LEN   │   PAYLOAD (N B)  │ XOR校验  │
│  1 B   │  1 B │  1 B   │      N B         │   1 B    │
└────────┴──────┴────────┴──────────────────┴──────────┘
```

固定开销 4 B，最大总帧长 = 4 + 251 = 255 B（MSDK 单包上限）。

### 指令分段

| 段 | 类别 | 指令 |
|---|---|---|
| `0x0x` | 飞控 | `CMD_TAKEOFF=0x01`, `CMD_LAND=0x02`, `CMD_HOVER=0x03`, `CMD_VEL=0x04` |
| `0x1x` | 云台 | `CMD_GIMBAL_YAW_FOLLOW=0x11`, `CMD_GIMBAL_ANGLE=0x12` |
| `0x2x` | 相机 | `CMD_CAM_MODE=0x21`, `CMD_CAM_SHOOT=0x22`, `CMD_CAM_RECORD=0x23`, `CMD_CAM_VIDEO_CFG=0x24`, `CMD_CAM_ZOOM=0x25` |
| `0x3x` | 配件 | `CMD_AUX_LIGHT=0x31` |
| `0x8x` | 应答 | `CMD_ACK=0x80`, `CMD_ACK_TAKEOFF_COMPLETE=0x81`, `CMD_ACK_LAND_COMPLETE=0x82`, `CMD_ACK_HOVER_COMPLETE=0x83` |

### 速度坐标系约定（关键）

| 维度 | ROS REP-103 (机体) | 协议 `VelPayload` (DJI 机体系) | 转换 |
|---|---|---|---|
| `x` | 前向 m/s（正=前） | `vx` 前向 m/s | 无 |
| `y` | **左向** m/s（正=左） | `vy` **右向** m/s | **取反** |
| `z` | 上向 m/s（正=上） | `vz` 上向 m/s | 无 |
| `yaw` | `angular.z` rad/s（正=逆时针） | `yaw_rate` deg/s（正=顺时针） | **取反并转 deg** |

---

## 五、门控状态机（核心安全机制）

`/drone/cmd_vel` 的转发受 `vel_forwarding_enabled_` 标志控制，避免起飞/降落/悬停过程中外部 cmd_vel 与 MSDK 自动动作抢 VirtualStick。

### 状态转移表

| 当前事件 | gate 变化 | 副作用 |
|---|---|---|
| 节点启动 | `false`（初始） | — |
| `takeoff` 服务被调用 | `→ false` | 清空 `vel_updated_`，发送 `CMD_TAKEOFF` |
| 收到 `CMD_ACK_TAKEOFF_COMPLETE` | `→ true` | 调用上游 `takeoff_complete` |
| `hover` 服务被调用 | `→ false` | 清空 `vel_updated_`，发送 `CMD_HOVER` |
| 收到 `CMD_ACK_HOVER_COMPLETE` | `→ true` | 调用上游 `hover_complete` |
| `land` 服务被调用 | `→ false` | 清空 `vel_updated_`，发送 `CMD_LAND` |
| 收到 `CMD_ACK_LAND_COMPLETE` | **保持 `false`** | 调用上游 `land_complete` |

### cmd_vel 处理策略（drop）

- 门**关闭**期间收到的 `cmd_vel`：**直接丢弃**，不缓存
- 门**打开**后必须等待**新的** `cmd_vel` 才会发送
- 一次 `cmd_vel` → 一次 VEL 帧（发送后立即清 `vel_updated_`）
- MSDK 端 1 秒 VEL 看门狗：若 ROS 端停止发送超过 1 s，MSDK 自动让飞机悬停

---

## 六、并发与线程安全

| 机制 | 用途 |
|---|---|
| `ros::AsyncSpinner(2)` | 服务回调与 Timer/订阅回调可并发 |
| `vel_mutex_` | 守护 `latest_vel_` / `vel_updated_` / `vel_forwarding_enabled_`（一把锁同管 3 项，避免门关闭时 stale vel 漏出） |
| `tx_mutex_` | 串行化所有 `DjiLowSpeedDataChannel_SendData` 调用 |
| `static instance_` 单例 | 解决 PSDK C 风格回调无 `user_data` 字段的限制 |
| `std::atomic<uint32_t> ack_count_` | ACK 计数（仅日志） |

### PSDK 接收线程

PSDK 自己的线程会调用 `StaticOnRecvFromMsdk(...)`，它仅读 `instance_` 指针并分发到实例方法。该线程会在收到完成事件时反向调用上游 ROS 服务（`takeoff_complete` 等），ROS 客户端线程安全。

---

## 七、配置参数

`config/config.yaml` 节选：

```yaml
indooruav_controller:
  services:
    takeoff: "indooruav_controller/controller_hardware/takeoff"
    # ...（共 15 个入站 + 3 个出站，详见服务接口章节）
  topics:
    cmd_vel: "/drone/cmd_vel"
  parameters:
    vel_send_rate_hz: 10.0   # cmd_vel 轮询频率
```

所有服务名、话题名都可在 yaml 中重映射，无需重新编译。

---

## 八、构建与运行

### 8.1 依赖

- ROS（catkin 工作空间）
- `geometry_msgs` / `std_msgs` / `std_srvs` / `roscpp`
- `message_generation` / `message_runtime`
- DJI Payload SDK 静态库（`psdk_lib/` 已随包提供）
- 架构支持：`x86_64` / `aarch64` / `armv7l`

### 8.2 编译

```bash
cd ~/catkin_ws
catkin_make --pkg indooruav_controller
source devel/setup.bash
```

### 8.3 启动

```bash
roslaunch indooruav_controller bringup_controller_hardware.launch
```

### 8.4 调试调用示例

```bash
# 起飞
rosservice call /indooruav_controller/controller_hardware/takeoff "{}"

# 切换至变焦镜头并设为 5x
rosservice call /indooruav_controller/controller_hardware/camera/zoom \
  "lens: 1
   action: 1
   ratio: 5.0"

# 云台俯仰 -30 度（绝对模式，2 秒到位）
rosservice call /indooruav_controller/controller_hardware/gimbal/angle \
  "mode: 0
   pitch: -30.0
   roll: 0.0
   yaw: 0.0
   duration: 2.0"

# 发布速度指令（需先收到 takeoff_complete）
rostopic pub -1 /drone/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {z: 0.0}}'
```

---

## 九、典型工作流

### 9.1 起飞 → 飞行 → 降落

```
[t=0]    上游 → takeoff 服务  ──▶ ControllerHardware 关门 + 发送 CMD_TAKEOFF
[t=Δ1]   MSDK App 控制飞机自动起飞至悬停高度
[t=Δ2]   MSDK → CMD_ACK_TAKEOFF_COMPLETE  ──▶ ControllerHardware 开门
[t=Δ2+]  ControllerHardware → 上游 takeoff_complete 服务
[t=Δ3]   上游开始发布 /drone/cmd_vel  ──▶ 转发为 CMD_VEL 帧（10 Hz 轮询）
[t=Δ4]   上游 → land 服务  ──▶ ControllerHardware 关门 + 发送 CMD_LAND
[t=Δ5]   MSDK 自动降落落地
[t=Δ6]   MSDK → CMD_ACK_LAND_COMPLETE  ──▶ ControllerHardware 通知上游 land_complete
```

### 9.2 关键不变量

- `takeoff_complete` 之前永远不会有 VEL 帧发往 MSDK
- `land` 之后永远不会有 VEL 帧发往 MSDK（直到下次 `takeoff_complete`）
- 所有 PSDK 写操作互斥串行
- 服务回调"发即忘"，不会因等待 ACK 而阻塞上游

---

## 十、维护要点

### 10.1 添加新指令的步骤

1. 在 `controller_hardware.hpp` 的 `drone_comm` 命名空间内添加 `CMD_*` 常量、`#pragma pack` 载荷结构体、`encode_*` 内联函数
2. 在 `srv/` 添加 `.srv` 文件（若需带参）；或直接复用 `std_srvs/Empty`
3. 在 `CMakeLists.txt` 的 `add_service_files` 注册新 srv
4. 在 `controller_hardware.hpp` 类内添加 `*_Callback` 方法声明
5. 在 `controller_hardware.cpp` 实现回调，并在 `LoadParameters` / `AdvertiseServiceServers` 注册
6. 在 `config/config.yaml` 的 `services:` 段添加默认服务名

### 10.2 协议端对齐检查

`controller_hardware.hpp` 中所有 `static_assert(sizeof(...) == N)` 必须与 MSDK Kotlin 端的同名结构保持一致，否则会出现"长度对但字节序错"的隐性 BUG。

### 10.3 调试日志

所有 TX/RX 事件均通过 `ROS_INFO` / `ROS_WARN` 输出，可用 `rosconsole` 配置过滤。VEL TX 失败采用 `ROS_WARN_THROTTLE(2.0, ...)` 限频。
