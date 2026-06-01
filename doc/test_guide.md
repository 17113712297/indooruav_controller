# indooruav_controller 测试工具使用指南

## 1. 启动

```bash
# 终端 1 — 启动 controller 节点（需无人机连接）
roslaunch indooruav_controller bringup_controller_hardware.launch

# 终端 2 — 启动测试脚本
rosrun indooruav_controller test_controller_hardware.py
```

## 2. 菜单说明

```
┌──────────────────────────────────────────────────────────────────┐
│             indooruav_controller 测试工具                         │
├──────────────────────────────────────────────────────────────────┤
│  f1  起飞 (takeoff)              f2  降落 (land)                  │
│  f3  悬停 (hover)                                                │
├──────────────────────────────────────────────────────────────────┤
│  v   速度控制 (10 Hz cmd_vel)                                     │
├──────────────────────────────────────────────────────────────────┤
│  g1  云台 Yaw Follow             g2  云台角度控制                 │
├──────────────────────────────────────────────────────────────────┤
│  cm  相机 → 拍照模式              cv  相机 → 录像模式              │
│  cs  拍照 (shoot)                                                │
│  rc  录像开始                     rs  录像停止                     │
│  vc  视频参数配置                 zo  变焦控制                     │
├──────────────────────────────────────────────────────────────────┤
│  l1  补光灯开                     l2  补光灯关                     │
│  l3  补光灯自动                                                   │
├──────────────────────────────────────────────────────────────────┤
│  a   一键起飞→悬停→降落 (测试序列)                                 │
│  h   帮助 (重印菜单)               q   退出                        │
└──────────────────────────────────────────────────────────────────┘
```

## 3. 功能详解

### 3.1 飞控

| 按键 | 功能 | 对应服务 |
|---|---|---|
| `f1` | 起飞 | `.../state_machine_action/takeoff` |
| `f2` | 降落 | `.../state_machine_action/land` |
| `f3` | 悬停 | `.../state_machine_action/hover` |

> 起飞/降落/悬停期间，`cmd_vel` 速度门关闭，速度指令将被丢弃。完成后 MSDK 回 ACK，门重新打开。

### 3.2 速度控制

输入 `v` 进入速度控制模式。坐标采用 **ROS REP-103 机体坐标系**：

```
Forward (前)
    ↑
    |
    x
    |
    +----→ y (左)
```

| 字段 | 含义 | 单位 | 正值 |
|---|---|---|---|
| `vx` (linear.x) | 前后速度 | m/s | 前 |
| `vy` (linear.y) | 左右速度 | m/s | 左 |
| `vz` (linear.z) | 升降速度 | m/s | 上 |
| `yaw_rate` (angular.z) | 偏航角速度 | rad/s | 逆时针 |

**示例：**

```
vel > 0.5 0 0 0       → 以 0.5 m/s 向前飞
vel > 0 0 -0.3 0      → 以 0.3 m/s 下降
vel > 0 0 0 0.5       → 以 0.5 rad/s 逆时针旋转
vel > 0 0.2 0.1 0     → 向左 0.2 m/s + 上升 0.1 m/s
vel >                  → 发送零速度 → 悬停 → 退出
```

- 输入新速度后脚本以 **10 Hz** 持续发布到 `/drone/cmd_vel`，直到再次输入覆盖
- 直接回车（空输入）会发送零速度并退出速度控制模式
- Controller 节点不打印速度日志；可在另一终端 `rostopic echo /drone/cmd_vel` 查看

### 3.3 云台控制

| 按键 | 功能 | 对应服务 |
|---|---|---|
| `g1` | Yaw Follow | `.../gimbal/yaw_follow` |
| `g2` | 角度控制 | `.../gimbal/angle` |

**g1 — Yaw Follow：** 设置 pitch/roll 偏移，云台 yaw 跟随机头。

```
  pitch (deg): -90      # 俯仰角
  roll  (deg): 0        # 横滚角
```

**g2 — 角度控制：** 绝对或相对角度模式。

```
  mode: 0=ABSOLUTE  1=RELATIVE
  pitch    (deg): -90
  roll     (deg): 0
  yaw      (deg): 45
  duration (s)  : 2.0    # 动作耗时，必须 > 0
```

### 3.4 相机控制

| 按键 | 功能 | 对应服务 |
|---|---|---|
| `cm` | 切换拍照模式 | `.../camera/mode_photo` |
| `cv` | 切换录像模式 | `.../camera/mode_video` |
| `cs` | 单张拍照 | `.../camera/shoot` |
| `rc` | 开始录像 | `.../notify_uav_video_recording_start` |
| `rs` | 停止录像 | `.../notify_uav_video_recording_stop` |
| `vc` | 视频参数配置 | `.../camera/video_config` |
| `zo` | 变焦 / 镜头切换 | `.../camera/zoom` |

**vc — 视频参数：**

```
  Resolution: 1=1920x1080  2=3840x2160  3=2720x1530
  Frame rate: 1=24  2=25  3=30  4=48  5=50  6=60
```

**zo — 变焦：**

```
  Lens:   0=WIDE  1=ZOOM  2=INFRARED
  Action: 0=SWITCH_ONLY  1=SWITCH_AND_SET
```

- `SWITCH_ONLY` — 仅切换镜头，不调变焦
- `SWITCH_AND_SET` — 切换镜头并设置变焦倍数（ratio 仅在 lens=ZOOM 时有效）

### 3.5 补光灯

| 按键 | 功能 | 对应服务 |
|---|---|---|
| `l1` | 开灯 | `.../notify_uav_open_light` |
| `l2` | 关灯 | `.../notify_uav_close_light` |
| `l3` | 自动 | `.../notify_uav_auto_light` |

## 4. 自动测试序列

输入 `a` 执行一键自动序列：**起飞 → 等待 6s → 悬停 → 等待 3s → 降落**。

```
> a
[自动测试序列] 起飞 → 悬停 → 降落
  [OK] takeoff succeeded.
  [OK] hover succeeded.
  [OK] land succeeded.
[自动测试序列] 完成.
```
