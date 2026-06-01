#!/usr/bin/env python3
"""
Interactive test tool for indooruav_controller.

Covers: takeoff / land / hover | velocity control | gimbal yaw-follow / angle |
camera mode / shoot / record / video-config / zoom | fill light.
"""

import rospy
import sys
import threading
from geometry_msgs.msg import Twist

try:
    raw_input
except NameError:
    raw_input = input  # Python 3

from std_srvs.srv import Empty, EmptyRequest
from indooruav_msgs.srv import (
    GimbalYawFollow, GimbalYawFollowRequest,
    GimbalAngle, GimbalAngleRequest,
    CameraVideoConfig, CameraVideoConfigRequest,
    CameraZoom, CameraZoomRequest,
)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
NS = "indooruav_controller/controller_hardware"
VEL_TOPIC = "indooruav_controller/waypoint_tracker/cmd_vel"
VEL_RATE_HZ = 10.0


def _srv(name):
    return "{}/{}".format(NS, name)


# ---------------------------------------------------------------------------
# Service registry: key -> (service_name, srv_type, has_payload)
# ---------------------------------------------------------------------------
SERVICES = {
    "takeoff":          (_srv("takeoff"),                       Empty,             False),
    "land":             (_srv("land"),                          Empty,             False),
    "hover":            (_srv("hover"),                         Empty,             False),
    "light_on":         (_srv("light_open"),                    Empty,             False),
    "light_off":        (_srv("light_close"),                   Empty,             False),
    "light_auto":       (_srv("light_auto"),                    Empty,             False),
    "cam_mode_photo":   (_srv("camera_mode_photo"),             Empty,             False),
    "cam_mode_video":   (_srv("camera_mode_video"),             Empty,             False),
    "cam_shoot":        (_srv("camera_photo_shoot"),            Empty,             False),
    "cam_record_start": (_srv("camera_video_start"),            Empty,             False),
    "cam_record_stop":  (_srv("camera_video_stop"),             Empty,             False),
    "gimbal_yaw_follow":(_srv("gimbal_yaw_follow"),            GimbalYawFollow,   True),
    "gimbal_angle":     (_srv("gimbal_angle"),                 GimbalAngle,       True),
    "cam_video_config": (_srv("camera_video_config"),          CameraVideoConfig, True),
    "cam_zoom":         (_srv("camera_zoom"),                  CameraZoom,        True),
}

# ---------------------------------------------------------------------------
# Prompt helpers
# ---------------------------------------------------------------------------

def _prompt_float(prompt, default=None):
    raw = raw_input(prompt)
    if not raw and default is not None:
        return default
    return float(raw)


def _prompt_int(prompt, default=None):
    raw = raw_input(prompt)
    if not raw and default is not None:
        return default
    return int(raw)


def build_gimbal_yaw_follow():
    print("\n-- Gimbal Yaw Follow --")
    req = GimbalYawFollowRequest()
    req.pitch = _prompt_float("  pitch (deg): ", 0.0)
    req.roll  = _prompt_float("  roll  (deg): ", 0.0)
    return req


def build_gimbal_angle():
    print("\n-- Gimbal Angle --")
    req = GimbalAngleRequest()
    print("  mode: 0=ABSOLUTE  1=RELATIVE")
    req.mode = _prompt_int("  mode: ", 0)
    req.pitch = _prompt_float("  pitch    (deg): ", 0.0)
    req.roll  = _prompt_float("  roll     (deg): ", 0.0)
    req.yaw   = _prompt_float("  yaw      (deg): ", 0.0)
    req.duration = _prompt_float("  duration (s)  : ", 2.0)
    return req


def build_cam_video_config():
    print("\n-- Camera Video Config --")
    print("  Resolution: 1=1920x1080  2=3840x2160  3=2720x1530")
    print("  Frame rate: 1=24  2=25  3=30  4=48  5=50  6=60")
    req = CameraVideoConfigRequest()
    req.resolution = _prompt_int("  resolution: ", 1)
    req.frame_rate = _prompt_int("  frame_rate: ", 3)
    return req


def build_cam_zoom():
    print("\n-- Camera Zoom --")
    print("  Lens:   0=WIDE  1=ZOOM  2=INFRARED")
    print("  Action: 0=SWITCH_ONLY  1=SWITCH_AND_SET")
    req = CameraZoomRequest()
    req.lens   = _prompt_int("  lens  : ", 0)
    req.action = _prompt_int("  action: ", 0)
    if req.action == 1:
        req.ratio = _prompt_float("  ratio : ", 1.0)
    else:
        req.ratio = 0.0
    return req


PAYLOAD_BUILDERS = {
    "gimbal_yaw_follow": build_gimbal_yaw_follow,
    "gimbal_angle":      build_gimbal_angle,
    "cam_video_config":  build_cam_video_config,
    "cam_zoom":          build_cam_zoom,
}

# ---------------------------------------------------------------------------
# Service caller
# ---------------------------------------------------------------------------
_proxies = {}


def _get_proxy(srv_name, srv_type):
    if srv_name not in _proxies:
        rospy.wait_for_service(srv_name, timeout=5.0)
        _proxies[srv_name] = rospy.ServiceProxy(srv_name, srv_type)
    return _proxies[srv_name]


def call_service(key):
    srv_name, srv_type, has_payload = SERVICES[key]
    if has_payload:
        req = PAYLOAD_BUILDERS[key]()
    else:
        req = EmptyRequest()
    try:
        proxy = _get_proxy(srv_name, srv_type)
        rospy.loginfo("Calling %s ...", srv_name)
        proxy(req)
        rospy.loginfo("  -> success")
        print("  [OK] {} succeeded.".format(key))
        return True
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        print("  [FAIL] {} failed: {}".format(key, e))
        return False

# ---------------------------------------------------------------------------
# Velocity control
# ---------------------------------------------------------------------------

VEL_HELP = """
  速度控制模式 (ROS REP-103 机体坐标系)
    linear.x  = 前 (m/s)    linear.y  = 左 (m/s)
    linear.z  = 上 (m/s)    angular.z = 逆时针 (rad/s)

  输入格式: vx vy vz yaw_rate  (4 个数字，空格分隔)
  示例:  0.5 0 0 0      → 以 0.5 m/s 向前飞
         0 0 -0.3 0     → 以 0.3 m/s 下降
         0 0 0 0.5      → 以 0.5 rad/s 逆时针旋转
         0 0 0 0        → 悬停 (零速度)
         回车 (空输入)    → 发零 → 退出速度控制"""


def _parse_vel(raw):
    """Parse 'vx vy vz yaw_rate' string, return Twist or None on failure."""
    parts = raw.split()
    if len(parts) == 0:
        return Twist()  # stop
    if len(parts) != 4:
        print("  需要 4 个数字: vx vy vz yaw_rate")
        return None
    try:
        vx, vy, vz, yr = map(float, parts)
    except ValueError:
        print("  解析失败，请输数字。")
        return None

    twist = Twist()
    twist.linear.x  = vx
    twist.linear.y  = vy
    twist.linear.z  = vz
    twist.angular.z = yr
    return twist


def vel_mode(pub, rate):
    """
    Enter velocity-control sub-loop.
    Publishes the latest user-set velocity at 10 Hz.  Empty line → stop & exit.
    """
    print(VEL_HELP)

    # shared mutable: [latest_twist, running_flag]
    latest = [Twist()]
    running = [True]

    def _pub_loop():
        r = rospy.Rate(VEL_RATE_HZ)
        while not rospy.is_shutdown() and running[0]:
            pub.publish(latest[0])
            r.sleep()

    thread = threading.Thread(target=_pub_loop)
    thread.daemon = True
    thread.start()

    try:
        while not rospy.is_shutdown():
            raw = raw_input("vel > ").strip()
            if not raw:
                # stop & exit
                rospy.loginfo("Velocity control: stop & exit.")
                break
            twist = _parse_vel(raw)
            if twist is not None:
                latest[0] = twist
                rospy.loginfo("  set: vx=%.2f vy=%.2f vz=%.2f yaw_rate=%.2f",
                              twist.linear.x, twist.linear.y,
                              twist.linear.z, twist.angular.z)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        running[0] = False
        # send a final zero-velocity to stop the drone
        pub.publish(Twist())
        thread.join(timeout=2.0)
        print("  已退出速度控制模式。")

# ---------------------------------------------------------------------------
# Menus
# ---------------------------------------------------------------------------

MAIN_MENU = """
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
└──────────────────────────────────────────────────────────────────┘"""


def print_menu():
    print(MAIN_MENU)


def auto_sequence():
    print("\n[自动测试序列] 起飞 → 悬停 → 降落")
    rospy.sleep(0.5)
    call_service("takeoff")
    rospy.sleep(6.0)
    call_service("hover")
    rospy.sleep(3.0)
    call_service("land")
    print("[自动测试序列] 完成.\n")

# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    rospy.init_node("test_controller_hardware", anonymous=True)

    # velocity publisher (10 Hz)
    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)
    vel_rate = rospy.Rate(VEL_RATE_HZ)

    print_menu()

    while not rospy.is_shutdown():
        try:
            cmd = raw_input("\n> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("")
            break

        if cmd in ("q", "quit", "exit"):
            break
        elif cmd in ("h", "help"):
            print_menu()
        elif cmd == "f1":
            call_service("takeoff")
        elif cmd == "f2":
            call_service("land")
        elif cmd == "f3":
            call_service("hover")
        elif cmd == "v":
            vel_mode(vel_pub, vel_rate)
        elif cmd == "g1":
            call_service("gimbal_yaw_follow")
        elif cmd == "g2":
            call_service("gimbal_angle")
        elif cmd == "cm":
            call_service("cam_mode_photo")
        elif cmd == "cv":
            call_service("cam_mode_video")
        elif cmd == "cs":
            call_service("cam_shoot")
        elif cmd == "rc":
            call_service("cam_record_start")
        elif cmd == "rs":
            call_service("cam_record_stop")
        elif cmd == "vc":
            call_service("cam_video_config")
        elif cmd == "zo":
            call_service("cam_zoom")
        elif cmd == "l1":
            call_service("light_on")
        elif cmd == "l2":
            call_service("light_off")
        elif cmd == "l3":
            call_service("light_auto")
        elif cmd == "a":
            auto_sequence()
        else:
            print("  未知指令 '{}'，输入 h 查看菜单.".format(cmd))

    # ensure zero velocity on exit
    vel_pub.publish(Twist())


if __name__ == "__main__":
    main()
