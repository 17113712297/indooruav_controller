#!/usr/bin/env python3
"""
Test tool for indooruav_controller simulation node.

Covers: takeoff / land / hover | velocity control via indooruav_controller/waypoint_tracker/cmd_vel.
"""

import rospy
import threading

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyRequest

try:
    raw_input
except NameError:
    raw_input = input

# Service names (from simulate_config.yaml)
SRV_TAKEOFF = "indooruav_controller/controller_simulate/takeoff"
SRV_LAND    = "indooruav_controller/controller_simulate/land"
SRV_HOVER   = "indooruav_controller/controller_simulate/hover"

# Velocity topic published by waypoint_tracker, forwarded by controller_simulate_node
VEL_TOPIC = "indooruav_controller/waypoint_tracker/cmd_vel"
VEL_RATE_HZ = 30.0


def _call_empty(srv_name):
    rospy.wait_for_service(srv_name, timeout=5.0)
    proxy = rospy.ServiceProxy(srv_name, Empty)
    try:
        rospy.loginfo("Calling %s ...", srv_name)
        proxy(EmptyRequest())
        rospy.loginfo("  -> success")
        print("  [OK] {} succeeded.".format(srv_name))
        return True
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        print("  [FAIL] {} failed: {}".format(srv_name, e))
        return False


VEL_HELP = """
  Velocity control mode (ROS REP-103 body frame)
    linear.x  = forward (m/s)    linear.y  = left (m/s)
    linear.z  = up (m/s)         angular.z = CCW (rad/s)

  Format: vx vy vz yaw_rate  (4 numbers)
  Examples:  0.5 0 0 0   -> fly forward at 0.5 m/s
             0 0 -0.3 0  -> descend at 0.3 m/s
             0 0 0 0.5   -> rotate CCW at 0.5 rad/s
             0 0 0 0     -> stop
             empty line   -> stop & exit"""


def _parse_vel(raw):
    parts = raw.split()
    if len(parts) == 0:
        return Twist()
    if len(parts) != 4:
        print("  Need 4 numbers: vx vy vz yaw_rate")
        return None
    try:
        vx, vy, vz, yr = map(float, parts)
    except ValueError:
        print("  Parse failed, enter numbers.")
        return None

    twist = Twist()
    twist.linear.x  = vx
    twist.linear.y  = vy
    twist.linear.z  = vz
    twist.angular.z = yr
    return twist


def vel_mode(pub):
    print(VEL_HELP)

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
        pub.publish(Twist())
        thread.join(timeout=2.0)
        print("  Exited velocity control mode.")


def auto_sequence(pub):
    print("\n[Auto sequence] takeoff -> hover -> velocity -> land")
    rospy.sleep(0.5)

    print("  [1/4] takeoff...")
    _call_empty(SRV_TAKEOFF)
    rospy.sleep(6.0)

    print("  [2/4] fly forward at 0.5 m/s for 3s...")
    fwd = Twist()
    fwd.linear.x = 0.5
    pub.publish(fwd)
    rospy.sleep(3.0)

    print("  [3/4] hover...")
    pub.publish(Twist())
    _call_empty(SRV_HOVER)
    rospy.sleep(3.0)

    print("  [4/4] land...")
    _call_empty(SRV_LAND)

    print("[Auto sequence] done.\n")


MENU = """
+-------------------------------------------------------------+
|        indooruav_controller SIM test tool                    |
+-------------------------------------------------------------+
|  1  takeoff             2  land             3  hover        |
|  v  velocity control                                        |
|  a  auto sequence (takeoff -> vel -> hover -> land)         |
|  h  help                  q  quit                           |
+-------------------------------------------------------------+"""


def main():
    rospy.init_node("test_controller_simulate", anonymous=True)

    vel_pub = rospy.Publisher(VEL_TOPIC, Twist, queue_size=1)

    print(MENU)

    while not rospy.is_shutdown():
        try:
            cmd = raw_input("\n> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("")
            break

        if cmd in ("q", "quit", "exit"):
            break
        elif cmd in ("h", "help"):
            print(MENU)
        elif cmd == "1":
            _call_empty(SRV_TAKEOFF)
        elif cmd == "2":
            _call_empty(SRV_LAND)
        elif cmd == "3":
            _call_empty(SRV_HOVER)
        elif cmd == "v":
            vel_mode(vel_pub)
        elif cmd == "a":
            auto_sequence(vel_pub)
        else:
            print("  Unknown command '{}', type h for menu.".format(cmd))

    vel_pub.publish(Twist())


if __name__ == "__main__":
    main()
