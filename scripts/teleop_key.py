#!/usr/bin/env python3

import argparse
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist


def make_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Keyboard teleop for tank chassis")
    p.add_argument("--linear", type=float, default=0.3, help="Max linear speed [m/s]")
    p.add_argument("--angular", type=float, default=0.6, help="Max angular speed [rad/s]")
    p.add_argument("--topic", default="/cmd_vel", help="Twist topic (default: /cmd_vel)")
    return p


def get_key_blocking(fd: int, old: list) -> str:
    """Enter raw mode, read one key, restore terminal, return the key."""
    tty.setraw(fd)
    key = sys.stdin.read(1)
    termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return key


def run(args: argparse.Namespace) -> None:
    rclpy.init()
    node = rclpy.create_node("teleop_key")
    pub = node.create_publisher(Twist, args.topic, 10)

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)

    print("=" * 50)
    print("   Tank chassis keyboard teleop")
    print(f"   linear max: {args.linear} m/s   angular max: {args.angular} rad/s")
    print("-" * 50)
    print("   W          forward")
    print("   S          backward")
    print("   A          rotate left (ccw)")
    print("   D          rotate right (cw)")
    print("   Q          forward + left")
    print("   E          forward + right")
    print("   + / -      speed up / slow down")
    print("   Space      emergency stop")
    print("   Ctrl+C     quit")
    print("=" * 50)
    print("   [ready] press a key to move, release to stop")

    speed_scale = 1.0

    try:
        while rclpy.ok():
            key = get_key_blocking(fd, old)

            twist = Twist()
            if key == "w":
                twist.linear.x = 1.0
            elif key == "s":
                twist.linear.x = -1.0
            elif key == "a":
                twist.angular.z = 1.0
            elif key == "d":
                twist.angular.z = -1.0
            elif key == "q":
                twist.linear.x = 1.0
                twist.angular.z = 1.0
            elif key == "e":
                twist.linear.x = 1.0
                twist.angular.z = -1.0
            elif key == " ":
                speed_scale = 1.0
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pub.publish(twist)
                print("   [stop]")
                continue
            elif key == "+" or key == "=":
                speed_scale = min(2.0, speed_scale + 0.1)
                print(f"   [speed scale: {speed_scale:.1f}]")
                continue
            elif key == "-" or key == "_":
                speed_scale = max(0.1, speed_scale - 0.1)
                print(f"   [speed scale: {speed_scale:.1f}]")
                continue
            elif key == "\x03":
                break
            else:
                # unknown key or release sequence → stop
                pass

            twist.linear.x *= args.linear * speed_scale
            twist.angular.z *= args.angular * speed_scale
            pub.publish(twist)

    finally:
        twist = Twist()
        pub.publish(twist)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run(make_parser().parse_args())
