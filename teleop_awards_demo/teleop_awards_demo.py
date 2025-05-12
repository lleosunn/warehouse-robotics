import sys
import threading

import geometry_msgs.msg
import rclpy

import time

import numpy as np

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

msg = """
This node takes keypresses from the keyboard and publishes them
as Twist/TwistStamped messages.
---------------------------
Acceleration: w
Deceleration: s

CTRL-C to quit
"""

# z/x : increase/decrease maximum linear speed by 10%

a = 0.2 # comfortable acceleration
b = 0.14 # comfortable deceleration

moveBindings = {
    'w': (a, 0, 0, 0),
    's': (-b, 0, 0, 0),
}

# speedBindings = {
#     'z': 1.1,
#     'x': .9,
# }

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed):
    return 'currently:\t max speed' % (speed)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('teleop_awards_demo')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.5 # maximum velocity

    x = 0.0 # current speed
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print(msg)
        print(vels(speed))

        start_time = time.time()
        idx = 0

        while True:
            if start_time + 0.1*idx < time.time(): # update every 0.1 seconds

                key = getKey(settings)
                if key in moveBindings.keys():
                    
                    x = np.clip(x + 0.1*moveBindings[key][0], 0, speed)

                # elif key in speedBindings.keys():
                #     speed = speed * speedBindings[key][0]
                #     a = a * speedBindings[key][0]
                #     b = b * speedBindings[key][0]

                    # print(vels(speed))
                    # if (status == 14):
                    #     print(msg)
                    # status = (status + 1) % 15
                else:
                    x = 0.0
                    if (key == '\x03'):
                        break

                if stamped:
                    twist_msg.header.stamp = node.get_clock().now().to_msg()

                twist.linear.x = x * speed
                twist.linear.y = 0.0
                twist.linear.z = 0.0
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                twist.angular.z = 0.0
                pub.publish(twist_msg)

                idx += 1

    except Exception as e:
        print(e)

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()
