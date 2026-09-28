#!/usr/bin/env python3
"""Configure and activate one lifecycle node, surviving lost service replies.

    ros2 run rospider_gazebo lifecycle_activate.py slam_toolbox

Replaces nav2_lifecycle_manager for slam.launch.py; see
rospider_gazebo/lifecycle.py for why. Exits 0 once the node is active,
1 if it is not active within --timeout seconds.
"""

import argparse
import sys

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rospider_gazebo.lifecycle import bring_up

TRANSITIONS = {'configure': Transition.TRANSITION_CONFIGURE,
               'activate': Transition.TRANSITION_ACTIVATE}
CALL_TIMEOUT = 3.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('node')
    parser.add_argument('--timeout', type=float, default=60.0)
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = rclpy.create_node('lifecycle_activate_' + args.node.strip('/').replace('/', '_'))
    get = node.create_client(GetState, f'/{args.node.strip("/")}/get_state')
    change = node.create_client(ChangeState, f'/{args.node.strip("/")}/change_state')

    def call(client, request):
        if not client.wait_for_service(timeout_sec=CALL_TIMEOUT):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT)
        return future.result() if future.done() else None

    def get_state():
        response = call(get, GetState.Request())
        return None if response is None else response.current_state.label

    def change_state(name):
        node.get_logger().info(f'{args.node}: {name}')
        request = ChangeState.Request()
        request.transition.id = TRANSITIONS[name]
        call(change, request)

    active = bring_up(get_state, change_state, args.timeout)
    if active:
        node.get_logger().info(f'{args.node} is active')
    else:
        node.get_logger().error(f'{args.node} is not active after {args.timeout:.0f} s')
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if active else 1


if __name__ == '__main__':
    sys.exit(main())
