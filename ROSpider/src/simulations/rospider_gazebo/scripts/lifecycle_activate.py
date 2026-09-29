#!/usr/bin/env python3
"""Configure and activate lifecycle nodes in order, surviving lost service replies.

    ros2 run rospider_gazebo lifecycle_activate.py slam_toolbox
    ros2 run rospider_gazebo lifecycle_activate.py controller_server planner_server ...

Replaces nav2_lifecycle_manager for slam.launch.py and vslam.launch.py's Nav2;
see rospider_gazebo/lifecycle.py for why. Exits 0 once every node is active,
1 if they are not all active within --timeout seconds.
"""

import argparse
import sys

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rospider_gazebo.lifecycle import bring_up_all

TRANSITIONS = {'configure': Transition.TRANSITION_CONFIGURE,
               'activate': Transition.TRANSITION_ACTIVATE}
CALL_TIMEOUT = 3.0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('nodes', nargs='+')
    parser.add_argument('--timeout', type=float, default=60.0)
    args, _ = parser.parse_known_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    names = [name.strip('/') for name in args.nodes]

    rclpy.init()
    node = rclpy.create_node('lifecycle_activate_' + names[0].replace('/', '_'))
    clients = {name: (node.create_client(GetState, f'/{name}/get_state'),
                      node.create_client(ChangeState, f'/{name}/change_state'))
               for name in names}

    def call(client, request):
        if not client.wait_for_service(timeout_sec=CALL_TIMEOUT):
            return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=CALL_TIMEOUT)
        return future.result() if future.done() else None

    def get_state(name):
        response = call(clients[name][0], GetState.Request())
        return None if response is None else response.current_state.label

    def change_state(name, transition):
        node.get_logger().info(f'{name}: {transition}')
        request = ChangeState.Request()
        request.transition.id = TRANSITIONS[transition]
        call(clients[name][1], request)

    left = bring_up_all(names, get_state, change_state, args.timeout)
    if left:
        node.get_logger().error(f'not active after {args.timeout:.0f} s: {", ".join(left)}')
    else:
        node.get_logger().info(f'active: {", ".join(names)}')
    node.destroy_node()
    rclpy.try_shutdown()
    return 1 if left else 0


if __name__ == '__main__':
    sys.exit(main())
