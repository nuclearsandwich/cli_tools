# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse

import rclpy

# TODO(mikaelarguedas) revisit this once it's specified
HIDDEN_NODE_PREFIX = '_'


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-a', '--all', action='store_true',
                        help='display all nodes even hidden ones')
    parser.add_argument('-t', '--time', default=0.01, type=float,
                        help='spin time to collect nodes (in seconds)')
    parser.add_argument('-n', '--number-of-nodes', action='store_true',
                        help='display the number of nodes discovered')
    args = parser.parse_args()

    timeout_reached = False

    def timer_callback():
        nonlocal timeout_reached
        timeout_reached = True

    rclpy.init()

    node = rclpy.create_node(HIDDEN_NODE_PREFIX + 'rosnode_list')
    timer = node.create_timer(args.time, timer_callback)

    while not timeout_reached:
        rclpy.spin_once(node)

    if rclpy.ok():
        node_names = node.get_node_names()
        if not args.all:
            for node_name in node_names:
                if node_name.startswith(HIDDEN_NODE_PREFIX):
                    node_names.remove(node_name)
        print(*node_names, sep='\n')
        if args.number_of_nodes:
            print('Number of currently available nodes: %d' % len(node_names))
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()
