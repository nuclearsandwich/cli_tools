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

HIDDEN_TOPIC_PREFIX = '_'


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-a', '--all', action='store_true',
                        help='display all topics even hidden ones')
    parser.add_argument('-t', '--time', default=0.01, type=float,
                        help='spin time to collect topics (in seconds)')
    parser.add_argument('-n', '--number-of-topics', action='store_true',
                        help='display the number of topics discovered')
    args = parser.parse_args()

    timeout_reached = False

    def timer_callback():
        nonlocal timeout_reached
        timeout_reached = True

    rclpy.init()

    node = rclpy.create_node('rostopic_list')
    timer = node.create_timer(args.time, timer_callback)

    while not timeout_reached:
        rclpy.spin_once(node)

    if rclpy.ok():
        topic_names_and_types = node.get_topic_names_and_types()
        topic_list = str(topic_names_and_types).split('\n')
        if not args.all:
            for topic in topic_list:
                if topic.startswith(HIDDEN_TOPIC_PREFIX):
                    topic_list.remove(topic)
        print(*topic_list, sep='\n')
        if args.number_of_topics:
            print('Number of currently available topics: %d' % len(topic_list))
    node.destroy_timer(timer)
    node.destroy_node()
    rclpy.shutdown()
