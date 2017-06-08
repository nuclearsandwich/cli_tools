# Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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
import collections
import importlib
import sys

import rclpy
from rclpy.qos import qos_profile_sensor_data
import yaml


# Custom representer for getting clean YAML output that preserves the order in
# an OrderedDict.
# Inspired by:
# http://stackoverflow.com/a/16782282/7169408
def represent_ordereddict(dumper, data):
    l = []
    for k, v in data.items():
        key = dumper.represent_data(k)
        value = dumper.represent_data(v)
        l.append((key, value))
    return yaml.nodes.MappingNode(u'tag:yaml.org,2002:map', l)


def register_yaml_representer():
    # Register our custom representer for YAML output
    yaml.add_representer(collections.OrderedDict, represent_ordereddict)


# Convert a msg to an OrderedDict. We do this instead of implementing a generic
# __dict__() method in the msg because we want to preserve order of fields from
# the .msg file(s).
def msg_to_ordereddict(msg):
    d = collections.OrderedDict()
    # We rely on __slots__ retaining the order of the fields in the .msg file.
    for s in msg.__slots__:
        v = getattr(msg, s, None)
        if True in [isinstance(v, t) for t in
                    [bool, bytes, dict, float, int, list, str, tuple, collections.OrderedDict]]:
            d[s[1:]] = v
        else:
            d[s[1:]] = msg_to_ordereddict(v)
    return d


def msg_to_yaml(msg):
    return yaml.dump(msg_to_ordereddict(msg), width=sys.maxsize)


def msg_to_csv(msg):
    def recurse(val):
        r = ''
        if True in [isinstance(val, t) for t in [list, tuple]]:
            for v in val:
                r += recurse(v)
        elif True in [isinstance(val, t) for t in [bool, bytes, float, int, str]]:
            r = str(val) + ','
        else:
            r = msg_to_csv(val) + ','
        return r
    result = ''
    # We rely on __slots__ retaining the order of the fields in the .msg file.
    for s in msg.__slots__:
        v = getattr(msg, s, None)
        result += recurse(v)
    return result[:-1]


def subscriber_cb(msg):
    print(msg_to_yaml(msg))


def subscriber_cb_plot(msg):
    print(msg_to_csv(msg))


def subscriber(message_type, topic_name, plot):
    separator_idx = message_type.find('/')
    message_package = message_type[:separator_idx]
    message_name = message_type[separator_idx + 1:]
    module = importlib.import_module(message_package + '.msg')
    msg_mod = getattr(module, message_name)

    rclpy.init()

    node = rclpy.create_node('subscriber_%s_%s' % (message_package, message_name))

    if plot:
        node.create_subscription(
            msg_mod, topic_name, subscriber_cb_plot, qos_profile=qos_profile_sensor_data)
    else:
        node.create_subscription(
            msg_mod, topic_name, subscriber_cb, qos_profile=qos_profile_sensor_data)

    while rclpy.ok():
        rclpy.spin_once(node)
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-p', '--plot', action='store_true',
                        help='produce comma-separated output (e.g., for plotting)')
    parser.add_argument('message_type',
                        help='type of the ROS message: e.g. "std_msgs/String"')
    parser.add_argument('topic_name',
                        help='name of the ROS topic to listen to: e.g. "chatter"')
    args = parser.parse_args()
    register_yaml_representer()
    try:
        subscriber(args.message_type, args.topic_name, args.plot)
    except KeyboardInterrupt:
        pass
