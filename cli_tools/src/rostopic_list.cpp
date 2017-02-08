// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "cmdline_parser.h"  // NOLINT

using namespace std::chrono_literals;

void
print_usage()
{
  printf("usage:\n");
  printf("rostopic_list [-h]\n");
  printf("arguments:\n");
  printf("-h: prints this help message\n");
  printf("-t: spin time to collect topics\n");
  printf("-v: verbose output prints\n");
}

void print(auto & topic_map)
{
  for (auto & map_entry : topic_map) {
    printf("%s\n", map_entry.first.c_str());
  }
}

void print_verbose(auto & topic_map)
{
  for (auto & map_entry : topic_map) {
    printf("Name: %s Type: %s\n", map_entry.first.c_str(), map_entry.second.c_str());
  }
  printf("Number of currently available topics: %zu\n", topic_map.size());
}

int main(int argc, char ** argv)
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  bool verbose = cli_option_exist(argv, argv + argc, "-v");

  auto timeout_str = cli_get_option(argv, argv + argc, "-t");
  auto timeout = 100;
  if (timeout_str) {
    timeout = std::atoi(timeout_str);
  }

  rclcpp::init(argc, argv);

  auto node_handle = std::make_shared<rclcpp::node::Node>("rostopic_list");
  rclcpp::executors::SingleThreadedExecutor exe;

  auto t = node_handle->create_wall_timer(std::chrono::milliseconds(timeout), [&exe]() {
    exe.cancel();
  });
  exe.add_node(node_handle);
  exe.spin();

  auto topic_map = node_handle->get_topic_names_and_types();
  if (verbose) {
    print_verbose(topic_map);
  } else {
    print(topic_map);
  }

  return 0;
}
