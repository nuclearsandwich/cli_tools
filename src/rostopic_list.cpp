#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node_handle = std::make_shared<rclcpp::node::Node>("rostopic_list");
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node_handle);
  exe.spin_once();

  auto topic_map = node_handle->get_topic_names_and_types();
  printf("Number of currently available topics: %zu\n", topic_map.size());
  for (auto & map_entry : topic_map)
  {
    printf("Name: %s\tType: %s\n", map_entry.first.c_str(), map_entry.second.c_str());
  }
  return 0;
}
