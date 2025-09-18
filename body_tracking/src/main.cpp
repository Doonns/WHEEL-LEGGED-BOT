#include <fstream>
#include <memory>
#include <sstream>

#include "include/body_tracking.h"
#include "include/smart_subscriber.h" // 暂时保留，但不再直接使用

int main(int argc, char *argv[]) {
  std::stringstream ss;
  ss << "\n\tThis is body tracking package (RGB Feature Version).\n\n"
     << "\tgesture strategy usage\n"
     << "\nWake up gesture is \"Okay\".\n"
     << "Cancel gesture is \"Palm\".\n"
     << "Control will be reset if body is lost.\n"
     << "============================================\n";
  std::cout << ss.str() << std::endl;

  rclcpp::init(argc, argv);

  // 使用多线程执行器，因为 message_filters 可能在后台需要线程资源
  // 并且我们的参数节点、控制节点也可以在不同线程中运行
  rclcpp::executors::MultiThreadedExecutor exec;

  // 获取 TrackingManager 单例
  auto tracking_manager = TrackingManager::Instance();

  // 将 TrackingManager 内部创建的节点添加到执行器
  exec.add_node(tracking_manager->GetMainNode());
  auto other_nodes = tracking_manager->GetNodes();
  for (auto &node : other_nodes) {
    exec.add_node(node);
  }

  // 不再需要手动创建 SmartMsgSubscriber
  /*
  auto smart_msg_subscriber = std::make_shared<SmartMsgSubscriber>(
      "ai_msg_sub_node",
      std::bind(&TrackingManager::FeedSmart,
                TrackingManager::Instance(),
                std::placeholders::_1));
  exec.add_node(smart_msg_subscriber);
  */
  
  RCLCPP_INFO(rclcpp::get_logger("main"), "Spinning executor...");
  exec.spin();

  // release node before shutdown!
  tracking_manager->Release();

  rclcpp::shutdown();

  std::cout << "tracking node exit!\n";
  return 0;
}