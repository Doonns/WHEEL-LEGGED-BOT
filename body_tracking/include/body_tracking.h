// include/body_tracking.h

#ifndef TRACKING_STRATEGY_H_
#define TRACKING_STRATEGY_H_

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <condition_variable>

#include "ai_msgs/msg/perception_targets.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "include/common.h"
#include "include/param_node.h"
#include "include/robot_ctrl_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"

#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
class TrackingManager {
 public:
  static std::shared_ptr<TrackingManager> Instance();
  ~TrackingManager();

  void Release();
  std::vector<std::shared_ptr<rclcpp::Node>> GetNodes();
  const TrackCfg &GetTrackCfg() const;
  std::shared_ptr<rclcpp::Node> GetMainNode();

 private:
  TrackingManager();

  // --- 新的回调函数 ---
  void AiMsgCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& targets_msg);
  void ImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& image_msg);

  void RunTrackingStrategy(
      const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg,
      const cv::Mat& image);
  void TrackingWithoutNavStrategy(
      const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg,
      const cv::Mat& image);
  void ProcessSmart(
      const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg,
      const cv::Mat& image);
  
  void UpdateTrackAngle();
  bool RotateSwitch();
  void CancelMove();
  void DoRotateMove();
  bool TrackingSwitchWithVision();
  void RunTrackLostProtectionStrategy();
  void RunOverMovingProtectionStrategy();
  
  // --- 辅助函数 ---
  void reset_tracking_info();
  ColorFeature extract_feature(const cv::Mat& image, const std::vector<int>& roi);
  bool match_feature(const ColorFeature& f1, const ColorFeature& f2);
  bool is_akimbo_pose(const ai_msgs::msg::Point& kps);
  bool is_arms_crossed_pose(const ai_msgs::msg::Point& kps);
  bool is_arms_extended_pose(const ai_msgs::msg::Point& kps);   //双臂水平
  std::mutex robot_strategy_mtx_;
  std::atomic_bool last_frame_done_;

  std::shared_ptr<rclcpp::Node> main_node_ = nullptr;
  std::shared_ptr<ParametersClass> param_node_ = nullptr;
  std::shared_ptr<RobotCmdVelNode> robot_cmdvel_node_ = nullptr;

  TrackInfo track_info_;
  TrackCfg track_cfg_;
  bool start_ = false;

  // --- 普通订阅者和图像缓存 ---
  rclcpp::Subscription<ai_msgs::msg::PerceptionTargets>::SharedPtr targets_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_; // <-- 
  
  
  sensor_msgs::msg::CompressedImage::ConstSharedPtr latest_image_msg_ = nullptr; // <-- 修改类型
  std::mutex image_buffer_mutex_; 

  bool last_cmdvel_is_cancel_ = false;
  int last_cmdvel_type_ = -1;
  float last_move_step_ratio_ = 1.0;
};
#endif