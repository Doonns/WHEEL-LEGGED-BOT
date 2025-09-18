// Copyright (c) 2024，D-Robotics.
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

#include "include/body_tracking.h"

#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "include/time_helper.h"
#include "include/util.h"

std::shared_ptr<TrackingManager> TrackingManager::Instance() {
  static std::shared_ptr<TrackingManager> inst = nullptr;
  if (!inst) {
    inst = std::shared_ptr<TrackingManager>(new TrackingManager());
  }
  return inst;
}

TrackingManager::TrackingManager() {
  start_ = true;
  track_info_.is_movectrl_running = false;
  last_frame_done_ = true;

  main_node_ = std::make_shared<rclcpp::Node>("body_tracking_node");

  param_node_ = std::make_shared<ParametersClass>(&track_cfg_);
  robot_cmdvel_node_ =
      std::make_shared<RobotCmdVelNode>("horizon_tracking_RobotCmdVel");
      
  main_node_->declare_parameter<std::string>("image_topic", "/image");
  main_node_->declare_parameter<std::string>("ai_msg_topic", "/hobot_mono2d_body_detection");
  
  std::string image_topic = main_node_->get_parameter("image_topic").as_string();
  std::string ai_msg_topic = main_node_->get_parameter("ai_msg_topic").as_string();

  RCLCPP_INFO(main_node_->get_logger(), "Subscribing to image_topic: %s", image_topic.c_str());
  RCLCPP_INFO(main_node_->get_logger(), "Subscribing to ai_msg_topic: %s", ai_msg_topic.c_str());

  // --- 初始化两个独立的订阅者 ---
  // 图像订阅者
  // 创建一个与发布者完全匹配的 QoS
  auto image_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  image_sub_ = main_node_->create_subscription<sensor_msgs::msg::CompressedImage>( // <-- 修改类型
      image_topic, 
      image_qos, // <-- 使用新的、匹配的 QoS
      std::bind(&TrackingManager::ImageCallback, this, std::placeholders::_1));

  // AI消息订阅者 (为了保险，也改成 reliable)
  auto ai_msg_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
  targets_sub_ = main_node_->create_subscription<ai_msgs::msg::PerceptionTargets>(
      ai_msg_topic,
      ai_msg_qos, // <-- 改成 reliable
      std::bind(&TrackingManager::AiMsgCallback, this, std::placeholders::_1));
  
  RCLCPP_INFO(main_node_->get_logger(), "TrackingManager initialized with independent subscribers.");
}

TrackingManager::~TrackingManager() {}

/**
 * @brief 从图像的指定ROI中提取颜色特征
 * @param image 输入的OpenCV图像 (BGR格式)
 * @param roi 感兴趣区域，格式为 [x1, y1, x2, y2]
 * @return 提取出的 ColorFeature (R, G, B)
 */
ColorFeature TrackingManager::extract_feature(const cv::Mat& image, const std::vector<int>& roi) {
  if (roi.size() < 4) {
    RCLCPP_ERROR(main_node_->get_logger(), "extract_feature: Invalid ROI size.");
    return ColorFeature{0, 0, 0};
  }

  // 截取ROI的中心区域，以减少背景和边缘的干扰
  // 我们取ROI中心 50% 宽、50% 高的区域
  int roi_width = roi[2] - roi[0];
  int roi_height = roi[3] - roi[1];
  
  int center_x = roi[0] + roi_width / 2;
  int center_y = roi[1] + roi_height / 2;

  int feature_roi_width = roi_width * 0.5;
  int feature_roi_height = roi_height * 0.5;

  int x1 = center_x - feature_roi_width / 2;
  int y1 = center_y - feature_roi_height / 2;

  // 创建一个 cv::Rect 用于裁剪，并进行边界检查
  cv::Rect feature_rect(x1, y1, feature_roi_width, feature_roi_height);
  cv::Rect valid_rect = feature_rect & cv::Rect(0, 0, image.cols, image.rows);

  if (valid_rect.width <= 0 || valid_rect.height <= 0) {
    RCLCPP_WARN(main_node_->get_logger(), "extract_feature: Feature ROI is out of image bounds or has zero size.");
    return ColorFeature{0, 0, 0};
  }
  
  // 提取特征区域的图像块
  cv::Mat feature_roi_mat = image(valid_rect);
  
  // 计算该区域的平均颜色
  // cv::mean 返回一个 cv::Scalar，其在BGR图像中的顺序是 (Blue, Green, Red, Alpha)
  cv::Scalar mean_color = cv::mean(feature_roi_mat);
  
  // 返回时按照 R, G, B 的顺序存储
  return ColorFeature{mean_color[2], mean_color[1], mean_color[0]};
}

/**
 * @brief 比较两个颜色特征是否匹配
 * @param f1 第一个特征
 * @param f2 第二个特征
 * @return 如果每个通道的差值都在阈值内，则返回 true
 */
bool TrackingManager::match_feature(const ColorFeature& f1, const ColorFeature& f2) {
  return (std::abs(f1.r - f2.r) < track_cfg_.feature_match_threshold) &&
         (std::abs(f1.g - f2.g) < track_cfg_.feature_match_threshold) &&
         (std::abs(f1.b - f2.b) < track_cfg_.feature_match_threshold);
}


/**
 * @brief 重置追踪状态信息
 */
void TrackingManager::reset_tracking_info() {
  track_info_.tracking_sta = TrackingStatus::INITING;
  track_info_.track_id = 0;
  track_info_.frame_ts = 0;
  track_info_.frame_ts_sec = 0;
  track_info_.frame_ts_nanosec = 0;
  track_info_.serial_lost_num = 0;
  track_info_.last_rect.clear();
  track_info_.present_rect.clear();
  track_info_.present_body_kps.clear();
  track_info_.is_movectrl_running = false; // 直接对 atomic 赋值
  track_info_.has_face_head = false;
  track_info_.serial_lost_face_head_num = 0;
  track_info_.gesture = 0;
  track_info_.angel_with_robot_ = 0;
  track_info_.robot_y_negtive_radian_with_track_ = 0;
  track_info_.robot_x_positive_radian_with_track_ = 0;
  track_info_.move_distance_ = 0;
  track_info_.move_direction = 0;
  track_info_.move_step = 0.1;

  // 重置我们新增的特征信息
  track_info_.target_feature = ColorFeature{0, 0, 0};
  track_info_.feature_is_set = false;
}





/**
 * @brief 判断是否为“叉腰”姿态
 * @param kps 从 ai_msgs::msg::PerceptionTargets 中获取的一组关键点
 * @return 如果是叉腰姿态，返回 true
 */
// src/body_tracking.cpp

// src/body_tracking.cpp (is_akimbo_pose 函数)

bool TrackingManager::is_akimbo_pose(const ai_msgs::msg::Point& kps) {
    if (kps.point.size() < 13) return false; 

    const auto& p = kps.point;
    
    // ... (关键点有效性检查代码不变) ...

    auto l_shoulder = p[6], r_shoulder = p[5]; 
    auto l_elbow = p[8],    r_elbow = p[7];    
    auto l_wrist = p[10],   r_wrist = p[9];    
    auto l_hip = p[12],     r_hip = p[11];     

    float shoulder_width = std::abs(l_shoulder.x - r_shoulder.x);
    if (shoulder_width < 20) { 
        RCLCPP_INFO(main_node_->get_logger(), "is_akimbo_pose: Shoulder width too small (%.1f)", shoulder_width);
        return false;
    }
    
    float body_center_x = (l_shoulder.x + r_shoulder.x) / 2.0f;

    // --- 叉腰姿态的几何条件判断（Y坐标：越小越靠上） ---

    // 1. 垂直方向（Y轴）判断：手腕在腰部区域，手肘在手腕上方
    // Y轴下沉范围：手腕Y应在肩膀Y下方 (min_dist)，但不能低于臀部太多 (max_dist)
    // 增大 y_wrist_below_shoulder_min_dist 阈值，让手腕可以更低一点也算合格
    float y_wrist_below_shoulder_min_dist = std::max(20.0f, shoulder_width * 0.2f); // 手腕至少比肩膀低20像素或20%肩宽
    float y_wrist_below_shoulder_max_dist = std::max(80.0f, shoulder_width * 0.8f); // 手腕不能比肩膀低太多，80像素或80%肩宽

    bool left_wrist_y_ok = (l_wrist.y > l_shoulder.y + y_wrist_below_shoulder_min_dist) && 
                           (l_wrist.y < l_shoulder.y + y_wrist_below_shoulder_max_dist); 
                           
    bool right_wrist_y_ok = (r_wrist.y > r_shoulder.y + y_wrist_below_shoulder_min_dist) && 
                            (r_wrist.y < r_shoulder.y + y_wrist_below_shoulder_max_dist); 

    // 手肘在手腕上方
    bool left_elbow_y_above_wrist_ok = (l_elbow.y < l_wrist.y - 10.0f); 
    bool right_elbow_y_above_wrist_ok = (r_elbow.y < r_wrist.y - 10.0f);

    // 2. 水平方向（X轴）判断：手腕向内，手肘向外
    float x_elbow_outward_threshold = std::max(20.0f, shoulder_width * 0.2f); 
    // 重新定义手腕内收阈值：手腕X应在同侧肩膀X的内侧，但不要求严格在身体中线内（允许稍微越过）
    float x_wrist_inward_offset = std::max(5.0f, shoulder_width * 0.05f); // 相对肩膀X的内收偏移

    // 左手叉腰 X 坐标判断
    bool left_elbow_x_out = (l_elbow.x < l_shoulder.x - x_elbow_outward_threshold); 
    // 左手腕应在左肩X的右侧 (X更大)，并且不应该太靠右（例如，不能超过右肩X）
    bool left_wrist_x_in = (l_wrist.x > l_shoulder.x + x_wrist_inward_offset) && (l_wrist.x < r_shoulder.x - x_wrist_inward_offset); 

    // 右手叉腰 X 坐标判断
    bool right_elbow_x_out = (r_elbow.x > r_shoulder.x + x_elbow_outward_threshold); 
    // 右手腕应在右肩X的左侧 (X更小)，并且不应该太靠左（例如，不能超过左肩X）
    bool right_wrist_x_in = (r_wrist.x < r_shoulder.x - x_wrist_inward_offset) && (r_wrist.x > l_shoulder.x + x_wrist_inward_offset); 

    // --- 调试日志 --- 
    RCLCPP_INFO(main_node_->get_logger(), 
                "Akimbo Pose Check: L_wrist(%.0f,%.0f) L_elbow(%.0f,%.0f) L_shoulder(%.0f,%.0f) L_hip(%.0f,%.0f) | R_wrist(%.0f,%.0f) R_elbow(%.0f,%.0f) R_shoulder(%.0f,%.0f) R_hip(%.0f,%.0f)",
                l_wrist.x, l_wrist.y, l_elbow.x, l_elbow.y, l_shoulder.x, l_shoulder.y, l_hip.x, l_hip.y,
                r_wrist.x, r_wrist.y, r_elbow.x, r_elbow.y, r_shoulder.x, r_shoulder.y, r_hip.x, r_hip.y);
    
    // 打印更详细的子条件状态
    RCLCPP_INFO(main_node_->get_logger(), 
                "Akimbo Conditions L: YW:%d, YE:%d, XEo:%d, XWi:%d (XWi-Cond1:%d, XWi-Cond2:%d) | Akimbo Conditions R: YW:%d, YE:%d, XEo:%d, XWi:%d (XWi-Cond1:%d, XWi-Cond2:%d) | SW:%.1f, BCx:%.1f, YW_Min_Dist:%.1f, YW_Max_Dist:%.1f, Y_El_Ab_Wr:%.1f, X_El_Out_Thr:%.1f, X_Wr_In_Offset:%.1f",
                left_wrist_y_ok, left_elbow_y_above_wrist_ok, left_elbow_x_out, left_wrist_x_in, 
                (l_wrist.x > l_shoulder.x + x_wrist_inward_offset), (l_wrist.x < r_shoulder.x - x_wrist_inward_offset), // XWi的两个子条件
                right_wrist_y_ok, right_elbow_y_above_wrist_ok, right_elbow_x_out, right_wrist_x_in,
                (r_wrist.x < r_shoulder.x - x_wrist_inward_offset), (r_wrist.x > l_shoulder.x + x_wrist_inward_offset), // XWi的两个子条件
                shoulder_width, body_center_x, y_wrist_below_shoulder_min_dist, y_wrist_below_shoulder_max_dist, 10.0f, x_elbow_outward_threshold, x_wrist_inward_offset); 
    // --- 结束调试日志 ---

    bool is_left_akimbo = left_wrist_y_ok && left_elbow_y_above_wrist_ok && left_elbow_x_out && left_wrist_x_in;
    bool is_right_akimbo = right_wrist_y_ok && right_elbow_y_above_wrist_ok && right_elbow_x_out && right_wrist_x_in;

    if (is_left_akimbo || is_right_akimbo) {
        RCLCPP_WARN(main_node_->get_logger(), "!!! Akimbo Pose Detected (Start Tracking) !!!");
        return true;
    }
    return false;
}



/**
 * @brief 判断是否为“双臂交叉于胸前”姿态
 * @param kps 从 ai_msgs::msg::PerceptionTargets 中获取的一组关键点
 * @return 如果是双臂交叉姿态，返回 true
 */
// src/body_tracking.cpp (is_arms_crossed_pose 函数)

// 禁用
bool TrackingManager::is_arms_crossed_pose(const ai_msgs::msg::Point& kps) {
    if (kps.point.size() < 13) return false; 

    const auto& p = kps.point;
    // 检查核心关键点是否被检测到
    if ( (p[5].x == 0 && p[5].y == 0) || (p[6].x == 0 && p[6].y == 0) ||
         (p[9].x == 0 && p[9].y == 0) || (p[10].x == 0 && p[10].y == 0) ) {
        RCLCPP_INFO(main_node_->get_logger(), "is_arms_crossed_pose: Missing essential keypoint data (0,0).");
        return false;
    }

    // ******** 关键修改：交换左右关键点引用 ********
    auto l_shoulder = p[6], r_shoulder = p[5]; // p[6]现在是左肩，p[5]是右肩
    auto l_wrist = p[10],   r_wrist = p[9];    // p[10]现在是左手腕，p[9]是右手腕
    // *************************************************

    float body_center_x = (l_shoulder.x + r_shoulder.x) / 2.0f;
    float shoulder_width = std::abs(l_shoulder.x - r_shoulder.x);
    if (shoulder_width < 50) { 
        RCLCPP_INFO(main_node_->get_logger(), "is_arms_crossed_pose: Shoulder width too small (%.1f)", shoulder_width);
        return false;
    }
    
    // 双臂交叉的几何条件判断
    // 1. 左右手腕的X坐标发生了交叉 (左手腕在身体中线右侧，右手腕在身体中线左侧)
    // 这是相对于我们现在矫正过的左右识别的，所以条件是 l_wrist.x > body_center_x (左手腕在右半边)
    bool wrists_crossed_x = (l_wrist.x > body_center_x) && (r_wrist.x < body_center_x);
    
    // 2. 两个手腕的高度约在肩膀高度下方 (Y更大)，但不能太低（不能低于估计的臀部Y）
    float approx_hip_y = l_shoulder.y + shoulder_width * 1.5; 
    float y_cross_threshold = std::max(20.0f, shoulder_width * 0.3f); 
    bool left_wrist_height_valid = (l_wrist.y > l_shoulder.y + y_cross_threshold) && (l_wrist.y < approx_hip_y);
    bool right_wrist_height_valid = (r_wrist.y > r_shoulder.y + y_cross_threshold) && (r_wrist.y < approx_hip_y);


    RCLCPP_INFO(main_node_->get_logger(), 
                "Arms Crossed Check: L_wrist(%.0f,%.0f) R_wrist(%.0f,%.0f) | Shoulders(%.0f,%.0f)-(%.0f,%.0f) | Body_Cx:%.1f | Approx_Hip_Y:%.0f",
                l_wrist.x, l_wrist.y, r_wrist.x, r_wrist.y,
                l_shoulder.x, l_shoulder.y, r_shoulder.x, r_shoulder.y, body_center_x, approx_hip_y);
    RCLCPP_INFO(main_node_->get_logger(), 
                "Arms Crossed Conditions: X_Crossed:%d, L_H_Valid:%d, R_H_Valid:%d | SW:%.1f, Y_Cross_Thr:%.1f",
                wrists_crossed_x, left_wrist_height_valid, right_wrist_height_valid, shoulder_width, y_cross_threshold);


    if (wrists_crossed_x && left_wrist_height_valid && right_wrist_height_valid) {
        RCLCPP_WARN(main_node_->get_logger(), "!!! Arms Crossed Pose Detected (Stop Tracking) !!!");
        return true;
    }
    return false;
}

/**
 * @brief 判断是否为“双臂伸展”（T-Pose）姿态
 * @param kps 从 ai_msgs::msg::PerceptionTargets 中获取的一组关键点
 * @return 如果是双臂伸展姿态，返回 true
 */
bool TrackingManager::is_arms_extended_pose(const ai_msgs::msg::Point& kps) {
    if (kps.point.size() < 13) return false;

    const auto& p = kps.point;
    
    // 检查核心关键点是否被检测到 (肩膀, 手肘, 手腕)
    if ( (p[5].x == 0 && p[5].y == 0) || (p[6].x == 0 && p[6].y == 0) ||
         (p[7].x == 0 && p[7].y == 0) || (p[8].x == 0 && p[8].y == 0) ||
         (p[9].x == 0 && p[9].y == 0) || (p[10].x == 0 && p[10].y == 0) ) {
        RCLCPP_INFO(main_node_->get_logger(), "is_arms_extended_pose: Missing essential keypoint data (0,0).");
        return false;
    }

    // 正确的 COCO 17-point 索引引用
    auto& r_shoulder = p[5];
    auto& l_shoulder = p[6];
    auto& r_elbow = p[7];
    auto& l_elbow = p[8];
    auto& r_wrist = p[9];
    auto& l_wrist = p[10];

    float shoulder_width = std::abs(l_shoulder.x - r_shoulder.x);
    if (shoulder_width < 20) {
        RCLCPP_INFO(main_node_->get_logger(), "is_arms_extended_pose: Shoulder width too small (%.1f)", shoulder_width);
        return false;
    }

    // --- 双臂伸展姿态的几何条件判断 ---

    // 1. 水平伸展判断 (X轴):
    //    - 左臂：手肘在肩膀左侧，手腕在手肘左侧
    //    - 右臂：手肘在肩膀右侧，手腕在手肘右侧
    // bool left_arm_extended_x = (l_elbow.x < l_shoulder.x) && (l_wrist.x < l_elbow.x);
    // bool right_arm_extended_x = (r_elbow.x > r_shoulder.x) && (r_wrist.x > r_elbow.x);
    
    
    bool left_arm_extended_x = (l_elbow.x > l_shoulder.x) && (l_wrist.x > l_elbow.x);
    bool right_arm_extended_x = (r_elbow.x < r_shoulder.x) && (r_wrist.x < r_elbow.x);

    // 2. 水平对齐判断 (Y轴):
    //    - 手腕、手肘和肩膀的Y坐标应该相近，允许有一定的容差
    //    - 容差可以基于肩宽来设定，使其对远近不敏感
    float y_alignment_threshold = std::max(50.0f, shoulder_width * 0.25f); // 允许25像素或25%肩宽的垂直偏差

    bool left_arm_aligned_y = (std::abs(l_wrist.y - l_shoulder.y) < y_alignment_threshold) &&
                              (std::abs(l_elbow.y - l_shoulder.y) < y_alignment_threshold);
                              
    bool right_arm_aligned_y = (std::abs(r_wrist.y - r_shoulder.y) < y_alignment_threshold) &&
                               (std::abs(r_elbow.y - r_shoulder.y) < y_alignment_threshold);

    // --- 调试日志 ---
    RCLCPP_INFO(main_node_->get_logger(),
                "Arms Extended Check: L_wrist(%.0f,%.0f) L_elbow(%.0f,%.0f) L_shoulder(%.0f,%.0f) | R_wrist(%.0f,%.0f) R_elbow(%.0f,%.0f) R_shoulder(%.0f,%.0f)",
                l_wrist.x, l_wrist.y, l_elbow.x, l_elbow.y, l_shoulder.x, l_shoulder.y,
                r_wrist.x, r_wrist.y, r_elbow.x, r_elbow.y, r_shoulder.x, r_shoulder.y);

    RCLCPP_INFO(main_node_->get_logger(),
                "Arms Extended Conditions L: X_Ext:%d, Y_Align:%d | R: X_Ext:%d, Y_Align:%d | SW:%.1f, Y_Thr:%.1f",
                left_arm_extended_x, left_arm_aligned_y, right_arm_extended_x, right_arm_aligned_y,
                shoulder_width, y_alignment_threshold);
    // --- 结束调试日志 ---

    // 只要有一只手臂满足条件，就可以认为是取消动作，这样更灵活
    if ((left_arm_extended_x && left_arm_aligned_y) || (right_arm_extended_x && right_arm_aligned_y)) {
        RCLCPP_WARN(main_node_->get_logger(), "!!! Arms Extended Pose Detected (Stop Tracking) !!!");
        return true;
    }
    
    return false;
}




void TrackingManager::Release() {
  RCLCPP_WARN(main_node_->get_logger(), "TrackingManager release");
  start_ = false;

  // 释放资源
  targets_sub_.reset();
  image_sub_.reset();
  param_node_.reset();
  robot_cmdvel_node_.reset();
  main_node_.reset();
}

std::shared_ptr<rclcpp::Node> TrackingManager::GetMainNode() {
  return main_node_;
}


void TrackingManager::UpdateTrackAngle() {
  if (track_info_.present_rect.empty()) return;

  cv::Point2f fit_center(track_cfg_.img_width / 2, track_cfg_.img_height);
  cv::Point2f start_pt(track_cfg_.img_width, track_cfg_.img_height);
  cv::Point2f end_pt(
      (track_info_.present_rect[0] + track_info_.present_rect[2]) / 2,
      (track_info_.present_rect[1] + track_info_.present_rect[3]) / 2);

  // 计算目标中心在图像中的X坐标
  float target_center_x_in_image = (track_info_.present_rect[0] + track_info_.present_rect[2]) / 2.0f;
  // 图像中心X坐标
  float image_center_x = track_cfg_.img_width / 2.0f;
  // 目标相对于图像中心的X偏差
  float x_deviation_from_center = target_center_x_in_image - image_center_x; // 正值表示偏右，负值表示偏左

  
 // 打印输入参数 (可选，但强烈推荐)
  RCLCPP_INFO(main_node_->get_logger(),
              "UpdateTrackAngle Inputs: fit_center(%.1f, %.1f), start_pt(%.1f, %.1f), end_pt(%.1f, %.1f)",
              fit_center.x, fit_center.y, start_pt.x, start_pt.y, end_pt.x, end_pt.y);

  // 调用 CalAngelOfTwoVector 函数 (假设 clock_wise 传 true)
  // 请确认你在这里调用的 clock_wise 参数是你期望的值
  float calculated_angle = CalAngelOfTwoVector(fit_center, start_pt, end_pt, true);

  // 打印计算出的原始角度
  RCLCPP_INFO(main_node_->get_logger(),
              "CalAngelOfTwoVector calculated angle (before abs/transformation): %.2f degrees", calculated_angle);

  // 将结果赋给 track_info_
  track_info_.angel_with_robot_ = calculated_angle;
  // 打印最终赋给 track_info_ 的值，用于后续逻辑 (RotateSwitch)
  RCLCPP_INFO(main_node_->get_logger(),
              "track_info_.angel_with_robot_ set to: %.2f degrees",
              track_info_.angel_with_robot_);

  {
    std::stringstream ss;
    ss << "frame_ts: " << track_info_.frame_ts
       << ", track_id: " << track_info_.track_id << ", angel_with_robot: "
       << std::abs(track_info_.angel_with_robot_ - 90)
       // << ", img_width_: " << img_width_ << ", img_height_: " << img_height_
       // << ", rect: " << track_info_.present_rect[0] << " " <<
       // track_info_.present_rect[1]
       // << " " << track_info_.present_rect[2]  << " " <<
       // track_info_.present_rect[3]
       << std::endl;
    RCLCPP_INFO(main_node_->get_logger(), // 改用 main_node_->get_logger()
                "UpdateTrackAngle: %s",
                ss.str().data());
  }

  auto transform_y_angle_to_x_radian =
      [this](float robot_y_negtive_angel) -> float {
    float robot_x_positive_angel = robot_y_negtive_angel;
    bool kps_is_clock_wise = true;
    if (robot_y_negtive_angel < 90) {
      kps_is_clock_wise = true;
      robot_x_positive_angel = 90 - robot_x_positive_angel;
    } else if (robot_y_negtive_angel > 90) {
      kps_is_clock_wise = false;
      robot_x_positive_angel = robot_x_positive_angel - 90;
    }
    float robot_x_positive_radian = robot_x_positive_angel * PI / 180.0;
    if (kps_is_clock_wise) {
      robot_x_positive_radian = (-1.0) * robot_x_positive_radian;
    }

    std::stringstream ss;
    ss << "robot robot_y_negtive_angel: " << robot_y_negtive_angel
       << ", robot_x_positive_angel: " << robot_x_positive_angel
       << ", robot_x_positive_radian: " << robot_x_positive_radian
       << ", kps_is_clock_wise: " << kps_is_clock_wise;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "%s", ss.str().c_str());
    return robot_x_positive_radian;
  };

  track_info_.robot_x_positive_radian_with_track_ =
      transform_y_angle_to_x_radian(track_info_.angel_with_robot_);
  track_info_.robot_y_negtive_radian_with_track_ =
      track_info_.angel_with_robot_ * PI / 180.0;
}

bool TrackingManager::RotateSwitch() {
  if (!last_cmdvel_is_cancel_ && 0 == last_cmdvel_type_) return true;

  if (track_info_.serial_lost_num > track_cfg_.track_serial_lost_num_thr) {
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "track_id: %d, serial_lost_num: %d, thr: %d, stop rotate!",
                track_info_.track_id,
                track_info_.serial_lost_num,
                track_cfg_.track_serial_lost_num_thr);
    return false;
  }

  // todo add lowpass strategy
  int activate_robot_rotate_thr = track_cfg_.activate_robot_rotate_thr;
  if (!track_info_.has_face_head) {
    activate_robot_rotate_thr = activate_robot_rotate_thr * 0.5;
  } else if (!last_cmdvel_is_cancel_ && 1 == last_cmdvel_type_) {
    activate_robot_rotate_thr = activate_robot_rotate_thr * 2;
  }
  if (std::abs(track_info_.angel_with_robot_ - 90) >=
      activate_robot_rotate_thr) {
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "RotateSwitchenable, activate_robot_rotate_thr: %d",
                activate_robot_rotate_thr);
    return true;
  }
  // >>>>>>> 在这里添加日志 <<<<<<<
  RCLCPP_INFO(main_node_->get_logger(), "RotateSwitch returning false. Angle diff: %.1f, Thr: %d", 
              std::abs(track_info_.angel_with_robot_ - 90), activate_robot_rotate_thr);

  return false;
}

void TrackingManager::DoRotateMove() {
  if (!robot_cmdvel_node_) return;
  RCLCPP_INFO(main_node_->get_logger(), "DoRotateMove called. Current TrackingStatus: %d", static_cast<int>(track_info_.tracking_sta));
  int direction = 1;
  float step = 0;
  auto twist = std::make_shared<Twist>();
  twist->linear.x = 0;
  twist->linear.y = 0;
  twist->linear.z = 0;
  twist->angular.x = 0;
  twist->angular.y = 0;
  twist->angular.z = 0;

  bool do_move = false;
  bool do_rotate = false;
  if (RotateSwitch()) {
    do_rotate = true;
    // transform angel to yaw shift and wise
    int yaw_shift = 0;
    bool rotate_clock_wise = true;
    if (track_info_.angel_with_robot_ <= 90) {
      yaw_shift = 90 - track_info_.angel_with_robot_;
      rotate_clock_wise = true;
    } else if (track_info_.angel_with_robot_ <= 180) {
      yaw_shift = track_info_.angel_with_robot_ - 90;
      rotate_clock_wise = false;
    } else if (track_info_.angel_with_robot_ <= 270) {
      yaw_shift = track_info_.angel_with_robot_ - 90;
      rotate_clock_wise = false;
    } else if (track_info_.angel_with_robot_ <= 360) {
      yaw_shift = 360 - track_info_.angel_with_robot_ + 90;
      rotate_clock_wise = true;
    }

    float rotate_step_ratio = 1.0f;
    if (yaw_shift > track_cfg_.activate_robot_rotate_thr * 2 ||
        !track_info_.has_face_head) {
      rotate_step_ratio = 2.0;
    }
    float rotate_step = track_cfg_.rotate_step * rotate_step_ratio;
    std::stringstream ss;
    ss << "frame_ts: " << track_info_.frame_ts << ", yaw_shift: " << yaw_shift
       << ", rotate_clock_wise: " << rotate_clock_wise
       << ", rotate_step_ratio: " << rotate_step_ratio
       << ", rotate_step: " << rotate_step;
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "rotate switch on: %s",
                ss.str().data());

    if (rotate_clock_wise) direction = 0;
    step = rotate_step;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "rotate direction: %d, step: %f",
                direction,
                step);
    int direct = 1;
    if (0 == direction) {
      direct = -1;
    }
    // 弧度＝度×π/180
    // twist->angular.z = direct * step * 3.14 / 180;
    twist->angular.z = direct * step;
  }

  
  if (TrackingSwitchWithVision()) {
    do_move = true;
    direction = track_info_.move_direction;
    step = track_info_.move_step;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "move switch on, direction: %d, step: %f",
                direction,
                step);
    if (0 == direction) {
      twist->linear.x += step;
    } else if (1 == direction) {
      twist->linear.x -= step;
    } else if (2 == direction) {
      twist->linear.y += step;
    } else if (3 == direction) {
      twist->linear.y -= step;
    }
  }

  if (do_move || do_rotate) {
    last_cmdvel_is_cancel_ = false;
    last_cmdvel_type_ = 2;
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "Do rotate move, ts sec: %llu, nanosec: %llu",
                track_info_.frame_ts_sec,
                track_info_.frame_ts_nanosec);
    robot_cmdvel_node_->RobotCtl(*twist);

    // if (do_rotate) {
    //   // 避免由于智能结果输出时间间隔不均匀导致的小车过度运动
    //   static int cmd_ms = 30;
    //   std::this_thread::sleep_for(std::chrono::milliseconds(cmd_ms));
    //   CancelMove();
    // }
  } else {
    CancelMove();
  }
}

void TrackingManager::CancelMove() {
  if (last_cmdvel_is_cancel_) return;
  if (robot_cmdvel_node_) {
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "cancel move");
    auto twist = std::make_shared<Twist>();
    twist->linear.x = 0;
    twist->linear.y = 0;
    twist->linear.z = 0;
    twist->angular.x = 0;
    twist->angular.y = 0;
    twist->angular.z = 0;
    robot_cmdvel_node_->RobotCtl(*twist);
  }

  last_cmdvel_is_cancel_ = true;
  last_cmdvel_type_ = -1;
  last_move_step_ratio_ = 1.0;
}

bool TrackingManager::TrackingSwitchWithVision() {
  if (track_info_.present_rect.empty()) {
    RCLCPP_INFO(main_node_->get_logger(), "TrackingSwitchWithVision returning false: present_rect empty");
    return false;
  }

  // // todo 20211230 如果没有face/head，认为距离很近，不需要move
  // if (!track_info_.has_face_head) {
    
  //   RCLCPP_INFO(main_node_->get_logger(), "TrackingSwitchWithVision returning false: no face/head detected");
    
  //   RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
  //               "track has no face and head, too close to robot");
  //   return false;
  // }

  if (track_info_.present_rect[1] < track_cfg_.stop_robot_move_to_top_thr) {
    // 距离上边界很近，不需要move。如果移动可能拍不到face/head，导致检测不到body
    RCLCPP_INFO(main_node_->get_logger(), "TrackingSwitchWithVision returning false: too close to top. Rect[1]: %d, Thr: %d",
                track_info_.present_rect[1], track_cfg_.stop_robot_move_to_top_thr);
    return false;
  }

  // 如果body rect宽度超过画面宽度一定比例，认为距离很近，不需要move
  int body_rect_width =
      track_info_.present_rect[2] - track_info_.present_rect[0];
  if (body_rect_width >=
      track_cfg_.img_width * track_cfg_.stop_move_rect_width_ratio_thr) {
    RCLCPP_INFO(
        rclcpp::get_logger("TrackingManager"),
        "track width: %d, exceeds %f of img_width_: %d, too close to robot",
        body_rect_width,
        track_cfg_.stop_move_rect_width_ratio_thr,
        track_cfg_.img_width);
    RCLCPP_INFO(main_node_->get_logger(), "TrackingSwitchWithVision returning false: too close to person. Rect width: %d, Thr: %.1f",
                body_rect_width, track_cfg_.img_width * track_cfg_.stop_move_rect_width_ratio_thr);
    return false;
  }

  // 根据body rect宽度，计算move step，宽度越小，step越大
  track_info_.move_step = track_cfg_.move_step;
  float move_step_ratio = 1.0;
  int body_rect_to_top = track_info_.present_rect[1];
  // 只有当robot和target之间的角度较小时才可以加速move，否则可能导致robot撞倒障碍物或者跟丢target
  if (abs(track_info_.robot_x_positive_radian_with_track_ * 180 / PI) <
      track_cfg_.activate_robot_rotate_thr) {
    if (body_rect_to_top > track_cfg_.img_height * 0.5) {
      move_step_ratio = 4;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.45) {
      move_step_ratio = 3.5;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.4) {
      move_step_ratio = 3.0;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.35) {
      move_step_ratio = 2.5;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.3) {
      move_step_ratio = 2.0;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.25) {
      move_step_ratio = 1.5;
    } else if (body_rect_to_top > track_cfg_.img_height * 0.2) {
      move_step_ratio = 1.2;
    }
  }

  if (move_step_ratio > (last_move_step_ratio_ + 0.5)) {
    // 限制速度，避免突然速度
    move_step_ratio = (last_move_step_ratio_ + 0.5);
    last_move_step_ratio_ = move_step_ratio;
  }

  track_info_.move_step = move_step_ratio * track_cfg_.move_step;

  // 只根据body rect宽度判断是否需要move 20220119
  int to_top_thr =
      track_cfg_.activate_robot_move_thr;  // track_cfg_.img_height * 0.1;
  if (body_rect_width <
          track_cfg_.img_width * track_cfg_.start_move_rect_width_ratio_thr &&
      body_rect_to_top > to_top_thr) {
    std::stringstream ss;
    ss << "Do move! body_rect_width: " << body_rect_width << ", thr: "
       << track_cfg_.img_width * track_cfg_.stop_move_rect_width_ratio_thr
       << ", move_step_ratio: " << move_step_ratio
       << ", body_rect_to_top: " << body_rect_to_top
       << ", img_height: " << track_cfg_.img_height
       << ", move_step: " << track_info_.move_step;
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"), "%s", ss.str().c_str());
    RCLCPP_INFO(main_node_->get_logger(), "TrackingSwitchWithVision returning true: conditions met.");    
    return true;
  } else {
    std::stringstream ss;
    ss << "Do not move! body_rect_width: " << body_rect_width << ", thr: "
       << track_cfg_.img_width * track_cfg_.stop_move_rect_width_ratio_thr
       << ", move_step_ratio: " << move_step_ratio
       << ", body_rect_to_top: " << body_rect_to_top
       << ", img_height: " << track_cfg_.img_height
       << ", move_step: " << track_info_.move_step;
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "%s", ss.str().c_str());
    RCLCPP_INFO(main_node_->get_logger(), "TrackingSwitchWithVision returning false: conditions not met. body_rect_width: %d < %.1f || body_rect_to_top: %d <= %d",
                body_rect_width, track_cfg_.img_width * track_cfg_.start_move_rect_width_ratio_thr,
                body_rect_to_top, to_top_thr);
  }
  return false;
}

void TrackingManager::ProcessSmart(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg,
    const cv::Mat& image) {
  if (!msg || !rclcpp::ok() || image.empty()) {
    return;
  }

  uint64_t frame_ts = (msg->header.stamp.sec % 1000) * 1000 +
                      msg->header.stamp.nanosec / 1000 / 1000;

  track_info_.frame_ts_sec = msg->header.stamp.sec;
  track_info_.frame_ts_nanosec = msg->header.stamp.nanosec;
  track_info_.frame_ts = frame_ts;

  // =========================================================================
  // 核心逻辑重构
  // =========================================================================

  if (track_info_.tracking_sta == TrackingStatus::TRACKING) {
    // --- 状态为 TRACKING: 寻找取消姿态，并用特征匹配来寻找目标 ---
    bool find_track_by_feature = false;
    bool cancel_pose_detected = false;
    
    // 优先检查当前追踪的目标是否做出了取消姿态
    for (const auto &target : msg->targets) {
        if (target.track_id == track_info_.track_id) {
            for (const auto& kps_group : target.points) {
                if (kps_group.type == "body_kps") {
                     if (is_arms_extended_pose(kps_group)) {
                        cancel_pose_detected = true;
                    }
                    break;
                }
            }
            break;
        }
    }
    
    if (cancel_pose_detected) {
        RCLCPP_WARN(main_node_->get_logger(), "Cancel Pose Detected! Resetting state.");
        reset_tracking_info();  // 重置所有追踪信息
        track_info_.tracking_sta = TrackingStatus::INITING; // 切换到初始状态
        CancelMove(); // 停止机器人运动
        return; // 直接返回，不再处理本帧后续逻辑
    }
    
    // 遍历当前帧检测到的所有人体目标
    for (const auto &target : msg->targets) {
      // 我们只关心人体
      if (track_cfg_.track_body) {
        bool tar_has_body = false;
        std::vector<int> body_rect;
        for (const auto &roi : target.rois) {
          if ("body" == roi.type) {
            tar_has_body = true;
            body_rect = {roi.rect.x_offset, roi.rect.y_offset,
                         roi.rect.x_offset + roi.rect.width, roi.rect.y_offset + roi.rect.height};
            break;
          }
        }
        if (!tar_has_body) continue;

        // 对这个候选人提取特征
        ColorFeature candidate_feature = extract_feature(image, body_rect);

        // 与我们锁定的目标特征进行匹配
        if (track_info_.feature_is_set && match_feature(track_info_.target_feature, candidate_feature)) {
          // 匹配成功！这个人就是我们的目标！
          find_track_by_feature = true;
          
          RCLCPP_INFO(main_node_->get_logger(),
                      "Feature match success! Old Track ID: %lu -> New Track ID: %lu",
                      track_info_.track_id, target.track_id);
          
          // 更新追踪信息为当前匹配到的人
          track_info_.track_id = target.track_id;
          track_info_.serial_lost_num = 0;
          track_info_.last_rect = track_info_.present_rect;
          track_info_.present_rect = body_rect;

          // (可选) 平滑更新特征，以适应光照的缓慢变化
          track_info_.target_feature.r = 0.9 * track_info_.target_feature.r + 0.1 * candidate_feature.r;
          track_info_.target_feature.g = 0.9 * track_info_.target_feature.g + 0.1 * candidate_feature.g;
          track_info_.target_feature.b = 0.9 * track_info_.target_feature.b + 0.1 * candidate_feature.b;

          break; // 找到了，就不用再继续遍历了
        }
      }
    }

    if (!find_track_by_feature) {
        // 如果这一帧通过颜色特征没有匹配到任何人
        RCLCPP_WARN(main_node_->get_logger(), "Feature match failed. Trying to find by last known ID.");
        
        // 尝试用上一次的 track_id 找一下
        bool find_by_last_id = false;
        for (const auto &target : msg->targets) {
            if (target.track_id == track_info_.track_id) {
                // 找到了上一次的ID，即使颜色暂时不匹配，我们也暂时相信它
                // 这可能是由于光照突变
                RCLCPP_INFO(main_node_->get_logger(), "Found target by last ID %lu, color mismatch. Temporarily trusting.", track_info_.track_id);
                
                // 更新位置，但【不要】更新颜色特征
                // 提取body_rect
                std::vector<int> body_rect;
                // ... (从 target.rois 获取 body_rect 的代码) ...
                if (!body_rect.empty()){
                    track_info_.present_rect = body_rect;
                    track_info_.serial_lost_num = 0; // 重置丢失计数
                    find_by_last_id = true;
                    find_track_by_feature = true; // 把它也设为true，跳过后续的丢失处理
                }
                break;
            }
        }
    
        if (!find_by_last_id) {
            // 如果连上一次的ID都找不到了，这才算是真正的“丢失”一帧
            RCLCPP_WARN(main_node_->get_logger(), "Completely lost target for this frame. Lost count: %d", track_info_.serial_lost_num + 1);
            track_info_.serial_lost_num++;
            // 让机器人原地等待，而不是立即停止所有运动
            track_info_.present_rect.clear(); // 清空当前位置，让DoRotateMove知道要原地等待
            if (track_info_.serial_lost_num >= track_cfg_.track_serial_lost_num_thr) {
                RCLCPP_ERROR(main_node_->get_logger(), "Track lost (timeout). Resetting state.");
                reset_tracking_info();
                track_info_.tracking_sta = TrackingStatus::LOST;
                CancelMove(); // 彻底丢失后才停止
            }
        }
    }    
    


  } else { // 状态为 INITING 或 LOST: 寻找“叉腰”姿态来启动追踪
    if (msg->targets.empty()) {
      return;
    }

    // 1. 寻找做出“叉腰”姿态的人
    for (const auto &target : msg->targets) {
      bool tar_has_body = false;
      std::vector<int> body_rect;
      for (const auto &roi : target.rois) {
          if ("body" == roi.type) {
              tar_has_body = true;
              body_rect = {roi.rect.x_offset, roi.rect.y_offset, 
                           roi.rect.x_offset + roi.rect.width, roi.rect.y_offset + roi.rect.height};
              break;
          }
      }
      if (!tar_has_body) continue;


      bool has_kps = false;
      for (const auto& kps_group : target.points) {
          if (kps_group.type == "body_kps") {
              has_kps = true;
              break;
          }
      }
      RCLCPP_INFO(main_node_->get_logger(), "Target ID %lu, has body_kps: %d", target.track_id, has_kps);
   
      

      // 检查这个人是否有关节点并且姿态是叉腰
      for (const auto& kps_group : target.points) {
          if (kps_group.type == "body_kps") {
              if (is_akimbo_pose(kps_group)) {
                  // 找到了！这个人做出了启动姿态！
                  RCLCPP_WARN(main_node_->get_logger(), "Akimbo wakeup pose detected on body with track ID: %lu", target.track_id);
                  
                  // 重置并设置追踪信息
                  reset_tracking_info(); // 清空旧数据
                  track_info_.tracking_sta = TrackingStatus::TRACKING;
                  track_info_.track_id = target.track_id;
                  track_info_.present_rect = body_rect;

                  // === 核心：提取并缓存特征 ===
                  track_info_.target_feature = extract_feature(image, body_rect);
                  track_info_.feature_is_set = true;
                  
                  RCLCPP_WARN(main_node_->get_logger(),
                              "Target locked! Feature RGB: [R=%.1f, G=%.1f, B=%.1f]",
                              track_info_.target_feature.r,
                              track_info_.target_feature.g,
                              track_info_.target_feature.b);
                  
                  // 找到了就直接返回，避免一帧内被其他人干扰
                  return; 
              }
              break; // 每个人只检查一组身体关键点
          }
      }
    }
  }

  // 这部分检查 has_face_head 的逻辑可以暂时保留，因为它与运动策略相关
  // [注意] 这部分逻辑可能需要调整，因为它依赖于旧的 find_track 结果
  if (TrackingStatus::TRACKING == track_info_.tracking_sta) {
    track_info_.has_face_head = false;
    for (const auto &target : msg->targets) {
      // 只检查当前追踪到的目标
      if (target.track_id == track_info_.track_id) {
          auto match_rect = [](std::vector<int> body_rect,
                             std::vector<int> face_head_rect) -> bool {
            if (body_rect.empty() || face_head_rect.empty()) return false;
            return (body_rect[0] <= face_head_rect[0] &&
                    body_rect[1] <= face_head_rect[1] &&
                    body_rect[2] >= face_head_rect[2] &&
                    body_rect[3] >= face_head_rect[3]);
          };

          for (const auto &roi : target.rois) {
              if (roi.type == "face" || roi.type == "head") {
                  std::vector<int> face_head_rect = {roi.rect.x_offset, roi.rect.y_offset,
                                                     roi.rect.x_offset + roi.rect.width, roi.rect.y_offset + roi.rect.height};
                  if (match_rect(track_info_.present_rect, face_head_rect)) {
                      track_info_.has_face_head = true;
                      break; 
                  }
              }
          }
          if (track_info_.has_face_head) break;
      }
    }

    if (track_info_.has_face_head) {
      track_info_.serial_lost_face_head_num = 0;
    } else {
      track_info_.serial_lost_face_head_num++;
    }

    RCLCPP_INFO_STREAM(
        rclcpp::get_logger("TrackingManager"),
        "serial_lost_face_head_num: " << track_info_.serial_lost_face_head_num);
  }
}



void TrackingManager::RunTrackLostProtectionStrategy() {
  if (track_info_.serial_lost_face_head_num >= 20) {
    // start protection strategy
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "track lost warnning! serial_lost_face_head_num: %d",
                track_info_.serial_lost_face_head_num);
    // cancel move goal
    CancelMove();
  }
  return;

  int angle_with_track = std::abs(track_info_.angel_with_robot_ - 90);
  // start protection strategy
  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "The angle between robot and track is %d, limit is %d",
              angle_with_track,
              track_cfg_.track_lost_protection_angel_thr);

  if (angle_with_track < track_cfg_.track_lost_protection_angel_thr) {
    return;
  }

  // start protection strategy
  RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
              "track lost warnning! The angle between robot and track is %d "
              "exceeds limit %d, track lost protection strategy is activated!",
              std::abs(track_info_.angel_with_robot_ - 90),
              track_cfg_.track_lost_protection_angel_thr);
  // cancel move goal
  CancelMove();
}

void TrackingManager::RunOverMovingProtectionStrategy() {
  if (!last_cmdvel_is_cancel_ && 0 == last_cmdvel_type_ &&
      (std::abs(track_info_.angel_with_robot_ - 90) <
       track_cfg_.track_overmoving_protection_angel_thr)) {
    // start protection strategy
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "frame_ts %llu, robot over moving warnning! The angle between "
                "robot and track is %d, less than limit %d, robot overmoving "
                "strategy is activated!",
                track_info_.frame_ts,
                std::abs(track_info_.angel_with_robot_ - 90),
                track_cfg_.track_overmoving_protection_angel_thr);
    CancelMove();
  }
}


void TrackingManager::TrackingWithoutNavStrategy(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg,
    const cv::Mat& image) { // <-- 添加 const cv::Mat& image
  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "Run TrackingWithoutNav strategy");

  if (!last_frame_done_) {
    RCLCPP_WARN(rclcpp::get_logger("TrackingManager"),
                "last frame is not done");
    return;
  }

  // 1. update track info, 将图像传递给 ProcessSmart
  ProcessSmart(msg, image); // <-- 传递 image

  last_frame_done_ = false;

  if (TrackingStatus::INITING == track_info_.tracking_sta) {
    CancelMove();
    last_frame_done_ = true;
    return;
  }

  uint64_t frame_ts = (msg->header.stamp.sec % 1000) * 1000 +
                      msg->header.stamp.nanosec / 1000 / 1000;
  {
    std::stringstream ss;
    ss << "track_id: " << track_info_.track_id
       << ", frame_ts: " << track_info_.frame_ts
       << ", tracking_sta(0:INITING, 1:TRACKING, 2:LOST): "
       << static_cast<int>(track_info_.tracking_sta)
       << ", gesture: " << track_info_.gesture << ", y pixel to robot: "
       << (track_info_.present_rect.empty()
               ? -1
               : (track_cfg_.img_height - track_info_.present_rect[3]));
    if (!track_info_.present_rect.empty()) {
      ss << ", present_rect: " << track_info_.present_rect[0] << " "
         << track_info_.present_rect[1] << " " << track_info_.present_rect[2]
         << " " << track_info_.present_rect[3];
    }
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "%s", ss.str().data());
  }

  if (TrackingStatus::TRACKING != track_info_.tracking_sta) {
    RCLCPP_DEBUG(rclcpp::get_logger("TrackingManager"), "track is lost");
    CancelMove();
    track_info_.tracking_sta = TrackingStatus::INITING;
    last_frame_done_ = true;
    return;
  }

  if (frame_ts != track_info_.frame_ts) {
    // 如果当前帧中没有被跟随的track信息，track info只更新lost
    // info，当前帧不做跟随
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "update smart fail! msg frame_ts %llu, track frame_ts %llu",
                frame_ts,
                track_info_.frame_ts);
    CancelMove();
    last_frame_done_ = true;

    return;
  }

  // 2. cal the angle of robot and track
  UpdateTrackAngle();

  if (!last_cmdvel_is_cancel_) {
    RunOverMovingProtectionStrategy();
  }

  RunTrackLostProtectionStrategy();
  DoRotateMove();
  last_frame_done_ = true;
  return;
}

// 在函数定义处修改签名
void TrackingManager::RunTrackingStrategy(
    const ai_msgs::msg::PerceptionTargets::ConstSharedPtr &msg,
    const cv::Mat& image) { // <-- 添加 const cv::Mat& image
  std::unique_lock<std::mutex> lg(robot_strategy_mtx_);

  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"), "Run TrackingStrategy");
  {
    static auto last_tracking_ts = TimeHelper::GetCurrentTimestampMillSec();
    auto present_tracking_ts = TimeHelper::GetCurrentTimestampMillSec();
    RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
                "Run TrackingStrategy time ms diff: %llu",
                present_tracking_ts - last_tracking_ts);
    last_tracking_ts = present_tracking_ts;
  }

  auto start_tracking_ts = TimeHelper::GetCurrentTimestampMillSec();

  // 将图像传递给下一层
  TrackingWithoutNavStrategy(msg, image); // <-- 传递 image

  auto present_tracking_ts = TimeHelper::GetCurrentTimestampMillSec();
  RCLCPP_INFO(rclcpp::get_logger("TrackingManager"),
              "Run TrackingStrategy time ms cost: %llu",
              present_tracking_ts - start_tracking_ts);

  return;
}


/**
 * @brief 图像消息的回调函数，只负责缓存最新的图像
 */
void TrackingManager::ImageCallback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr& image_msg){
  RCLCPP_INFO_ONCE(main_node_->get_logger(), ">>>>>> ImageCallback is being triggered! <<<<<<");   
  std::lock_guard<std::mutex> lock(image_buffer_mutex_);
    latest_image_msg_ = image_msg;
}

/**
 * @brief AI检测结果的回调函数，这是我们的主处理入口
 */
void TrackingManager::AiMsgCallback(const ai_msgs::msg::PerceptionTargets::ConstSharedPtr& targets_msg) {
    RCLCPP_INFO_ONCE(main_node_->get_logger(), ">>>>>> AiMsgCallback is being triggered! <<<<<<");
    if (!rclcpp::ok() || !start_) {
        return;
    }

    // 从缓存中获取最新的图像
    sensor_msgs::msg::CompressedImage::ConstSharedPtr current_image_msg; // <-- 修改类型
    {
        std::lock_guard<std::mutex> lock(image_buffer_mutex_);
        if (!latest_image_msg_) {
            RCLCPP_WARN_ONCE(main_node_->get_logger(), "No image received yet.");
            return;
        }
        current_image_msg = latest_image_msg_;
    }

    // 将 CompressedImage 消息转换为 cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    try {
        // cv_bridge 可以直接处理 CompressedImage
        cv_ptr = cv_bridge::toCvCopy(current_image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(main_node_->get_logger(), "cv_bridge exception from compressed image: %s", e.what());
        return;
    }

    // 现在我们有了检测结果和（近似）对应的图像，调用主处理逻辑
    RunTrackingStrategy(targets_msg, cv_ptr->image);
}

std::vector<std::shared_ptr<rclcpp::Node>> TrackingManager::GetNodes() {
  std::vector<std::shared_ptr<rclcpp::Node>> node_ptrs;
  node_ptrs.push_back(param_node_);
  node_ptrs.push_back(robot_cmdvel_node_);
  return node_ptrs;
}

const TrackCfg &TrackingManager::GetTrackCfg() const { return track_cfg_; }
