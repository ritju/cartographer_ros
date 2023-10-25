/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/msg_conversion.h"
#include "gflags/gflags.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");

DEFINE_string(
    relocate_configuration_filename, "",
    "if localization score is too low, then start a new trajectory with this conifg file.");

namespace cartographer_ros {
namespace {

NodeOptions node_options;
TrajectoryOptions trajectory_options;
std::shared_ptr<Node> node;
rclcpp::Node::SharedPtr cartographer_node;
geometry_msgs::msg::PoseWithCovariance latest_pose;

bool relocalization_state = false; //是否在重定位状态，用以判断是否进入或推出重定位模式
bool transform_status = false; //是否接收到最新位姿，并在每次处理完数据后置为false
geometry_msgs::msg::Twist cmd_vel; //机器人在停止运行后，读取的位姿数据更加准确，作为新轨迹的初始位姿也更加准确
float localizaiton_score;
rclcpp::Time score_time, pose_time, initial_pose_time;
bool update_score_time = true; // localization_score没有时间戳，因此需要单独更新时间

void InitPose_Callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  std::tie(node_options, trajectory_options)
   = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
  node->FinishAllTrajectories();
  *trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose()
   = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(msg->pose.pose));
  // latest_pose = msg;
  if(FLAGS_start_trajectory_with_default_topics)
  {
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }
  /*
  重置重定位标识，防止指定初始位姿不准后，机器人无法进入冲定位模式
  */
  relocalization_state = false;
  initial_pose_time = msg->header.stamp;
  
}


void LocalizationScore_Callback(const std_msgs::msg::Float32 localization_score_msg)
{
  if ((pose_time - initial_pose_time).nanoseconds() > 5e+8)
  {
    auto now = rclcpp::Clock();
    localizaiton_score = localization_score_msg.data;
    if (localization_score_msg.data < 0.5 && (!relocalization_state))
    {
      
      std::tie(node_options, trajectory_options)
      = LoadOptions(FLAGS_configuration_directory, FLAGS_relocate_configuration_filename);
      
      node->FinishAllTrajectories();
      if (transform_status)
      {
        *trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose()
              = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(latest_pose.pose));
        transform_status = false;
      }
        if(FLAGS_start_trajectory_with_default_topics)
      {
        node->StartTrajectoryWithDefaultTopics(trajectory_options);
      }
      relocalization_state = true;    
      RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("cartographer_node"), now, 1000, "In relocating , relocalization_state: " << relocalization_state << "localization_score: " << localization_score_msg.data);
    }
    else if (localization_score_msg.data > 0.8 && relocalization_state && !cmd_vel.angular.z && !cmd_vel.linear.x)
    {
      if (update_score_time)
      {
        score_time = cartographer_node->get_clock()->now();
        update_score_time = false;
      }
      if ((pose_time - score_time).nanoseconds() > 1e+9)
      {
        std::tie(node_options, trajectory_options)
        = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
        node->FinishAllTrajectories();
        if (transform_status)
        {

          // RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("cartographer_node"), now, 1000, "set latest_pose.x: " << latest_pose.pose.position.x << ", set latest_pose.y: " << latest_pose.pose.position.y);

          *trajectory_options.trajectory_builder_options.mutable_initial_trajectory_pose()->mutable_relative_pose()
                = cartographer::transform::ToProto(cartographer_ros::ToRigid3d(latest_pose.pose));
          transform_status = false;
        }
        if(FLAGS_start_trajectory_with_default_topics)
        {
          node->StartTrajectoryWithDefaultTopics(trajectory_options);
        }
        relocalization_state = false;
        update_score_time = true;
        RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("cartographer_node"), now, 1000, "Localization good ! " << "localizaiton_score.data: " << localization_score_msg.data);
        }
      // RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("cartographer_node"), now, 1000, "Localization good outside!");
      // RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("cartographer_node"), now, 1000, "pose_time - score_time: " << (pose_time - score_time).nanoseconds());
      
      
      
    }
      // RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("cartographer_node"), now, 1000, "relocalization_state: " << relocalization_state << ", update_score_time: " << update_score_time << ", transform_status: " << transform_status);
  }

}


void PoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped &msg)
{
  auto now = rclcpp::Clock();
  latest_pose.pose = msg.pose.pose;
  pose_time = msg.header.stamp;
  transform_status = true;
  RCLCPP_INFO_STREAM_THROTTLE(rclcpp::get_logger("cartographer_node"), now, 1000, "sub latest_pose.x: " << latest_pose.pose.position.x << ", sub latest_pose.y: " << latest_pose.pose.position.y);


}
void CmdVelCallback(const geometry_msgs::msg::Twist &msg)
{
  cmd_vel = msg;
}

void Run() {
  cartographer_node = rclcpp::Node::make_shared("cartographer_node");
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  score_time = cartographer_node->get_clock()->now();
  pose_time = cartographer_node->get_clock()->now();
  initial_pose_time = cartographer_node->get_clock()->now();
  
  std::shared_ptr<tf2_ros::Buffer> tf_buffer =

      std::make_shared<tf2_ros::Buffer>(
        cartographer_node->get_clock(),
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
        cartographer_node);

  std::shared_ptr<tf2_ros::TransformListener> tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_callback =
      std::make_shared<tf2_ros::Buffer>(
        cartographer_node->get_clock(),
        tf2::durationFromSec(kTfBufferCacheTimeInSeconds));

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_callback =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_callback);

  std::tie(node_options, trajectory_options) =
      LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

  auto map_builder =
    cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
  node = std::make_shared<cartographer_ros::Node>(
    node_options, std::move(map_builder), tf_buffer, cartographer_node,
    FLAGS_collect_metrics);

  if (!FLAGS_load_state_filename.empty()) {
    node->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
  }

  if (FLAGS_start_trajectory_with_default_topics) {
    node->StartTrajectoryWithDefaultTopics(trajectory_options);
  }
  auto sub_opt = rclcpp::SubscriptionOptions();
  auto callback_group = cartographer_node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sub_opt.callback_group = callback_group;
  auto InitPose_sub = cartographer_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, InitPose_Callback, sub_opt);
  auto Localization_sub = cartographer_node->create_subscription<std_msgs::msg::Float32>("/localization_score", 1, LocalizationScore_Callback, sub_opt);
  auto Pose_sub = cartographer_node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/base_link_pose", 1, PoseCallback, sub_opt);
  auto Cmd_vel_sub = cartographer_node->create_subscription<geometry_msgs::msg::Twist>("/md_vel", 1, CmdVelCallback, sub_opt);
  rclcpp::spin(cartographer_node);

  node->FinishAllTrajectories();
  node->RunFinalOptimization();

  if (!FLAGS_save_state_filename.empty()) {
    node->SerializeState(FLAGS_save_state_filename,
                        true /* include_unfinished_submaps */);
  }
}

}  // namespace
}  // namespace cartographer_ros

int main(int argc, char** argv) {
  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  CHECK(!FLAGS_configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!FLAGS_configuration_basename.empty())
      << "-configuration_basename is missing.";

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  cartographer_ros::Run();
  ::rclcpp::shutdown();
}