#include "plugin_ugv.h"

namespace gazebo {

UGVSimpleController::UGVSimpleController() {}
UGVSimpleController::~UGVSimpleController() { this->updateConnection.reset(); }

void UGVSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!rclcpp::ok())
  {
    RCLCPP_FATAL(rclcpp::get_logger("UGVSimpleController"), "A ROS node for Gazebo has not been initialized, unable to load plugin. Load the Gazebo system plugin 'libgazebo_ros_init.so' in the gazebo_ros package");
    return;
  }

  RCLCPP_INFO(rclcpp::get_logger("UGVSimpleController"), "The UGV plugin is loading!");

  model_name_ = _model->GetName();
  gt_topic_ = "ugv_gt_pose";

  link = _model->GetLink();
  link_name_ = link->GetName();
  if (!link)
  {
    RCLCPP_FATAL(rclcpp::get_logger("UGVSimpleController"), "gazebo_ros_baro plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  node_handle_ = std::make_shared<rclcpp::Node>("ugv_control", model_name_);
  executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  pub_gt_pose_ = node_handle_->create_publisher<geometry_msgs::msg::Pose>(gt_topic_, 10);
  RCLCPP_INFO(rclcpp::get_logger("UGVSimpleController"), "Advertising the ground truth pose topic on: %s !", gt_topic_.c_str());

  updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&UGVSimpleController::Update, this));

  RCLCPP_INFO(rclcpp::get_logger("UGVSimpleController"), "The UGV plugin finished loading!");
}

void UGVSimpleController::Update()
{
  // Spin the ROS 2 executor to process callbacks
  executor_->spin_some(std::chrono::milliseconds(100));

  // Get current pose
  pose = link->WorldPose();

  // Publish ground truth pose
  geometry_msgs::msg::Pose gt_pose;
  gt_pose.position.x = pose.Pos().X();
  gt_pose.position.y = pose.Pos().Y();
  gt_pose.position.z = pose.Pos().Z();
  gt_pose.orientation.w = pose.Rot().W();
  gt_pose.orientation.x = pose.Rot().X();
  gt_pose.orientation.y = pose.Rot().Y();
  gt_pose.orientation.z = pose.Rot().Z();

  pub_gt_pose_->publish(gt_pose);

//   RCLCPP_INFO(rclcpp::get_logger("UGVSimpleController"), "Published UGV position: [%.2f, %.2f, %.2f]",
//               gt_pose.position.x, gt_pose.position.y, gt_pose.position.z);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(UGVSimpleController)
} // namespace gazebo
