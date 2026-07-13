#include "plugin_ugv.h"

namespace gazebo {

UGVSimpleController::UGVSimpleController() {}
UGVSimpleController::~UGVSimpleController() { this->updateConnection.reset(); }

void UGVSimpleController::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  ros_node_ = gazebo_ros::Node::Get(_sdf);

  gt_topic_ = "ugv_gt_pose";

  // GetLink() sin argumento devuelve el link base del modelo
  link = _model->GetLink();
  if (!link)
  {
    RCLCPP_FATAL(ros_node_->get_logger(),
      "UGVSimpleController: no se encontró el link base del modelo '%s'",
      _model->GetName().c_str());
    return;
  }
  link_name_ = link->GetName();

  pub_gt_pose_ = ros_node_->create_publisher<geometry_msgs::msg::Pose>(gt_topic_, 10);
  RCLCPP_INFO(ros_node_->get_logger(),
    "UGVSimpleController cargado — publicando ground-truth en /%s a ~100 Hz",
    gt_topic_.c_str());

  updateConnection = event::Events::ConnectWorldUpdateBegin(
      std::bind(&UGVSimpleController::Update, this));
}

void UGVSimpleController::Update()
{
  // Publicar a ~100 Hz: el physics step corre a 1 kHz (max_step_size=0.001)
  if (++step_counter_ % 10 != 0)
    return;

  const auto pose = link->WorldPose();

  geometry_msgs::msg::Pose gt_pose;
  gt_pose.position.x    = pose.Pos().X();
  gt_pose.position.y    = pose.Pos().Y();
  gt_pose.position.z    = pose.Pos().Z();
  gt_pose.orientation.w = pose.Rot().W();
  gt_pose.orientation.x = pose.Rot().X();
  gt_pose.orientation.y = pose.Rot().Y();
  gt_pose.orientation.z = pose.Rot().Z();

  pub_gt_pose_->publish(gt_pose);
}

GZ_REGISTER_MODEL_PLUGIN(UGVSimpleController)
} // namespace gazebo
