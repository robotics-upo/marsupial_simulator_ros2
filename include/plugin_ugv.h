#ifndef PLUGIN_UGV_HH
#define PLUGIN_UGV_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace gazebo
{
  class UGVSimpleController : public ModelPlugin
  {
  public:
    UGVSimpleController();
    ~UGVSimpleController() override;
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void Update();

  private:
    physics::WorldPtr world;
    physics::LinkPtr link;
    std::string link_name_;
    std::string model_name_;
    std::string gt_topic_;

    ignition::math::Pose3d pose;

    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_gt_pose_;
    rclcpp::Node::SharedPtr node_handle_;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;

    event::ConnectionPtr updateConnection;
  };
}

#endif // PLUGIN_UGV_HH
