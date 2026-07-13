#ifndef PLUGIN_UGV_HH
#define PLUGIN_UGV_HH

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
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
    physics::LinkPtr link;
    std::string link_name_;
    std::string gt_topic_;

    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_gt_pose_;

    // Publica a ~100 Hz en lugar de 1 kHz: salta 9 de cada 10 pasos de física
    unsigned int step_counter_{0};

    event::ConnectionPtr updateConnection;
  };
}

#endif // PLUGIN_UGV_HH
