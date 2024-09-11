#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <chrono>
#include <thread>
#include <atomic>

namespace gazebo
{
  class TetherPositionPublisher : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr sdf) override
    {
      // Verifica que el nodo ROS 2 esté inicializado
      if (!rclcpp::ok())
      {
        RCLCPP_FATAL(rclcpp::get_logger("TetherPositionPublisher"), "ROS node not initialized, failed to load plugin.");
        return;
      }

      this->model = _model;
      this->ros_node_ = gazebo_ros::Node::Get(sdf);
      this->publisher = this->ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("tether_positions", 10);

      // Subscripciones a los topics para UGV y UAV
      this->target_position_uav_subscriber = this->ros_node_->create_subscription<geometry_msgs::msg::Pose>(
          "/target_position_uav", 10, std::bind(&TetherPositionPublisher::onTargetUAVPoseChanged, this, std::placeholders::_1));

      this->target_position_ugv_subscriber = this->ros_node_->create_subscription<geometry_msgs::msg::Pose>(
          "/target_position_ugv", 10, std::bind(&TetherPositionPublisher::onTargetUGVPoseChanged, this, std::placeholders::_1));

      this->publishing.store(false);

      RCLCPP_INFO(this->ros_node_->get_logger(), "TetherPositionPublisher plugin loaded.");
    }

  private:
    void onTargetUAVPoseChanged(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
      if (hasPoseChanged(msg, last_uav_pose))
      {
        last_uav_pose = *msg;
        this->triggerPublishing();
      }
    }

    void onTargetUGVPoseChanged(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
      if (hasPoseChanged(msg, last_ugv_pose))
      {
        last_ugv_pose = *msg;
        this->triggerPublishing();
      }
    }

    bool hasPoseChanged(const geometry_msgs::msg::Pose::SharedPtr new_pose, const geometry_msgs::msg::Pose &last_pose)
    {
      return !(new_pose->position.x == last_pose.position.x &&
               new_pose->position.y == last_pose.position.y &&
               new_pose->position.z == last_pose.position.z &&
               new_pose->orientation.x == last_pose.orientation.x &&
               new_pose->orientation.y == last_pose.orientation.y &&
               new_pose->orientation.z == last_pose.orientation.z &&
               new_pose->orientation.w == last_pose.orientation.w);
    }

    void triggerPublishing()
    {
      // Si ya se está publicando, no hacer nada
      if (this->publishing.exchange(true))
      {
        return;
      }

      auto links = this->model->GetLinks();
      std::sort(links.begin(), links.end(), [](const physics::LinkPtr &a, const physics::LinkPtr &b) {
        return a->GetName() < b->GetName();
      });

      auto first_publish_time = this->ros_node_->now();

      for (size_t i = 0; i < links.size(); ++i)
      {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = first_publish_time;
        msg.header.frame_id = links[i]->GetName();
        msg.pose.position.x = links[i]->WorldPose().Pos().X();
        msg.pose.position.y = links[i]->WorldPose().Pos().Y();
        msg.pose.position.z = links[i]->WorldPose().Pos().Z();
        msg.pose.orientation.x = links[i]->WorldPose().Rot().X();
        msg.pose.orientation.y = links[i]->WorldPose().Rot().Y();
        msg.pose.orientation.z = links[i]->WorldPose().Rot().Z();
        msg.pose.orientation.w = links[i]->WorldPose().Rot().W();

        this->publisher->publish(msg);

        // Espera 1 ms antes de publicar el siguiente mensaje
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      // Marcar que la publicación ha terminado
      this->publishing.store(false);
    }

    physics::ModelPtr model;
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_position_uav_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_position_ugv_subscriber;
    std::atomic<bool> publishing; // Controla si ya se está publicando

    // Almacenar las últimas poses recibidas
    geometry_msgs::msg::Pose last_uav_pose;
    geometry_msgs::msg::Pose last_ugv_pose;
  };

  // Registra el plugin con Gazebo
  GZ_REGISTER_MODEL_PLUGIN(TetherPositionPublisher)
}
