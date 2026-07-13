#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <atomic>
#include <climits>
#include <cmath>
#include <string>

namespace gazebo
{
  class TetherPositionPublisher : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr sdf) override
    {
      this->model    = _model;
      this->ros_node_ = gazebo_ros::Node::Get(sdf);

      // A3: un único PoseArray por evento en lugar de N PoseStamped + sleep
      this->publisher = this->ros_node_->create_publisher<geometry_msgs::msg::PoseArray>(
          "tether_positions", 10);

      this->target_position_uav_subscriber =
          this->ros_node_->create_subscription<geometry_msgs::msg::Pose>(
              "/target_position_uav", 10,
              std::bind(&TetherPositionPublisher::onTargetUAVPoseChanged,
                        this, std::placeholders::_1));

      this->target_position_ugv_subscriber =
          this->ros_node_->create_subscription<geometry_msgs::msg::Pose>(
              "/target_position_ugv", 10,
              std::bind(&TetherPositionPublisher::onTargetUGVPoseChanged,
                        this, std::placeholders::_1));

      this->publishing.store(false);
      this->pending_.store(false);

      RCLCPP_INFO(this->ros_node_->get_logger(),
                  "TetherPositionPublisher cargado — publica PoseArray en /tether_positions");
    }

  private:
    // A4: comparación con tolerancia epsilon en lugar de ==
    static constexpr double POSE_EPS = 1e-4;

    bool hasPoseChanged(const geometry_msgs::msg::Pose::SharedPtr np,
                        const geometry_msgs::msg::Pose &lp)
    {
      return std::abs(np->position.x    - lp.position.x)    > POSE_EPS ||
             std::abs(np->position.y    - lp.position.y)    > POSE_EPS ||
             std::abs(np->position.z    - lp.position.z)    > POSE_EPS ||
             std::abs(np->orientation.x - lp.orientation.x) > POSE_EPS ||
             std::abs(np->orientation.y - lp.orientation.y) > POSE_EPS ||
             std::abs(np->orientation.z - lp.orientation.z) > POSE_EPS ||
             std::abs(np->orientation.w - lp.orientation.w) > POSE_EPS;
    }

    void onTargetUAVPoseChanged(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
      if (hasPoseChanged(msg, last_uav_pose))
      {
        last_uav_pose = *msg;
        triggerPublishing();
      }
    }

    void onTargetUGVPoseChanged(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
      if (hasPoseChanged(msg, last_ugv_pose))
      {
        last_ugv_pose = *msg;
        triggerPublishing();
      }
    }

    // A5: índice numérico para ordenar links correctamente (link_0 < link_1 < ... < link_final)
    static int linkIndex(const std::string &name)
    {
      if (name == "link_final") return INT_MAX;
      // Formato esperado: "link_N"
      try { return std::stoi(name.substr(5)); }
      catch (...) { return INT_MAX - 1; }
    }

    void triggerPublishing()
    {
      // A6: si ya se está publicando, marcar como pendiente para re-lanzar al terminar
      if (this->publishing.exchange(true))
      {
        this->pending_.store(true);
        return;
      }

      do {
        this->pending_.store(false);
        publishLinks();
      } while (this->pending_.load());

      this->publishing.store(false);
    }

    void publishLinks()
    {
      auto links = this->model->GetLinks();

      // A5: ordenar numéricamente
      std::sort(links.begin(), links.end(),
        [](const physics::LinkPtr &a, const physics::LinkPtr &b) {
          return linkIndex(a->GetName()) < linkIndex(b->GetName());
        });

      geometry_msgs::msg::PoseArray msg;
      msg.header.stamp    = this->ros_node_->now();
      msg.header.frame_id = "tether";
      msg.poses.reserve(links.size());

      for (const auto &lnk : links)
      {
        // A2: cachear WorldPose() — una sola llamada por link
        const auto pose = lnk->WorldPose();

        geometry_msgs::msg::Pose p;
        p.position.x    = pose.Pos().X();
        p.position.y    = pose.Pos().Y();
        p.position.z    = pose.Pos().Z();
        p.orientation.x = pose.Rot().X();
        p.orientation.y = pose.Rot().Y();
        p.orientation.z = pose.Rot().Z();
        p.orientation.w = pose.Rot().W();
        msg.poses.push_back(p);
      }

      // A3: publicar todo en un único mensaje, sin sleep
      this->publisher->publish(msg);
    }

    physics::ModelPtr model;
    gazebo_ros::Node::SharedPtr ros_node_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_position_uav_subscriber;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_position_ugv_subscriber;

    std::atomic<bool> publishing{false};
    std::atomic<bool> pending_{false};  // A6: evento perdido durante publicación

    geometry_msgs::msg::Pose last_uav_pose;
    geometry_msgs::msg::Pose last_ugv_pose;
  };

  GZ_REGISTER_MODEL_PLUGIN(TetherPositionPublisher)
}
