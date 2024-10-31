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
  /**
   * @brief Plugin to publish tether positions at a fixed rate.
   *
   * The TetherPositionPublisher plugin subscribes to UAV and UGV pose topics and publishes
   * tether positions as PoseStamped messages at a fixed interval (every second).
   */
  class TetherPositionPublisher : public ModelPlugin
  {
  public:
    /**
     * @brief Called when the plugin is loaded into the simulation.
     *
     * Initializes ROS 2 node, publishers, and timers for periodic publishing of tether positions.
     *
     * @param _model Pointer to the model that this plugin is attached to.
     * @param sdf Pointer to the SDF element that describes the plugin.
     */
    void Load(physics::ModelPtr _model, sdf::ElementPtr sdf) override
    {
      // Ensure that ROS 2 is initialized before proceeding
      if (!rclcpp::ok())
      {
        RCLCPP_FATAL(rclcpp::get_logger("TetherPositionPublisher"), "ROS 2 not initialized. Plugin loading failed.");
        return;
      }

      // Store the model pointer for later use
      this->model_ = _model;

      // Initialize the ROS 2 node using Gazebo's ROS node interface
      this->ros_node_ = gazebo_ros::Node::Get(sdf);

      if (!this->ros_node_)
      {
        RCLCPP_FATAL(this->ros_node_->get_logger(), "Failed to create ROS 2 node for TetherPositionPublisher plugin.");
        return;
      }

      // Initialize the publisher for tether positions
      this->publisher_ = this->ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("tether_positions", 10);

      // Initialize the subscriptions for UAV and UGV poses
      this->target_position_uav_subscriber_ = this->ros_node_->create_subscription<geometry_msgs::msg::Pose>(
          "/target_position_uav", 10, std::bind(&TetherPositionPublisher::onTargetUAVPoseChanged, this, std::placeholders::_1));

      this->target_position_ugv_subscriber_ = this->ros_node_->create_subscription<geometry_msgs::msg::Pose>(
          "/target_position_ugv", 10, std::bind(&TetherPositionPublisher::onTargetUGVPoseChanged, this, std::placeholders::_1));

      // Initialize the publishing flag
      this->publishing_.store(false);

      // Initialize the timer for periodic publishing (1 Hz)
      this->timer_ = this->ros_node_->create_wall_timer(
        std::chrono::seconds(10),
        std::bind(&TetherPositionPublisher::publishTetherPositions, this)
      );

      RCLCPP_INFO(this->ros_node_->get_logger(), "TetherPositionPublisher plugin successfully loaded.");
    }

  private:
    /**
     * @brief Callback function triggered when a new UAV pose is received.
     *
     * @param msg Shared pointer to the new UAV Pose message.
     */
    void onTargetUAVPoseChanged(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
      if (hasPoseChanged(msg, last_uav_pose_))
      {
        last_uav_pose_ = *msg;
        this->triggerPublishing();
      }
    }

    /**
     * @brief Callback function triggered when a new UGV pose is received.
     *
     * @param msg Shared pointer to the new UGV Pose message.
     */
    void onTargetUGVPoseChanged(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
      if (hasPoseChanged(msg, last_ugv_pose_))
      {
        last_ugv_pose_ = *msg;
        this->triggerPublishing();
      }
    }

    /**
     * @brief Determines whether the pose has changed compared to the last recorded pose.
     *
     * This function compares each component of the new pose with the last pose.
     * If any component differs, it returns true, indicating a pose change.
     *
     * @param new_pose Shared pointer to the new Pose message.
     * @param last_pose Reference to the last Pose message.
     * @return true If the pose has changed.
     * @return false If the pose remains unchanged.
     */
    bool hasPoseChanged(const geometry_msgs::msg::Pose::SharedPtr new_pose, const geometry_msgs::msg::Pose &last_pose)
    {
      // return !(new_pose->position.x == last_pose.position.x &&
      //          new_pose->position.y == last_pose.position.y &&
      //          new_pose->position.z == last_pose.position.z &&
      //          new_pose->orientation.x == last_pose.orientation.x &&
      //          new_pose->orientation.y == last_pose.orientation.y &&
      //          new_pose->orientation.z == last_pose.orientation.z &&
      //          new_pose->orientation.w == last_pose.orientation.w);
      // Uncomment the following line to always return true
      return true;
    }

    /**
     * @brief Initiates the publishing of tether positions.
     *
     * This function ensures that publishing does not overlap by using an atomic flag.
     * If publishing is already in progress, the function returns early.
     */
    void triggerPublishing()
    {
      // If already publishing, exit to prevent overlap
      if (this->publishing_.exchange(true))
      {
        return;
      }

      // Retrieve and sort all links by name for consistent publishing order
      auto links = this->model_->GetLinks();
      std::sort(links.begin(), links.end(), [](const physics::LinkPtr &a, const physics::LinkPtr &b) {
        return a->GetName() < b->GetName();
      });

      // Capture the current time for the header stamp
      auto first_publish_time = this->ros_node_->now();

      for (size_t i = 0; i < links.size(); ++i)
      {
        // Construct the PoseStamped message
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = first_publish_time;
        msg.header.frame_id = links[i]->GetName(); // Using link name as frame_id
        msg.pose.position.x = links[i]->WorldPose().Pos().X();
        msg.pose.position.y = links[i]->WorldPose().Pos().Y();
        msg.pose.position.z = links[i]->WorldPose().Pos().Z();
        msg.pose.orientation.x = links[i]->WorldPose().Rot().X();
        msg.pose.orientation.y = links[i]->WorldPose().Rot().Y();
        msg.pose.orientation.z = links[i]->WorldPose().Rot().Z();
        msg.pose.orientation.w = links[i]->WorldPose().Rot().W();

        // Publish the PoseStamped message
        this->publisher_->publish(msg);

        // Introduce a short delay to prevent network congestion
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      // Reset the publishing flag to allow future publishing cycles
      this->publishing_.store(false);
    }

    /**
     * @brief Publishes the current positions of tether links.
     *
     * This function is called periodically based on the timer's interval.
     * It iterates through all links of the model, constructs PoseStamped messages,
     * and publishes them to the specified ROS 2 topic.
     */
    void publishTetherPositions()
    {
      // Retrieve all links associated with the model
      auto links = this->model_->GetLinks();

      if (links.empty())
      {
        RCLCPP_WARN(this->ros_node_->get_logger(), "No links found in the model to publish tether positions.");
        return;
      }

      // Capture the current time for the header stamp
      auto publish_time = this->ros_node_->now();

      for (const auto &link : links)
      {
        // Construct the PoseStamped message
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = publish_time;
        pose_msg.header.frame_id = link->GetName(); // Using link name as frame_id

        // Retrieve the world pose of the link
        auto world_pose = link->WorldPose();

        // Populate the position fields
        pose_msg.pose.position.x = world_pose.Pos().X();
        pose_msg.pose.position.y = world_pose.Pos().Y();
        pose_msg.pose.position.z = world_pose.Pos().Z();

        // Populate the orientation fields
        pose_msg.pose.orientation.x = world_pose.Rot().X();
        pose_msg.pose.orientation.y = world_pose.Rot().Y();
        pose_msg.pose.orientation.z = world_pose.Rot().Z();
        pose_msg.pose.orientation.w = world_pose.Rot().W();

        // Publish the PoseStamped message
        this->publisher_->publish(pose_msg);

        RCLCPP_DEBUG(this->ros_node_->get_logger(), "Published tether position for link: %s", link->GetName().c_str());

        // Introduce a slight delay to prevent overwhelming the ROS 2 network
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }

    // Pointer to the Gazebo model
    physics::ModelPtr model_;

    // Shared pointer to the ROS 2 node
    gazebo_ros::Node::SharedPtr ros_node_;

    // ROS 2 publisher for tether positions
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;

    // ROS 2 subscriptions for UAV and UGV poses
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_position_uav_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_position_ugv_subscriber_;

    // Atomic flag to manage publishing state
    std::atomic<bool> publishing_{false};

    // Variables to store the last received poses
    geometry_msgs::msg::Pose last_uav_pose_;
    geometry_msgs::msg::Pose last_ugv_pose_;

    // Timer for periodic publishing (1 Hz)
    rclcpp::TimerBase::SharedPtr timer_;
  };

  // Register the plugin with Gazebo
  GZ_REGISTER_MODEL_PLUGIN(TetherPositionPublisher)
}
