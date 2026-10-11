#include "fake_vel_transform/fake_vel_transform.hpp"

#include <cmath>

#include <tf2/utils.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>

namespace fake_vel_transform
{
const std::string CMD_VEL_TOPIC = "/cmd_vel";
const std::string AFTER_TF_CMD_VEL = "/cmd_vel_chassis";
const std::string TRAJECTORY_TOPIC = "/local_plan";
std::string NAV_BASE_FRAME = "base_link";
std::string PLAN_FRAME = "map";
bool USE_PATH_HEADING_COMPENSATION = false;
bool SWAP_NAV_XY = false;
double CHASSIS_X_SIGN = 1.0;
double CHASSIS_Y_SIGN = 1.0;
const int TF_PUBLISH_FREQUENCY = 20;  // base_link to base_link_fake. Frequency in Hz.

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  // Declare and get the spin speed parameter
  this->declare_parameter<float>("spin_speed", -6.0);
  this->get_parameter("spin_speed", spin_speed_);
  this->declare_parameter<bool>("use_nav_wz", false);
  this->get_parameter("use_nav_wz", use_nav_wz_);
  this->declare_parameter<bool>("forward_nav_wz", false);
  this->get_parameter("forward_nav_wz", forward_nav_wz_);
  this->declare_parameter<std::string>("nav_base_frame", NAV_BASE_FRAME);
  this->get_parameter("nav_base_frame", NAV_BASE_FRAME);
  this->declare_parameter<bool>("use_path_heading_compensation", USE_PATH_HEADING_COMPENSATION);
  this->get_parameter("use_path_heading_compensation", USE_PATH_HEADING_COMPENSATION);
  this->declare_parameter<bool>("swap_nav_xy", SWAP_NAV_XY);
  this->get_parameter("swap_nav_xy", SWAP_NAV_XY);
  this->declare_parameter<double>("chassis_x_sign", CHASSIS_X_SIGN);
  this->get_parameter("chassis_x_sign", CHASSIS_X_SIGN);
  this->declare_parameter<double>("chassis_y_sign", CHASSIS_Y_SIGN);
  this->get_parameter("chassis_y_sign", CHASSIS_Y_SIGN);

  // TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // Create Publisher and Subscriber
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    CMD_VEL_TOPIC, 1, std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));
  cmd_vel_chassis_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    AFTER_TF_CMD_VEL, rclcpp::QoS(rclcpp::KeepLast(1)));
  local_pose_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    TRAJECTORY_TOPIC, 1,
    std::bind(&FakeVelTransform::localPoseCallback, this, std::placeholders::_1));

  // Create a timer to publish the transform regularly
  tf_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000 / TF_PUBLISH_FREQUENCY),
    std::bind(&FakeVelTransform::publishTransform, this));
}

// Get the local pose from planner
void FakeVelTransform::localPoseCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (!USE_PATH_HEADING_COMPENSATION) {
    current_angle_ = 0.0;
    return;
  }

  if (!msg || msg->poses.empty()) {
    RCLCPP_WARN(get_logger(), "Received empty or invalid PoseArray message");
    return;
  }

  // Choose a representative pose along the local plan.
  size_t index = std::min(msg->poses.size() / 4, msg->poses.size() - 1);
  const geometry_msgs::msg::Pose & selected_pose = msg->poses[index].pose;

  try {
    if (!msg->header.frame_id.empty()) {
      PLAN_FRAME = msg->header.frame_id;
    }

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped = tf2_buffer_->lookupTransform(PLAN_FRAME, NAV_BASE_FRAME, tf2::TimePointZero);
    base_link_angle_ = tf2::getYaw(transform_stamped.transform.rotation);

    // Derive the path heading from neighboring positions instead of trusting
    // pose.orientation directly. For omni trajectories, local plan orientations
    // can be 180 deg ambiguous even when the geometric path direction is correct.
    double path_heading = tf2::getYaw(selected_pose.orientation);
    if (msg->poses.size() >= 2) {
      size_t next_index = std::min(index + 1, msg->poses.size() - 1);
      if (next_index == index && index > 0) {
        next_index = index - 1;
      }

      const auto & current_position = msg->poses[index].pose.position;
      const auto & neighbor_position = msg->poses[next_index].pose.position;
      const double dx = neighbor_position.x - current_position.x;
      const double dy = neighbor_position.y - current_position.y;
      if (std::hypot(dx, dy) > 1e-4) {
        path_heading = std::atan2(dy, dx);
      }
    }

    const double angle_delta = path_heading - base_link_angle_;
    current_angle_ = std::atan2(std::sin(angle_delta), std::cos(angle_delta));
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s", PLAN_FRAME.c_str(), NAV_BASE_FRAME.c_str(), ex.what());
  }
}

// Transform the velocity from base_link to base_link_fake
void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try {
    const double angle_diff = USE_PATH_HEADING_COMPENSATION ? -current_angle_ : 0.0;

    geometry_msgs::msg::Twist aft_tf_vel;
    const double nav_frame_x =
      msg->linear.x * cos(angle_diff) + msg->linear.y * sin(angle_diff);
    const double nav_frame_y =
      -msg->linear.x * sin(angle_diff) + msg->linear.y * cos(angle_diff);

    const double chassis_cmd_x = SWAP_NAV_XY ? nav_frame_y : nav_frame_x;
    const double chassis_cmd_y = SWAP_NAV_XY ? nav_frame_x : nav_frame_y;

    // Default to the standard Nav2 x/y convention and only enable the old
    // axis-swap behavior explicitly for legacy bring-up experiments.
    // Odin routes the original Nav2 yaw rate to its gimbal in the sender.
    // Keep the legacy fixed chassis spin behavior for other launch modes.
    aft_tf_vel.angular.z = forward_nav_wz_ ? msg->angular.z :
      ((use_nav_wz_ && msg->angular.z != 0.0) ? spin_speed_ : 0.0);
    aft_tf_vel.linear.x = CHASSIS_X_SIGN * chassis_cmd_x;
    aft_tf_vel.linear.y = CHASSIS_Y_SIGN * chassis_cmd_y;

    cmd_vel_chassis_pub_->publish(aft_tf_vel);
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(this->get_logger(), "Could not transform odom to base_link: %s", ex.what());
  }
}

// Publish transform from base_link to base_link_fake
void FakeVelTransform::publishTransform()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = get_clock()->now();
  t.header.frame_id = NAV_BASE_FRAME;
  t.child_frame_id = "base_link_fake";
  tf2::Quaternion q;
  q.setRPY(0, 0, current_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
