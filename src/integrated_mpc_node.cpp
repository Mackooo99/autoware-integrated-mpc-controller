#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <chrono>
#include <memory>

class IntegratedMpcNode : public rclcpp::Node
{
public:
  IntegratedMpcNode()
  : Node("integrated_mpc_node")
  {
    ctrl_period_ = declare_parameter<double>("ctrl_period", 0.03);
    publish_enabled_ = declare_parameter<bool>("publish_enabled", false);

    using std::placeholders::_1;

    trajectory_sub_ =
      create_subscription<autoware_planning_msgs::msg::Trajectory>(
        "~/input/reference_trajectory", 1,
        [this](const autoware_planning_msgs::msg::Trajectory::SharedPtr msg) {
          trajectory_ = msg;
        });

    odometry_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
        "~/input/current_odometry", 1,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          odometry_ = msg;
        });

    steering_sub_ =
      create_subscription<autoware_vehicle_msgs::msg::SteeringReport>(
        "~/input/current_steering", 1,
        [this](const autoware_vehicle_msgs::msg::SteeringReport::SharedPtr msg) {
          steering_ = msg;
        });

    accel_sub_ =
      create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
        "~/input/current_accel", 1,
        [this](const geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr msg) {
          accel_ = msg;
        });

    operation_mode_sub_ =
      create_subscription<autoware_adapi_v1_msgs::msg::OperationModeState>(
        "~/input/current_operation_mode", 1,
        [this](const autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr msg) {
          operation_mode_ = msg;
        });

    control_pub_ =
      create_publisher<autoware_control_msgs::msg::Control>(
        "~/output/control_cmd", 1);

    predicted_trajectory_pub_ =
      create_publisher<autoware_planning_msgs::msg::Trajectory>(
        "~/output/predicted_trajectory", 1);

    const auto timer_period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(ctrl_period_));

    timer_ = create_wall_timer(
      timer_period,
      std::bind(&IntegratedMpcNode::onTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "Integrated MPC node started. ctrl_period = %.3f s, publish_enabled = %s",
      ctrl_period_,
      publish_enabled_ ? "true" : "false");
  }

private:
  void onTimer()
  {
    if (!trajectory_ || !odometry_ || !steering_ || !accel_ || !operation_mode_) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for trajectory, odometry, steering, acceleration and operation mode...");
      return;
    }

    RCLCPP_INFO_ONCE(
      get_logger(),
      "All required controller inputs are available.");

    // For the first integration test we deliberately do not control the vehicle.
    if (!publish_enabled_) {
      return;
    }

    /*
     * Temporary safe placeholder.
     * This will be replaced by:
     *
     * x0 -> Integrated MPC -> delta_cmd, jerk_cmd -> acceleration_cmd
     */

    autoware_control_msgs::msg::Control cmd;

    const auto stamp = now();
    cmd.stamp = stamp;

    cmd.lateral.stamp = stamp;
    cmd.lateral.steering_tire_angle =
      steering_->steering_tire_angle;
    cmd.lateral.steering_tire_rotation_rate = 0.0F;
    cmd.lateral.is_defined_steering_tire_rotation_rate = true;

    cmd.longitudinal.stamp = stamp;
    cmd.longitudinal.velocity = 0.0F;
    cmd.longitudinal.acceleration = 0.0F;
    cmd.longitudinal.jerk = 0.0F;
    cmd.longitudinal.is_defined_acceleration = true;
    cmd.longitudinal.is_defined_jerk = true;

    control_pub_->publish(cmd);
  }

  double ctrl_period_{0.03};
  bool publish_enabled_{false};

  autoware_planning_msgs::msg::Trajectory::SharedPtr trajectory_;
  nav_msgs::msg::Odometry::SharedPtr odometry_;
  autoware_vehicle_msgs::msg::SteeringReport::SharedPtr steering_;
  geometry_msgs::msg::AccelWithCovarianceStamped::SharedPtr accel_;
  autoware_adapi_v1_msgs::msg::OperationModeState::SharedPtr operation_mode_;

  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_sub_;
  rclcpp::Subscription<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr accel_sub_;
  rclcpp::Subscription<autoware_adapi_v1_msgs::msg::OperationModeState>::SharedPtr
    operation_mode_sub_;

  rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_pub_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    predicted_trajectory_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IntegratedMpcNode>());
  rclcpp::shutdown();
  return 0;
}
