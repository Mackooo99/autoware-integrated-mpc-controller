#include "autoware/integrated_mpc_trajectory_follower/integrated_prediction_builder.hpp"
#include "autoware/integrated_mpc_trajectory_follower/mpc_constraint_builder.hpp"
#include "autoware/integrated_mpc_trajectory_follower/mpc_cost_builder.hpp"
#include "autoware/integrated_mpc_trajectory_follower/prediction_reference_builder.hpp"
#include "autoware/integrated_mpc_trajectory_follower/tracking_state_builder.hpp"

#include "autoware/osqp_interface/osqp_interface.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_adapi_v1_msgs/msg/operation_mode_state.hpp>
#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>

#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cstdio>


class IntegratedMpcNode : public rclcpp::Node
{
public:
  IntegratedMpcNode()
  : Node("integrated_mpc_node")
  {
    // ========================================================
    // General controller parameters
    // ========================================================

    ctrl_period_ =
      declare_parameter<double>(
        "ctrl_period",
        0.03);

    publish_enabled_ =
      declare_parameter<bool>(
        "publish_enabled",
        false);

    wheel_base_ =
      declare_parameter<double>(
        "wheel_base",
        2.79);

    steer_tau_ =
      declare_parameter<double>(
        "steer_tau",
        0.27);

    nearest_dist_threshold_ =
      declare_parameter<double>(
        "nearest_dist_threshold",
        3.0);

    nearest_yaw_threshold_ =
      declare_parameter<double>(
        "nearest_yaw_threshold",
        1.046);


    // ========================================================
    // Prediction parameters
    // ========================================================

    prediction_horizon_ =
      declare_parameter<int>(
        "prediction_horizon",
        50);

    prediction_dt_ =
      declare_parameter<double>(
        "prediction_dt",
        0.1);

    min_prediction_length_ =
      declare_parameter<double>(
        "min_prediction_length",
        5.0);

    minimum_prediction_velocity_ =
      declare_parameter<double>(
        "minimum_prediction_velocity",
        0.1);


    // ========================================================
    // MPC cost weights
    // ========================================================

    weight_lat_error_ =
      declare_parameter<double>(
        "weight_lat_error",
        1.0);

    weight_heading_error_squared_vel_ =
      declare_parameter<double>(
        "weight_heading_error_squared_vel",
        0.3);

    weight_velocity_error_ =
      declare_parameter<double>(
        "weight_velocity_error",
        1.0);

    weight_acceleration_error_ =
      declare_parameter<double>(
        "weight_acceleration_error",
        0.1);

    weight_steering_input_ =
      declare_parameter<double>(
        "weight_steering_input",
        1.0);

    weight_steering_input_squared_vel_ =
      declare_parameter<double>(
        "weight_steering_input_squared_vel",
        0.25);

    weight_jerk_ =
      declare_parameter<double>(
        "weight_jerk",
        0.1);

    terminal_weight_lat_error_ =
      declare_parameter<double>(
        "terminal_weight_lat_error",
        1.0);

    terminal_weight_heading_error_ =
      declare_parameter<double>(
        "terminal_weight_heading_error",
        0.1);

    terminal_weight_velocity_error_ =
      declare_parameter<double>(
        "terminal_weight_velocity_error",
        1.0);


    // ========================================================
    // MPC hard constraints
    // ========================================================

    max_steering_angle_ =
      declare_parameter<double>(
        "max_steering_angle",
        0.7);

    min_acceleration_ =
      declare_parameter<double>(
        "min_acceleration",
        -5.0);

    max_acceleration_ =
      declare_parameter<double>(
        "max_acceleration",
        3.0);

    min_jerk_ =
      declare_parameter<double>(
        "min_jerk",
        -5.0);

    max_jerk_ =
      declare_parameter<double>(
        "max_jerk",
        2.0);

    curvature_for_steer_rate_ =
      declare_parameter<std::vector<double>>(
        "curvature_for_steer_rate",
        {0.001, 0.002, 0.01});

    steer_rate_deg_by_curvature_ =
      declare_parameter<std::vector<double>>(
        "steer_rate_deg_by_curvature",
        {40.0, 50.0, 60.0});

    velocity_for_steer_rate_ =
      declare_parameter<std::vector<double>>(
        "velocity_for_steer_rate",
        {10.0, 15.0, 20.0});

    steer_rate_deg_by_velocity_ =
      declare_parameter<std::vector<double>>(
        "steer_rate_deg_by_velocity",
        {60.0, 50.0, 40.0});


    // ========================================================
    // Helper/model builders
    // ========================================================

    tracking_state_builder_ =
      std::make_unique<
        autoware::integrated_mpc::TrackingStateBuilder>(
          wheel_base_,
          nearest_dist_threshold_,
          nearest_yaw_threshold_);

    prediction_reference_builder_ =
      std::make_unique<
        autoware::integrated_mpc::PredictionReferenceBuilder>(
          wheel_base_,
          static_cast<std::size_t>(
            prediction_horizon_),
          prediction_dt_,
          min_prediction_length_,
          minimum_prediction_velocity_);

    integrated_prediction_builder_ =
      std::make_unique<
        autoware::integrated_mpc::IntegratedPredictionBuilder>(
          wheel_base_,
          steer_tau_);


    // ========================================================
    // Subscribers
    // ========================================================

    trajectory_sub_ =
      create_subscription<
        autoware_planning_msgs::msg::Trajectory>(
          "~/input/reference_trajectory",
          rclcpp::QoS(1).reliable(),
          [this](
            const autoware_planning_msgs::msg::Trajectory::
              SharedPtr msg)
          {
            trajectory_ = msg;
          });

    odometry_sub_ =
      create_subscription<
        nav_msgs::msg::Odometry>(
          "~/input/current_odometry",
          rclcpp::QoS(1).reliable(),
          [this](
            const nav_msgs::msg::Odometry::
              SharedPtr msg)
          {
            odometry_ = msg;
          });

    steering_sub_ =
      create_subscription<
        autoware_vehicle_msgs::msg::SteeringReport>(
          "~/input/current_steering",
          rclcpp::QoS(1).reliable(),
          [this](
            const autoware_vehicle_msgs::msg::
              SteeringReport::SharedPtr msg)
          {
            steering_ = msg;
          });

    accel_sub_ =
      create_subscription<
        geometry_msgs::msg::AccelWithCovarianceStamped>(
          "~/input/current_accel",
          rclcpp::QoS(1).reliable(),
          [this](
            const geometry_msgs::msg::
              AccelWithCovarianceStamped::SharedPtr msg)
          {
            accel_ = msg;
          });

    /*
     * OperationModeState publisher in Autoware uses
     * TRANSIENT_LOCAL durability, so we match that QoS.
     */
    operation_mode_sub_ =
      create_subscription<
        autoware_adapi_v1_msgs::msg::OperationModeState>(
          "~/input/current_operation_mode",
          rclcpp::QoS(1)
            .reliable()
            .transient_local(),
          [this](
            const autoware_adapi_v1_msgs::msg::
              OperationModeState::SharedPtr msg)
          {
            operation_mode_ = msg;
          });


    // ========================================================
    // Publishers
    // ========================================================

    control_pub_ =
      create_publisher<
        autoware_control_msgs::msg::Control>(
          "~/output/control_cmd",
          rclcpp::QoS(1).reliable());

    predicted_trajectory_pub_ =
      create_publisher<
        autoware_planning_msgs::msg::Trajectory>(
          "~/output/predicted_trajectory",
          rclcpp::QoS(1).reliable());


    // ========================================================
    // Control timer
    // ========================================================

    const auto timer_period =
      std::chrono::duration_cast<
        std::chrono::nanoseconds>(
          std::chrono::duration<double>(
            ctrl_period_));

    timer_ =
      create_wall_timer(
        timer_period,
        std::bind(
          &IntegratedMpcNode::onTimer,
          this));


    // ========================================================
    // Startup diagnostics
    // ========================================================

    RCLCPP_INFO(
      get_logger(),
      "Integrated MPC node started.");

    RCLCPP_INFO(
      get_logger(),
      "ctrl_period = %.3f s",
      ctrl_period_);

    RCLCPP_INFO(
      get_logger(),
      "wheel_base = %.3f m",
      wheel_base_);

    RCLCPP_INFO(
      get_logger(),
      "prediction horizon = %d",
      prediction_horizon_);

    RCLCPP_INFO(
      get_logger(),
      "publish_enabled = %s",
      publish_enabled_ ?
        "true" :
        "false");
  }


private:
  void onTimer()
  {
    // ========================================================
    // 1. Check input availability
    // ========================================================

    if (
      !trajectory_ ||
      !odometry_ ||
      !steering_ ||
      !accel_ ||
      !operation_mode_)
    {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Waiting for controller inputs...");

      return;
    }

    if (trajectory_->points.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Received empty reference trajectory.");

      return;
    }


    // ========================================================
    // 2. Build current tracking state
    //
    // x0 =
    //
    // [ e_y, e_psi, e_v, delta, a ]^T
    // ========================================================

    const auto tracking_state =
      tracking_state_builder_->build(
        *trajectory_,
        *odometry_,
        *steering_,
        *accel_);

    if (!tracking_state) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Could not determine nearest reference point.");

      return;
    }

    const Eigen::VectorXd x0 =
      tracking_state->x0;


    // ========================================================
    // 3. Build prediction reference horizon
    // ========================================================

    const auto prediction_reference =
      prediction_reference_builder_->build(
        *trajectory_,
        tracking_state->nearest_index);


    // ========================================================
    // 4. Build integrated LTV prediction model
    //
    // X = Aex*x0 + Bex*U + Wex
    // ========================================================

    const auto integrated_prediction =
      integrated_prediction_builder_->build(
        prediction_reference,
        x0);


    // ========================================================
    // 5. Build MPC hard constraints
    // ========================================================

    autoware::integrated_mpc::MpcConstraintParameters
      constraint_parameters;

    constraint_parameters.max_steering_angle =
      max_steering_angle_;

    constraint_parameters.min_acceleration =
      min_acceleration_;

    constraint_parameters.max_acceleration =
      max_acceleration_;

    constraint_parameters.min_jerk =
      min_jerk_;

    constraint_parameters.max_jerk =
      max_jerk_;

    constraint_parameters.curvature_for_steer_rate =
      curvature_for_steer_rate_;

    constraint_parameters.steer_rate_deg_by_curvature =
      steer_rate_deg_by_curvature_;

    constraint_parameters.velocity_for_steer_rate =
      velocity_for_steer_rate_;

    constraint_parameters.steer_rate_deg_by_velocity =
      steer_rate_deg_by_velocity_;

    // ========================================================
  // Previous steering command used by the first rate bound
  //
  // Before the first published MPC command we initialize it
  // from the measured steering state.
  //
  // Afterwards we use the command that was ACTUALLY published.
  // ========================================================

  const double previous_steering_command =
    previous_command_initialized_
    ?
    previous_steering_command_
    :
    std::clamp(
      x0(3),
      -max_steering_angle_,
      max_steering_angle_);

    const auto mpc_constraints =
  autoware::integrated_mpc::
    MpcConstraintBuilder::build(
      integrated_prediction.extended,
      prediction_reference,
      x0,
      previous_steering_command,
      ctrl_period_,
      constraint_parameters);


    // ========================================================
    // 6. Build MPC cost
    // ========================================================

    autoware::integrated_mpc::MpcCostWeights
      cost_weights;

    cost_weights.lateral_error =
      weight_lat_error_;

    cost_weights.heading_error_squared_velocity =
      weight_heading_error_squared_vel_;

    cost_weights.velocity_error =
      weight_velocity_error_;

    cost_weights.acceleration_error =
      weight_acceleration_error_;

    cost_weights.steering_input =
      weight_steering_input_;

    cost_weights.steering_input_squared_velocity =
      weight_steering_input_squared_vel_;

    cost_weights.jerk =
      weight_jerk_;

    cost_weights.terminal_lateral_error =
      terminal_weight_lat_error_;

    cost_weights.terminal_heading_error =
      terminal_weight_heading_error_;

    cost_weights.terminal_velocity_error =
      terminal_weight_velocity_error_;

    /*
     * IMPORTANT:
     *
     * mpc_cost must be built BEFORE the OSQP block.
     *
     * OSQP uses:
     *
     *   P = H
     *   q = f
     */
    const auto mpc_cost =
      autoware::integrated_mpc::
        MpcCostBuilder::build(
          integrated_prediction.extended,
          prediction_reference,
          x0,
          cost_weights);


    // ========================================================
    // 7. Validate constraint matrices
    // ========================================================

    const bool constraints_finite =
      mpc_constraints.A.allFinite() &&
      mpc_constraints.lower.allFinite() &&
      mpc_constraints.upper.allFinite() &&
      mpc_constraints.steering_rate_limit.allFinite();

    bool bounds_valid = true;

    for (
      Eigen::Index i = 0;
      i < mpc_constraints.lower.size();
      ++i)
    {
      if (
        mpc_constraints.lower(i) >
        mpc_constraints.upper(i))
      {
        bounds_valid = false;
        break;
      }
    }


    // ========================================================
    // 8. Validate cost matrices
    // ========================================================

    const double h_symmetry_error =
      (
        mpc_cost.H -
        mpc_cost.H.transpose()
      ).norm();

    Eigen::SelfAdjointEigenSolver<
      Eigen::MatrixXd>
      h_eigen_solver(
        mpc_cost.H);

    double min_h_eigenvalue =
      std::numeric_limits<double>::
        quiet_NaN();

    if (
      h_eigen_solver.info() ==
      Eigen::Success)
    {
      min_h_eigenvalue =
        h_eigen_solver.
          eigenvalues().
          minCoeff();
    }

    const bool cost_finite =
      mpc_cost.Qex.allFinite() &&
      mpc_cost.Rex.allFinite() &&
      mpc_cost.H.allFinite() &&
      mpc_cost.f.allFinite() &&
      std::isfinite(
        mpc_cost.constant_term);


    // ========================================================
    // 9. Solve actual MPC QP with OSQP
    //
    // minimize:
    //
    //   0.5 * U^T H U + f^T U
    //
    // subject to:
    //
    //   lower <= A_constraint * U <= upper
    //
    // IMPORTANT:
    // During 9B we ONLY solve the QP.
    // We still do not publish the optimal command.
    // ========================================================

    const std::vector<double> q(
      mpc_cost.f.data(),
      mpc_cost.f.data() +
        mpc_cost.f.size());

    const std::vector<double> lower(
      mpc_constraints.lower.data(),
      mpc_constraints.lower.data() +
        mpc_constraints.lower.size());

    const std::vector<double> upper(
      mpc_constraints.upper.data(),
      mpc_constraints.upper.data() +
        mpc_constraints.upper.size());

    /*
     * For the first integration test we construct a fresh
     * solver on every timer iteration.
     *
     * This is NOT yet the final real-time implementation.
     * Later we can keep the solver persistent and use warm start.
     */
    autoware::osqp_interface::OSQPInterface solver(
      1.0e-6,
      true);

    const auto solve_start =
      std::chrono::steady_clock::now();

    const auto solver_result =
      solver.optimize(
        mpc_cost.H,
        mpc_constraints.A,
        q,
        lower,
        upper);

    const auto solve_end =
      std::chrono::steady_clock::now();

    const double total_solver_time_ms =
      std::chrono::duration<
        double,
        std::milli>(
          solve_end -
          solve_start).count();

    const std::string solver_status =
      solver.getStatusMessage();

    const auto solver_status_value =
      solver.getStatus();

    const auto solver_iterations =
      solver.getTakenIter();

    const double osqp_runtime_ms =
      solver.getRunTime() *
      1.0e3;

    const double solver_objective =
      solver.getObjVal();

    const auto solver_exit_flag =
      solver.getExitFlag();

    const bool solver_solved =
      solver_status.find("solved") !=
      std::string::npos;

    const std::size_t expected_solution_size =
      static_cast<std::size_t>(
        mpc_cost.f.size());

    const bool solution_size_valid =
      solver_result.primal_solution.size() ==
      expected_solution_size;


    // ========================================================
    // 10. Convert OSQP primal solution to Eigen
    // ========================================================

    Eigen::VectorXd optimal_input;

    if (solution_size_valid) {
      optimal_input =
        Eigen::Map<
          const Eigen::VectorXd>(
            solver_result.primal_solution.data(),
            static_cast<Eigen::Index>(
              solver_result.primal_solution.size()));
    } else {
      optimal_input =
        Eigen::VectorXd::Zero(
          mpc_cost.f.size());
    }

    const bool optimal_solution_finite =
      solution_size_valid &&
      optimal_input.allFinite();


    // ========================================================
    // 11. Independently validate optimal constraints
    // ========================================================

    double maximum_constraint_violation =
      0.0;

    std::size_t constraint_violation_count =
      0;

    if (solution_size_valid) {
  const Eigen::VectorXd constraint_value =
    mpc_constraints.A *
    optimal_input;

  std::string violation_log;

  for (
    Eigen::Index i = 0;
    i < constraint_value.size();
    ++i)
  {
    double violation = 0.0;

    if (
      constraint_value(i) <
      mpc_constraints.lower(i))
    {
      violation =
        mpc_constraints.lower(i) -
        constraint_value(i);
    }
    else if (
      constraint_value(i) >
      mpc_constraints.upper(i))
    {
      violation =
        constraint_value(i) -
        mpc_constraints.upper(i);
    }

    if (violation > 1.0e-6) {
      ++constraint_violation_count;

      // ======================================================
      // Identify constraint group
      //
      // For N = 50:
      //
      //   0   ... 49  : steering angle
      //   50  ... 99  : jerk
      //   100 ... 149 : acceleration
      //   150 ... 199 : steering rate
      // ======================================================

      const Eigen::Index N =
        static_cast<Eigen::Index>(
          prediction_reference.points.size());

      std::string constraint_name;
      Eigen::Index step = -1;

      if (i < N) {
        constraint_name =
          "steering_angle";

        step = i;
      }
      else if (i < 2 * N) {
        constraint_name =
          "jerk";

        step =
          i - N;
      }
      else if (i < 3 * N) {
        constraint_name =
          "acceleration";

        step =
          i - 2 * N;
      }
      else {
        constraint_name =
          "steering_rate";

        step =
          i - 3 * N;
      }

      char buffer[512];

      std::snprintf(
        buffer,
        sizeof(buffer),

        "  row=%ld"
        " type=%s"
        " k=%ld"
        " value=%+.9f"
        " lower=%+.9f"
        " upper=%+.9f"
        " violation=%.9e\n",

        static_cast<long>(i),

        constraint_name.c_str(),

        static_cast<long>(step),

        constraint_value(i),

        mpc_constraints.lower(i),

        mpc_constraints.upper(i),

        violation);

      violation_log +=
        buffer;
    }

    maximum_constraint_violation =
      std::max(
        maximum_constraint_violation,
        violation);
  }

  if (!violation_log.empty()) {
    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,

      "\n"
      "Constraint violation detail:\n"
      "%s",

      violation_log.c_str());
  }
}


    // ========================================================
    // 12. Independently validate objective value
    // ========================================================

    double direct_qp_objective =
      std::numeric_limits<double>::
        quiet_NaN();

    double full_mpc_cost =
      std::numeric_limits<double>::
        quiet_NaN();

    if (solution_size_valid) {
      /*
       * This is exactly the objective seen by OSQP:
       *
       *   0.5 U^T H U + f^T U
       */
      direct_qp_objective =
        0.5 *
        optimal_input.dot(
          mpc_cost.H *
          optimal_input)
        +
        mpc_cost.f.dot(
          optimal_input);

      /*
       * Full MPC cost additionally includes the constant
       * term discarded during QP optimization.
       */
      full_mpc_cost =
        direct_qp_objective +
        mpc_cost.constant_term;
    }

    const double objective_error =
      (
        solution_size_valid &&
        std::isfinite(
          solver_objective) &&
        std::isfinite(
          direct_qp_objective)
      )
      ?
      std::abs(
        solver_objective -
        direct_qp_objective)
      :
      std::numeric_limits<double>::
        infinity();


    // ========================================================
    // 13. Predict state trajectory for optimal U
    // ========================================================

    Eigen::VectorXd optimal_state_prediction;

    if (solution_size_valid) {
      optimal_state_prediction =
        integrated_prediction.extended.Aex *
          x0
        +
        integrated_prediction.extended.Bex *
          optimal_input
        +
        integrated_prediction.extended.Wex;
    }
    
    
    // ========================================================
// 9C diagnostic:
// inspect first few optimal control actions and terminal
// predicted state before enabling closed-loop control.
// ========================================================

if (
  solver_solved &&
  solution_size_valid &&
  optimal_solution_finite)
{
  const std::size_t preview_steps =
    std::min<std::size_t>(
      5,
      prediction_reference.points.size());

  std::string sequence_log =
    "\nOptimal control sequence preview:\n";

  for (
    std::size_t k = 0;
    k < preview_steps;
    ++k)
  {
    const Eigen::Index input_index =
      static_cast<Eigen::Index>(
        2 * k);

    const Eigen::Index state_index =
      static_cast<Eigen::Index>(
        5 * k);

    const double delta_cmd =
      optimal_input(
        input_index);

    const double jerk =
      optimal_input(
        input_index + 1);

    const double ey =
      optimal_state_prediction(
        state_index);

    const double epsi =
      optimal_state_prediction(
        state_index + 1);

    const double ev =
      optimal_state_prediction(
        state_index + 2);

    const double delta =
      optimal_state_prediction(
        state_index + 3);

    const double acceleration =
      optimal_state_prediction(
        state_index + 4);

    char buffer[512];

    std::snprintf(
      buffer,
      sizeof(buffer),
      "  k=%zu:"
      " delta_cmd=%+.6f"
      " jerk=%+.6f"
      " | ey=%+.6f"
      " epsi=%+.6f"
      " ev=%+.6f"
      " delta=%+.6f"
      " a=%+.6f\n",
      k,
      delta_cmd,
      jerk,
      ey,
      epsi,
      ev,
      delta,
      acceleration);

    sequence_log +=
      buffer;
  }


  // ======================================================
  // Terminal predicted state
  // ======================================================

  constexpr Eigen::Index NX = 5;

  const Eigen::Index terminal_offset =
    optimal_state_prediction.size() -
    NX;

  const Eigen::VectorXd terminal_state =
    optimal_state_prediction.segment(
      terminal_offset,
      NX);

  char terminal_buffer[512];

  std::snprintf(
    terminal_buffer,
    sizeof(terminal_buffer),

    "\n"
    "Terminal optimal state:\n"
    "  e_y   = %+.6f m\n"
    "  e_psi = %+.6f rad\n"
    "  e_v   = %+.6f m/s\n"
    "  delta = %+.6f rad\n"
    "  a     = %+.6f m/s^2",

    terminal_state(0),
    terminal_state(1),
    terminal_state(2),
    terminal_state(3),
    terminal_state(4));

  sequence_log +=
    terminal_buffer;

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "%s",
    sequence_log.c_str());
}



    // ========================================================
    // 14. Extract first receding-horizon action
    //
    // U =
    //
    // [delta_cmd_0, jerk_0,
    //  delta_cmd_1, jerk_1,
    //  ...
    //  delta_cmd_N-1, jerk_N-1]^T
    //
    // Only U[0] and U[1] would actually be applied during
    // a receding-horizon implementation.
    // ========================================================

    double first_steering_command =
      std::numeric_limits<double>::
        quiet_NaN();

    double first_jerk_command =
      std::numeric_limits<double>::
        quiet_NaN();

    double first_predicted_lateral_error =
      std::numeric_limits<double>::
        quiet_NaN();

    double first_predicted_heading_error =
      std::numeric_limits<double>::
        quiet_NaN();

    double first_predicted_velocity_error =
      std::numeric_limits<double>::
        quiet_NaN();

    double first_predicted_steering =
      std::numeric_limits<double>::
        quiet_NaN();

    double first_predicted_acceleration =
      std::numeric_limits<double>::
        quiet_NaN();

    if (
      solution_size_valid &&
      optimal_input.size() >= 2 &&
      optimal_state_prediction.size() >= 5)
    {
      first_steering_command =
        optimal_input(0);

      first_jerk_command =
        optimal_input(1);

      first_predicted_lateral_error =
        optimal_state_prediction(0);

      first_predicted_heading_error =
        optimal_state_prediction(1);

      first_predicted_velocity_error =
        optimal_state_prediction(2);

      first_predicted_steering =
        optimal_state_prediction(3);

      first_predicted_acceleration =
        optimal_state_prediction(4);
    }


    // ========================================================
// Closed-loop solution acceptance
//
// OSQP may return very small numerical constraint
// violations at active bounds.
//
// In S2 we observed approximately 1.2e-4, therefore
// 1e-3 is a conservative numerical acceptance threshold.
// ========================================================

constexpr double PUBLISH_CONSTRAINT_TOLERANCE =
  1.0e-3;

const bool solution_valid_for_control =
  solver_solved &&
  solution_size_valid &&
  optimal_solution_finite &&
  constraints_finite &&
  bounds_valid &&
  cost_finite &&
  maximum_constraint_violation <=
    PUBLISH_CONSTRAINT_TOLERANCE;

    // ========================================================
    // 15. OSQP diagnostics
    // ========================================================

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,

      "\n"
      "Integrated MPC OSQP:\n"
      "  status            = %s\n"
      "  status value      = %d\n"
      "  exit flag         = %d\n"
      "  iterations        = %d\n"
      "  OSQP runtime      = %.6f ms\n"
      "  total solve time  = %.6f ms\n"
      "  solution size     = %zu\n"
      "  solved            = %s\n"
      "  solution finite   = %s\n"
      "  objective error   = %.9e\n"
      "\n"
      "  solver objective  = %.9e\n"
      "  direct objective  = %.9e\n"
      "  full MPC cost     = %.9e\n"
      "\n"
      "  violations        = %zu\n"
      "  max violation     = %.9e\n"
      "\n"
      "  FIRST OPTIMAL INPUT:\n"
      "    delta_cmd       = %+.9f rad\n"
      "    jerk            = %+.9f m/s^3\n"
      "\n"
      "  FIRST OPTIMAL STATE:\n"
      "    e_y             = %+.9f m\n"
      "    e_psi           = %+.9f rad\n"
      "    e_v             = %+.9f m/s\n"
      "    delta           = %+.9f rad\n"
      "    a               = %+.9f m/s^2",

      solver_status.c_str(),

      static_cast<int>(
        solver_status_value),

      static_cast<int>(
        solver_exit_flag),

      static_cast<int>(
        solver_iterations),

      osqp_runtime_ms,

      total_solver_time_ms,

      solver_result.primal_solution.size(),

      solver_solved ?
        "true" :
        "false",

      optimal_solution_finite ?
        "true" :
        "false",

      objective_error,

      solver_objective,

      direct_qp_objective,

      full_mpc_cost,

      constraint_violation_count,

      maximum_constraint_violation,

      first_steering_command,

      first_jerk_command,

      first_predicted_lateral_error,

      first_predicted_heading_error,

      first_predicted_velocity_error,

      first_predicted_steering,

      first_predicted_acceleration);


    // ========================================================
    // 16. Constraint diagnostics
    // ========================================================

    constexpr double RAD2DEG =
      57.295779513082320876;

    const double min_steering_rate_deg =
      mpc_constraints.steering_rate_limit.
        minCoeff() *
      RAD2DEG;

    const double max_steering_rate_deg =
      mpc_constraints.steering_rate_limit.
        maxCoeff() *
      RAD2DEG;

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,

      "\n"
      "Integrated MPC constraints:\n"
      "  A             = %ld x %ld\n"
      "  lower         = %ld x 1\n"
      "  upper         = %ld x 1\n"
      "  steer limit   = +/- %.4f rad\n"
      "  accel limits  = [%.4f, %.4f] m/s^2\n"
      "  jerk limits   = [%.4f, %.4f] m/s^3\n"
      "  steer rate    = [%.4f, %.4f] deg/s\n"
      "  previous steer= %+.6f rad\n"
      "  bounds valid  = %s\n"
      "  finite        = %s",

      static_cast<long>(
        mpc_constraints.A.rows()),

      static_cast<long>(
        mpc_constraints.A.cols()),

      static_cast<long>(
        mpc_constraints.lower.size()),

      static_cast<long>(
        mpc_constraints.upper.size()),

      max_steering_angle_,

      min_acceleration_,
      max_acceleration_,

      min_jerk_,
      max_jerk_,

      min_steering_rate_deg,
      max_steering_rate_deg,

      previous_steering_command,

      bounds_valid ?
        "true" :
        "false",

      constraints_finite ?
        "true" :
        "false");


    // ========================================================
    // 17. Cost diagnostics
    // ========================================================

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,

      "\n"
      "Integrated MPC cost:\n"
      "  Qex           = %ld x %ld\n"
      "  Rex           = %ld x %ld\n"
      "  H             = %ld x %ld\n"
      "  f             = %ld x 1\n"
      "  ||H||         = %.6e\n"
      "  ||f||         = %.6e\n"
      "  symmetry err  = %.6e\n"
      "  min eig(H)    = %.6e\n"
      "  constant      = %.6e\n"
      "  finite        = %s",

      static_cast<long>(
        mpc_cost.Qex.rows()),

      static_cast<long>(
        mpc_cost.Qex.cols()),

      static_cast<long>(
        mpc_cost.Rex.rows()),

      static_cast<long>(
        mpc_cost.Rex.cols()),

      static_cast<long>(
        mpc_cost.H.rows()),

      static_cast<long>(
        mpc_cost.H.cols()),

      static_cast<long>(
        mpc_cost.f.size()),

      mpc_cost.H.norm(),

      mpc_cost.f.norm(),

      h_symmetry_error,

      min_h_eigenvalue,

      mpc_cost.constant_term,

      cost_finite ?
        "true" :
        "false");


    // ========================================================
    // 18. Check feasibility of nominal reference input
    //
    // Uref =
    //
    // [delta_ref_0, 0,
    //  delta_ref_1, 0,
    //  ...]
    //
    // This is only a diagnostic reference sequence.
    // ========================================================

    const Eigen::VectorXd reference_constraint_value =
      mpc_constraints.A *
      integrated_prediction.reference_input;

    std::size_t reference_violation_count =
      0;

    double maximum_reference_violation =
      0.0;

    for (
      Eigen::Index i = 0;
      i < reference_constraint_value.size();
      ++i)
    {
      double violation = 0.0;

      if (
        reference_constraint_value(i) <
        mpc_constraints.lower(i))
      {
        violation =
          mpc_constraints.lower(i) -
          reference_constraint_value(i);
      }
      else if (
        reference_constraint_value(i) >
        mpc_constraints.upper(i))
      {
        violation =
          reference_constraint_value(i) -
          mpc_constraints.upper(i);
      }

      if (violation > 0.0) {
        ++reference_violation_count;

        maximum_reference_violation =
          std::max(
            maximum_reference_violation,
            violation);
      }
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,

      "\n"
      "Constraint reference check:\n"
      "  Uref size      = %ld\n"
      "  violations     = %zu\n"
      "  max violation  = %.6e",

      static_cast<long>(
        integrated_prediction.
          reference_input.size()),

      reference_violation_count,

      maximum_reference_violation);


    // ========================================================
    // 19. Prediction-model diagnostics
    // ========================================================

    const auto & extended =
      integrated_prediction.extended;

    const bool prediction_finite =
      extended.Aex.allFinite() &&
      extended.Bex.allFinite() &&
      extended.Wex.allFinite() &&
      integrated_prediction.
        reference_input_response.
        allFinite();

    const std::size_t prediction_size =
      prediction_reference.points.size();

    if (prediction_size > 0) {
      constexpr Eigen::Index NX = 5;

      const Eigen::Index middle_index =
        static_cast<Eigen::Index>(
          prediction_size / 2);

      const Eigen::Index last_index =
        static_cast<Eigen::Index>(
          prediction_size - 1);

      const Eigen::VectorXd first_state =
        integrated_prediction.
          reference_input_response.segment(
            0,
            NX);

      const Eigen::VectorXd middle_state =
        integrated_prediction.
          reference_input_response.segment(
            middle_index * NX,
            NX);

      const Eigen::VectorXd last_state =
        integrated_prediction.
          reference_input_response.segment(
            last_index * NX,
            NX);

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,

        "\n"
        "Integrated prediction model:\n"
        "  Aex           = %ld x %ld\n"
        "  Bex           = %ld x %ld\n"
        "  Wex           = %ld x 1\n"
        "  ||Aex||       = %.6e\n"
        "  ||Bex||       = %.6e\n"
        "  ||Wex||       = %.6e\n"
        "  finite        = %s\n"
        "\n"
        "  FIRST predicted state:\n"
        "    e_y         = %+.6f\n"
        "    e_psi       = %+.6f\n"
        "    e_v         = %+.6f\n"
        "    delta       = %+.6f\n"
        "    a           = %+.6f\n"
        "\n"
        "  MIDDLE predicted state:\n"
        "    e_y         = %+.6f\n"
        "    e_psi       = %+.6f\n"
        "    e_v         = %+.6f\n"
        "    delta       = %+.6f\n"
        "    a           = %+.6f\n"
        "\n"
        "  LAST predicted state:\n"
        "    e_y         = %+.6f\n"
        "    e_psi       = %+.6f\n"
        "    e_v         = %+.6f\n"
        "    delta       = %+.6f\n"
        "    a           = %+.6f",

        static_cast<long>(
          extended.Aex.rows()),

        static_cast<long>(
          extended.Aex.cols()),

        static_cast<long>(
          extended.Bex.rows()),

        static_cast<long>(
          extended.Bex.cols()),

        static_cast<long>(
          extended.Wex.size()),

        extended.Aex.norm(),
        extended.Bex.norm(),
        extended.Wex.norm(),

        prediction_finite ?
          "true" :
          "false",

        first_state(0),
        first_state(1),
        first_state(2),
        first_state(3),
        first_state(4),

        middle_state(0),
        middle_state(1),
        middle_state(2),
        middle_state(3),
        middle_state(4),

        last_state(0),
        last_state(1),
        last_state(2),
        last_state(3),
        last_state(4));
    }


    // ========================================================
    // 20. Prediction-reference diagnostics
    // ========================================================

    if (!prediction_reference.points.empty()) {
      const auto & first =
        prediction_reference.points.front();

      const auto & middle =
        prediction_reference.points.at(
          prediction_reference.points.size() /
          2);

      const auto & last =
        prediction_reference.points.back();

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,

        "\n"
        "Prediction horizon:\n"
        "  dt            = %.4f s\n"
        "  size          = %zu\n"
        "  total length  = %.4f m\n"
        "\n"
        "  FIRST:\n"
        "    index       = %zu\n"
        "    s           = %.4f m\n"
        "    v_ref       = %.4f m/s\n"
        "    a_ref       = %.4f m/s^2\n"
        "    kappa       = %.6f 1/m\n"
        "    delta_ref   = %.4f rad\n"
        "\n"
        "  MIDDLE:\n"
        "    index       = %zu\n"
        "    s           = %.4f m\n"
        "    v_ref       = %.4f m/s\n"
        "    a_ref       = %.4f m/s^2\n"
        "    kappa       = %.6f 1/m\n"
        "\n"
        "  LAST:\n"
        "    index       = %zu\n"
        "    s           = %.4f m\n"
        "    v_ref       = %.4f m/s\n"
        "    a_ref       = %.4f m/s^2\n"
        "    kappa       = %.6f 1/m\n"
        "    delta_ref   = %.4f rad",

        prediction_reference.prediction_dt,

        prediction_reference.points.size(),

        prediction_reference.total_arc_length,

        first.trajectory_index,
        first.arc_length,
        first.velocity,
        first.acceleration,
        first.curvature,
        first.steering,

        middle.trajectory_index,
        middle.arc_length,
        middle.velocity,
        middle.acceleration,
        middle.curvature,

        last.trajectory_index,
        last.arc_length,
        last.velocity,
        last.acceleration,
        last.curvature,
        last.steering);
    }


    // ========================================================
    // 21. Current tracking-state diagnostics
    // ========================================================

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,

      "\n"
      "Integrated MPC tracking state:\n"
      "  nearest index = %zu\n"
      "  e_y           = %+.4f m\n"
      "  e_psi         = %+.4f rad\n"
      "  e_v           = %+.4f m/s\n"
      "  delta         = %+.4f rad\n"
      "  a             = %+.4f m/s^2\n"
      "  v_ref         = %+.4f m/s\n"
      "  a_ref         = %+.4f m/s^2\n"
      "  kappa_ref     = %+.6f 1/m\n"
      "  delta_ref     = %+.4f rad",

      tracking_state->nearest_index,

      tracking_state->lateral_error,
      tracking_state->heading_error,
      tracking_state->velocity_error,

      tracking_state->current_steer,
      tracking_state->current_acceleration,

      tracking_state->reference_velocity,
      tracking_state->reference_acceleration,

      tracking_state->reference_curvature,
      tracking_state->reference_steer);


    // ========================================================
    // 22. Publishing guard
    //
    // KEEP FALSE DURING 9B.
    //
    // The QP has now been solved, but we deliberately do not
    // let the optimal command control the vehicle yet.
    // ========================================================

    
      // ========================================================
// 23. Closed-loop publishing
// ========================================================

if (!publish_enabled_) {
  return;
}


// ========================================================
// 23.1 Initialize previous command
//
// Do this only immediately before the first real command
// can be published.
// ========================================================

if (!previous_command_initialized_) {
  previous_steering_command_ =
    std::clamp(
      x0(3),
      -max_steering_angle_,
      max_steering_angle_);

  previous_acceleration_command_ =
    std::clamp(
      x0(4),
      min_acceleration_,
      max_acceleration_);

  previous_command_initialized_ =
    true;

  RCLCPP_INFO(
    get_logger(),
    "Initialized MPC command state: "
    "steer=%+.6f rad, accel=%+.6f m/s^2",
    previous_steering_command_,
    previous_acceleration_command_);
}


// ========================================================
// 23.2 Solver fallback
//
// If the QP result is not safe to use, publish a
// conservative hold command instead of an invalid MPC
// solution.
// ========================================================

if (!solution_valid_for_control) {
  RCLCPP_ERROR_THROTTLE(
    get_logger(),
    *get_clock(),
    1000,
    "MPC solution rejected. "
    "status=%s, finite=%s, max_violation=%.6e. "
    "Publishing hold command.",
    solver_status.c_str(),
    optimal_solution_finite ? "true" : "false",
    maximum_constraint_violation);

  autoware_control_msgs::msg::Control fallback_cmd;

  const auto stamp =
    now();

  fallback_cmd.stamp =
    stamp;

  // ---------------- Lateral ----------------

  fallback_cmd.lateral.stamp =
    stamp;

  fallback_cmd.lateral.steering_tire_angle =
    static_cast<float>(
      previous_steering_command_);

  fallback_cmd.lateral.steering_tire_rotation_rate =
    0.0F;

  fallback_cmd.lateral.is_defined_steering_tire_rotation_rate =
    true;

  // ---------------- Longitudinal ----------------

  fallback_cmd.longitudinal.stamp =
    stamp;

  /*
   * Hold approximately the current longitudinal velocity.
   */
  fallback_cmd.longitudinal.velocity =
    static_cast<float>(
      odometry_->
        twist.twist.linear.x);

  /*
   * Preserve the previous acceleration command rather than
   * introducing a sudden acceleration jump.
   */
  fallback_cmd.longitudinal.acceleration =
    static_cast<float>(
      previous_acceleration_command_);

  fallback_cmd.longitudinal.jerk =
    0.0F;

  fallback_cmd.longitudinal.is_defined_acceleration =
    true;

  fallback_cmd.longitudinal.is_defined_jerk =
    true;

  control_pub_->publish(
    fallback_cmd);

  return;
}


// ========================================================
// 23.3 Extract first receding-horizon MPC action
//
// U* =
// [delta_cmd_0, jerk_0,
//  delta_cmd_1, jerk_1,
//  ...]
//
// Only the first pair is applied.
// ========================================================

const double raw_steering_command =
  optimal_input(0);

const double raw_jerk_command =
  optimal_input(1);


// ========================================================
// 23.4 Hard-clamp jerk
//
// OSQP numerical tolerances can create values such as
// 2.00012 even when the mathematical constraint is 2.0.
//
// Never send such numerical overshoot downstream.
// ========================================================

const double jerk_command =
  std::clamp(
    raw_jerk_command,
    min_jerk_,
    max_jerk_);


// ========================================================
// 23.5 Hard-clamp first steering command
//
// We enforce BOTH:
//   1. absolute steering bound
//   2. first-step steering-rate bound
//
// even after OSQP, so numerical solver tolerance can never
// generate an out-of-range command.
// ========================================================

const double max_first_steering_rate =
  mpc_constraints.
    steering_rate_limit(0);

const double max_first_steering_change =
  max_first_steering_rate *
  ctrl_period_;

const double steering_rate_lower =
  previous_steering_command_ -
  max_first_steering_change;

const double steering_rate_upper =
  previous_steering_command_ +
  max_first_steering_change;

double steering_command =
  std::clamp(
    raw_steering_command,
    steering_rate_lower,
    steering_rate_upper);

steering_command =
  std::clamp(
    steering_command,
    -max_steering_angle_,
    max_steering_angle_);


// ========================================================
// 23.6 Steering-rate command
// ========================================================

const double steering_rotation_rate =
  (
    steering_command -
    previous_steering_command_
  ) /
  ctrl_period_;


// ========================================================
// 23.7 Convert MPC jerk into acceleration command
//
// IMPORTANT:
//
// The MPC prediction horizon uses prediction_dt = 0.1 s,
// but the real controller publishes every ctrl_period =
// 0.03 s.
//
// Therefore the actually published acceleration command
// advances by:
//
//   a_cmd(k+1) = a_cmd(k) + j0 * ctrl_period
//
// This preserves the requested jerk in the ACTUAL control
// loop.
//
// Example:
//
//   j0 = 2.0 m/s^3
//   ctrl_period = 0.03 s
//
// gives:
//
//   Delta a = 0.06 m/s^2
//
// per published control cycle.
// ========================================================

double acceleration_command =
  previous_acceleration_command_ +
  jerk_command *
  ctrl_period_;

acceleration_command =
  std::clamp(
    acceleration_command,
    min_acceleration_,
    max_acceleration_);


// ========================================================
// 23.8 Construct Autoware control message
// ========================================================

autoware_control_msgs::msg::Control cmd;

const auto stamp =
  now();

cmd.stamp =
  stamp;


// ---------------- Lateral command ----------------

cmd.lateral.stamp =
  stamp;

cmd.lateral.steering_tire_angle =
  static_cast<float>(
    steering_command);

cmd.lateral.steering_tire_rotation_rate =
  static_cast<float>(
    steering_rotation_rate);

cmd.lateral.is_defined_steering_tire_rotation_rate =
  true;


// ---------------- Longitudinal command ----------------

cmd.longitudinal.stamp =
  stamp;

/*
 * Velocity remains the planner/reference velocity target.
 *
 * Acceleration and jerk are generated by our integrated MPC.
 */
cmd.longitudinal.velocity =
  static_cast<float>(
    tracking_state->
      reference_velocity);

cmd.longitudinal.acceleration =
  static_cast<float>(
    acceleration_command);

cmd.longitudinal.jerk =
  static_cast<float>(
    jerk_command);

cmd.longitudinal.is_defined_acceleration =
  true;

cmd.longitudinal.is_defined_jerk =
  true;


// ========================================================
// 23.9 Publish
// ========================================================

control_pub_->publish(
  cmd);


// ========================================================
// 23.10 Save the command that was ACTUALLY published
//
// These values are used during the next control iteration.
// ========================================================

previous_steering_command_ =
  steering_command;

previous_acceleration_command_ =
  acceleration_command;


// ========================================================
// 23.11 Closed-loop diagnostic
// ========================================================

RCLCPP_INFO_THROTTLE(
  get_logger(),
  *get_clock(),
  500,

  "\n"
  "Integrated MPC CLOSED LOOP:\n"
  "  raw steer       = %+.6f rad\n"
  "  sent steer      = %+.6f rad\n"
  "  steer rate      = %+.6f rad/s\n"
  "  raw jerk        = %+.6f m/s^3\n"
  "  sent jerk       = %+.6f m/s^3\n"
  "  sent accel      = %+.6f m/s^2\n"
  "  target velocity = %+.6f m/s\n"
  "  e_y             = %+.6f m\n"
  "  e_psi           = %+.6f rad\n"
  "  e_v             = %+.6f m/s",

  raw_steering_command,
  steering_command,
  steering_rotation_rate,

  raw_jerk_command,
  jerk_command,

  acceleration_command,

  tracking_state->
    reference_velocity,

  tracking_state->
    lateral_error,

  tracking_state->
    heading_error,

  tracking_state->
    velocity_error);


  }


  // ==========================================================
  // Parameters
  // ==========================================================

  double ctrl_period_{0.03};

  bool publish_enabled_{false};

  double wheel_base_{2.79};

  double steer_tau_{0.27};

  double nearest_dist_threshold_{3.0};

  double nearest_yaw_threshold_{1.046};


  // Prediction

  int prediction_horizon_{50};

  double prediction_dt_{0.1};

  double min_prediction_length_{5.0};

  double minimum_prediction_velocity_{0.1};


  // Cost weights

  double weight_lat_error_{1.0};

  double weight_heading_error_squared_vel_{0.3};

  double weight_velocity_error_{1.0};

  double weight_acceleration_error_{0.1};

  double weight_steering_input_{1.0};

  double weight_steering_input_squared_vel_{0.25};

  double weight_jerk_{0.1};

  double terminal_weight_lat_error_{1.0};

  double terminal_weight_heading_error_{0.1};

  double terminal_weight_velocity_error_{1.0};


  // Constraints

  double max_steering_angle_{0.7};

  double min_acceleration_{-5.0};

  double max_acceleration_{3.0};

  double min_jerk_{-5.0};

  double max_jerk_{2.0};

  std::vector<double>
    curvature_for_steer_rate_{
      0.001,
      0.002,
      0.01};

  std::vector<double>
    steer_rate_deg_by_curvature_{
      40.0,
      50.0,
      60.0};

  std::vector<double>
    velocity_for_steer_rate_{
      10.0,
      15.0,
      20.0};

  std::vector<double>
    steer_rate_deg_by_velocity_{
      60.0,
      50.0,
      40.0};




    // ==========================================================
// Previous actually published MPC command
//
// Needed for:
//   - first steering-rate constraint
//   - jerk-consistent acceleration command
//
// These are initialized from the measured vehicle state
// before the first MPC command is published.
// ==========================================================

  bool previous_command_initialized_{false};

  double previous_steering_command_{0.0};

  double previous_acceleration_command_{0.0};  

  // ==========================================================
  // Builders
  // ==========================================================

  std::unique_ptr<
    autoware::integrated_mpc::
      TrackingStateBuilder>
    tracking_state_builder_;

  std::unique_ptr<
    autoware::integrated_mpc::
      PredictionReferenceBuilder>
    prediction_reference_builder_;

  std::unique_ptr<
    autoware::integrated_mpc::
      IntegratedPredictionBuilder>
    integrated_prediction_builder_;


  // ==========================================================
  // Latest subscribed inputs
  // ==========================================================

  autoware_planning_msgs::msg::Trajectory::
    SharedPtr trajectory_;

  nav_msgs::msg::Odometry::
    SharedPtr odometry_;

  autoware_vehicle_msgs::msg::SteeringReport::
    SharedPtr steering_;

  geometry_msgs::msg::AccelWithCovarianceStamped::
    SharedPtr accel_;

  autoware_adapi_v1_msgs::msg::OperationModeState::
    SharedPtr operation_mode_;


  // ==========================================================
  // ROS interfaces
  // ==========================================================

  rclcpp::Subscription<
    autoware_planning_msgs::msg::Trajectory>::
    SharedPtr trajectory_sub_;

  rclcpp::Subscription<
    nav_msgs::msg::Odometry>::
    SharedPtr odometry_sub_;

  rclcpp::Subscription<
    autoware_vehicle_msgs::msg::SteeringReport>::
    SharedPtr steering_sub_;

  rclcpp::Subscription<
    geometry_msgs::msg::AccelWithCovarianceStamped>::
    SharedPtr accel_sub_;

  rclcpp::Subscription<
    autoware_adapi_v1_msgs::msg::OperationModeState>::
    SharedPtr operation_mode_sub_;

  rclcpp::Publisher<
    autoware_control_msgs::msg::Control>::
    SharedPtr control_pub_;

  rclcpp::Publisher<
    autoware_planning_msgs::msg::Trajectory>::
    SharedPtr predicted_trajectory_pub_;

  rclcpp::TimerBase::SharedPtr
    timer_;
};


// ============================================================
// Main
// ============================================================

int main(
  int argc,
  char ** argv)
{
  rclcpp::init(
    argc,
    argv);

  rclcpp::spin(
    std::make_shared<
      IntegratedMpcNode>());

  rclcpp::shutdown();

  return 0;
}
