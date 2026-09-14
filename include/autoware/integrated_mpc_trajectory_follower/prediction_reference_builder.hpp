#pragma once

#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <autoware_planning_msgs/msg/trajectory_point.hpp>

#include <cstddef>
#include <vector>

namespace autoware::integrated_mpc
{

struct PredictionReferencePoint
{
  std::size_t trajectory_index{0};

  // Arc length measured from the current/nearest reference point.
  double arc_length{0.0};

  double velocity{0.0};
  double acceleration{0.0};

  double curvature{0.0};
  double steering{0.0};
};


struct PredictionReferenceHorizon
{
  std::vector<PredictionReferencePoint> points;

  // Actual prediction sampling time used for this horizon.
  double prediction_dt{0.0};

  // Spatial distance between current reference point
  // and final prediction reference point.
  double total_arc_length{0.0};
};


class PredictionReferenceBuilder
{
public:
  PredictionReferenceBuilder(
    double wheel_base,
    std::size_t prediction_horizon,
    double nominal_prediction_dt,
    double min_prediction_length,
    double minimum_prediction_velocity);

  PredictionReferenceHorizon build(
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    std::size_t start_index) const;

private:
  std::vector<double> calculateCumulativeArcLength(
    const autoware_planning_msgs::msg::Trajectory & trajectory) const;

  
  std::vector<double> calculateRelativeTime(
	  const autoware_planning_msgs::msg::Trajectory & trajectory,
	  const std::vector<double> & cumulative_arc_length,
	  std::size_t start_index) const;
  
  
double calculatePredictionDt(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  std::size_t start_index,
  const std::vector<double> & cumulative_arc_length,
  const std::vector<double> & relative_time) const;

PredictionReferencePoint interpolateReferenceAtTime(
  const autoware_planning_msgs::msg::Trajectory & trajectory,
  const std::vector<double> & cumulative_arc_length,
  const std::vector<double> & relative_time,
  std::size_t start_index,
  double target_time,
  double start_arc_length) const;

  double wheel_base_;
  std::size_t prediction_horizon_;

  double nominal_prediction_dt_;
  double min_prediction_length_;
  double minimum_prediction_velocity_;
};

}  // namespace autoware::integrated_mpc
