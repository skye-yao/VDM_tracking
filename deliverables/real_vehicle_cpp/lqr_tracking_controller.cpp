// Completed 3-state vehicle-template LQR; internal angles are radians.
#include <driverless/tracking_controller/lqr_tracking_controller.h>
#include <driverless/utils.h>
#include <algorithm>
#include <cmath>
#include <limits>

#define __NAME__ "lqr_tracking_controller"

using namespace dcom;

LQRTrackingController::LQRTrackingController()
    : TrackingControllerBase(), vehicle_model_(nullptr) {}

bool LQRTrackingController::init(ros::NodeHandle &nh,
                                 ros::NodeHandle &nh_private) {
  nh_private.param<int>("lqr_iteration_range", iterationRange_, 100);
  nh_private.param<float>("lqr_QValue", QValue_, 3.0);
  nh_private.param<float>("lqr_RValue_", RValue_, 2.0);
  nh_private.param<double>("lqr_iteration_eps", iterationEps_, 1.0e-4);
  nh_private.param<float>("max_side_accel", max_side_accel_, 1.0);
  if (!vehicle_model_) {
    ROS_ERROR("[%s] Set vehicle model failed!", __NAME__);
    return false;
  }
  if (!vehicle_params_.validity || iterationRange_ <= 0 || !std::isfinite(iterationEps_) || iterationEps_ <= 0 ||
      !std::isfinite(QValue_) || QValue_ <= 0 ||
      !std::isfinite(RValue_) || RValue_ <= 0 ||
      !std::isfinite(max_side_accel_) || max_side_accel_ <= 0 ||
      !std::isfinite(vehicle_params_.wheel_base) || vehicle_params_.wheel_base <= 0 ||
      !std::isfinite(vehicle_params_.min_roadwheel_angle) ||
      !std::isfinite(vehicle_params_.max_roadwheel_angle) ||
      vehicle_params_.min_roadwheel_angle > 0 || vehicle_params_.max_roadwheel_angle < 0 ||
      !std::isfinite(vehicle_params_.max_deceleration) ||
      vehicle_params_.max_deceleration <= 0) return false;

  // 权重矩阵配置保留，学生可以通过调整参数观察控制效果。
  Q_.resize(3, 3);
  Q_ << QValue_, 0, 0, 0, QValue_, 0, 0, 0, 3;
  R_.resize(2, 2);
  R_ << RValue_, 0.0, 0.0, RValue_;
  return vehicle_params_.validity;
}

TrackerResult LQRTrackingController::step(
    const LocalPath::ConstPtr local_path,
    const VehicleState::ConstPtr vehicle_state) {
  const auto stop = [&]() -> TrackerResult {
    result_ = TrackerResult{};
    if (vehicle_params_.validity &&
        std::isfinite(vehicle_params_.max_deceleration) &&
        vehicle_params_.max_deceleration > 0)
      result_.Decel = vehicle_params_.max_deceleration;
    return result_;
  };
  if (!local_path || !vehicle_state || !vehicle_model_ || local_path->size() == 0 ||
      static_cast<size_t>(local_path->vehicle_pose_index) >= local_path->size())
    return stop();
  if (!std::isfinite(vehicle_state->pose.x) ||
      !std::isfinite(vehicle_state->pose.y) ||
      !std::isfinite(vehicle_state->pose.yaw) ||
      !std::isfinite(vehicle_state->speed) || vehicle_state->speed < 0 ||
      !std::isfinite(vehicle_state->road_wheel_angle) ||
      !std::isfinite(max_speed_kph_) || max_speed_kph_ < 0)
    return stop();
  for (size_t i = local_path->vehicle_pose_index; i < local_path->size(); ++i) {
    const auto &point = local_path->at(i);
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.yaw) || !std::isfinite(point.curvature)) return stop();
  }

  float expect_speed = max_speed_kph_;
  float expect_decel = 0.0f;
  const Pose &ego_pose = vehicle_state->pose;
  const float ego_spd = vehicle_state->speed;
  const float ego_spd_ms = ego_spd / 3.6f;
  const float road_wheel_angle = vehicle_state->road_wheel_angle;
  const LocalPathPoint nearest_path_point =
      local_path->at(local_path->vehicle_pose_index);

  // 车辆模型线性化由已有 vehicle_model_ 完成，学生无需实现。
  Eigen::MatrixXd veh_state(1, 4);
  // Only regularize the linearization speed at startup (as in Python LQR).
  // Braking and speed limits below still use measured speed.
  veh_state << ego_pose.x, ego_pose.y, ego_pose.yaw, std::max(0.5f, ego_spd_ms);
  const float curvature = nearest_path_point.curvature;
  const double ref_front_wheel_angle =
      atan2(vehicle_params_.wheel_base * curvature, 1.0);  // radians
  const double ref_yaw = nearest_path_point.yaw;
  std::vector<double> ref_value(2);
  ref_value[0] = ref_front_wheel_angle;
  ref_value[1] = ref_yaw;
  const std::vector<Eigen::MatrixXd> ab =
      vehicle_model_->update(veh_state, ref_value);
  if (ab.size() != 2 || ab[0].rows() != 3 || ab[0].cols() != 3 ||
      ab[1].rows() != 3 || ab[1].cols() != 2 ||
      !ab[0].allFinite() || !ab[1].allFinite()) return stop();
  A_ = ab[0];
  B_ = ab[1];

  // LQR-1: global position error in metres and current-reference yaw in radians.
  float lat_err = dis2ray(nearest_path_point, ego_pose);
  const double yaw_difference = ego_pose.yaw - nearest_path_point.yaw;
  double yaw_error = std::atan2(std::sin(yaw_difference), std::cos(yaw_difference));
  Eigen::MatrixXd X(3, 1);
  X << ego_pose.x - nearest_path_point.x, ego_pose.y - nearest_path_point.y, yaw_error;
  if (!X.allFinite() || !std::isfinite(lat_err)) return stop();

  // LQR-2: model and feedback use radians; convert ONCE at the output boundary.
  const double target_angle_increment = getTargetAngleIncreament(X);
  if (!std::isfinite(target_angle_increment)) return stop();
  double expect_angle = (ref_front_wheel_angle + target_angle_increment) * 180.0 / M_PI;
  if (!std::isfinite(expect_angle)) return stop();
  // Saturate the total angle, preserving corrective direction.
  expect_angle = std::max(static_cast<double>(vehicle_params_.min_roadwheel_angle),
      std::min(static_cast<double>(vehicle_params_.max_roadwheel_angle), expect_angle));

  // 以下安全约束与纵向规划均调用现有工程函数，不需要学生实现。
  expect_angle = limitRoadwheelAngleBySpeed(expect_angle, ego_spd_ms,
                                             max_side_accel_);
  expect_speed =
      limitSpeedByCurrentRoadwheelAngle(expect_speed, road_wheel_angle);

  const float curvature_search_distance =
      ego_spd_ms * ego_spd_ms / 2.0f + 10.0f;
  const auto curvature_info = maxCurvatureInRange(
      *local_path, local_path->vehicle_pose_index, curvature_search_distance);
  const float max_speed_by_curve =
      maxTolarateSpeedByCurvature(curvature_info.first, max_side_accel_);
  if (max_speed_by_curve < expect_speed) {
    expect_speed = max_speed_by_curve;
    const SpeedControlInfo ctrl_info = calculateCurveBrakeDecel(
        ego_spd_ms, max_speed_by_curve / 3.6f, curvature_info.second,
        vehicle_params_.max_deceleration);
    if (ctrl_info.decel > expect_decel) {
      expect_decel = ctrl_info.decel;
      expect_speed = ctrl_info.max_spd;
    }
  }

  if (!std::isfinite(expect_angle) || !std::isfinite(expect_speed) ||
      !std::isfinite(expect_decel)) return stop();
  expect_speed = std::max(0.0f, std::min(max_speed_kph_, expect_speed));
  result_.Decel = expect_decel;
  result_.Speed = expect_speed;
  result_.SteerAngle = expect_angle;
  result_.LateralErr = lat_err;
  result_.YawErr = -yaw_error;  // diagnostics: reference-current, matching PP (rad)
  return result_;
}

// LQR-3: u = -solve(R + B'PB, B'PA) X; input index 1 is steering (rad).
double LQRTrackingController::getTargetAngleIncreament(Eigen::MatrixXd &X) {
  const Eigen::MatrixXd P = calRicatti();
  if (!P.allFinite()) return std::numeric_limits<double>::quiet_NaN();
  const Eigen::MatrixXd S = R_ + B_.transpose() * P * B_;
  Eigen::LDLT<Eigen::MatrixXd> solver(S);
  if (solver.info() != Eigen::Success || !solver.isPositive())
    return std::numeric_limits<double>::quiet_NaN();
  const Eigen::MatrixXd K = solver.solve(B_.transpose() * P * A_);
  const Eigen::MatrixXd u = -K * X;
  if (solver.info() != Eigen::Success || !u.allFinite())
    return std::numeric_limits<double>::quiet_NaN();
  return u(1, 0);
}

// LQR-4: bounded Riccati iteration with an absolute convergence criterion.
Eigen::MatrixXd LQRTrackingController::calRicatti() {
  Eigen::MatrixXd P = Q_;
  for (int i = 0; i < iterationRange_; ++i) {
    const Eigen::MatrixXd S = R_ + B_.transpose() * P * B_;
    Eigen::LDLT<Eigen::MatrixXd> solver(S);
    if (!S.allFinite() || solver.info() != Eigen::Success || !solver.isPositive())
      return Eigen::MatrixXd::Constant(3, 3, std::numeric_limits<double>::quiet_NaN());
    const Eigen::MatrixXd gain = solver.solve(B_.transpose() * P * A_);
    Eigen::MatrixXd next = Q_ + A_.transpose() * P * A_ - A_.transpose() * P * B_ * gain;
    next = (0.5 * (next + next.transpose())).eval();
    if (solver.info() != Eigen::Success || !next.allFinite())
      return Eigen::MatrixXd::Constant(3, 3, std::numeric_limits<double>::quiet_NaN());
    const double difference = (next - P).cwiseAbs().maxCoeff();
    P = next;
    if (difference < iterationEps_) return P;
  }
  // A finite-horizon approximation, not a claim that the DARE has converged.
  ROS_WARN_THROTTLE(5.0, "LQR Riccati iteration limit reached; using finite-iteration gain");
  return P;
}
