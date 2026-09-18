// Completed PP template; input speed km/h, output steering degrees.
#include <driverless/tracking_controller/pp_tracking_controller.h>
#include <driverless/utils.h>
#include <algorithm>
#include <cmath>

using namespace dcom;

PurePursuitTrackingController::PurePursuitTrackingController()
    : TrackingControllerBase() {}

bool PurePursuitTrackingController::init(ros::NodeHandle &nh,
                                         ros::NodeHandle &nh_private) {
  // 参数、发布器和车辆参数校验由框架提供，无需学生实现。
  nh_private.param<float>("foreSightDis_speedCoefficient",
                          foreSightDis_speedCoefficient_, 1.8);
  nh_private.param<float>("foreSightDis_latErrCoefficient",
                          foreSightDis_latErrCoefficient_, -1.0);
  nh_private.param<float>("min_foresight_distance", min_foresight_distance_,
                          5.0);
  nh_private.param<float>("max_side_accel", max_side_accel_, 1.0);
  pub_track_point_ =
      nh.advertise<visualization_msgs::Marker>("pp_track_point", 1);
  return vehicle_params_.validity && std::isfinite(vehicle_params_.wheel_base) &&
      vehicle_params_.wheel_base > 0 &&
      std::isfinite(min_foresight_distance_) && min_foresight_distance_ > 0 &&
      std::isfinite(foreSightDis_speedCoefficient_) &&
      std::isfinite(foreSightDis_latErrCoefficient_) &&
      std::isfinite(max_side_accel_) && max_side_accel_ > 0 &&
      std::isfinite(vehicle_params_.min_roadwheel_angle) &&
      std::isfinite(vehicle_params_.max_roadwheel_angle) &&
      vehicle_params_.min_roadwheel_angle <= 0 &&
      vehicle_params_.max_roadwheel_angle >= 0 &&
      std::isfinite(vehicle_params_.max_deceleration) &&
      vehicle_params_.max_deceleration > 0;
}

TrackerResult PurePursuitTrackingController::step(
    const LocalPath::ConstPtr local_path,
    const VehicleState::ConstPtr vehicle_state) {
  // Clear every field: an invalid input must not reuse the previous command.
  const auto stop = [&]() -> TrackerResult {
    result_ = TrackerResult{};
    if (vehicle_params_.validity &&
        std::isfinite(vehicle_params_.max_deceleration) &&
        vehicle_params_.max_deceleration > 0)
      result_.Decel = vehicle_params_.max_deceleration;
    return result_;
  };
  if (!local_path || !vehicle_state || local_path->size() == 0 ||
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
        !std::isfinite(point.yaw) || !std::isfinite(point.curvature))
      return stop();
  }

  const Pose &ego_pose = vehicle_state->pose;
  const float ego_spd = vehicle_state->speed;
  const float ego_spd_ms = ego_spd / 3.6f;  // km/h -> m/s
  const float road_wheel_angle = vehicle_state->road_wheel_angle;
  const LocalPathPoint nearest_pose =
      local_path->at(local_path->vehicle_pose_index);

  // PP-1: signed lateral error from the existing geometry helper.
  float lat_err = dis2ray(nearest_pose, ego_pose);
  float abs_lat_err = fabs(lat_err);

  // PP-2: reference minus vehicle heading, radians in [-pi, pi].
  const double yaw_difference = nearest_pose.yaw - ego_pose.yaw;
  float yaw_err = std::atan2(std::sin(yaw_difference), std::cos(yaw_difference));

  // PP-3: retain the real-vehicle template's lookahead parameterization.
  disThreshold_ = std::max(min_foresight_distance_,
      foreSightDis_speedCoefficient_ * ego_spd_ms +
      foreSightDis_latErrCoefficient_ * abs_lat_err);

  // PP-4: initialize from the last point even if the search has no iterations.
  size_t target_index = local_path->size() - 1;
  auto dis_yaw = getDisAndYaw(local_path->at(target_index), ego_pose);
  for (size_t i = local_path->vehicle_pose_index + 1; i < local_path->size(); ++i) {
    target_index = i;
    dis_yaw = getDisAndYaw(local_path->at(i), ego_pose);
    if (dis_yaw.first >= disThreshold_) break;
  }
  if (!std::isfinite(dis_yaw.first) || !std::isfinite(dis_yaw.second) ||
      !std::isfinite(lat_err) || !std::isfinite(yaw_err)) return stop();

  // 可视化调用由工程提供，学生不需要实现。
  publishTrackPointMarker(local_path->at(target_index), ego_pose);

  // PP-5: actual target distance; the helper returns road-wheel degrees.
  float expect_angle = 0.0f;
  const double sin_theta = std::sin(dis_yaw.second - ego_pose.yaw);
  if (dis_yaw.first > 1.0e-6 && std::abs(sin_theta) > 1.0e-6) {
    const double radius = dis_yaw.first / (2.0 * sin_theta);
    expect_angle = generateRoadwheelAngleByRadius(vehicle_params_.wheel_base, radius);
  }
  expect_angle = std::max(vehicle_params_.min_roadwheel_angle,
      std::min(vehicle_params_.max_roadwheel_angle, expect_angle));

  // 以下纵向规划和安全约束均为已有功能函数，不需要学生实现。
  float expect_speed = max_speed_kph_;
  float expect_decel = 0.0f;
  if (abs_lat_err > 0.1f) {
    const float max_spd_by_lateral_error =
        dcom::computeYby2ends(0.1f, 6.5f, 1.5f, 2.5f, abs_lat_err);
    if (max_spd_by_lateral_error < expect_speed) {
      expect_speed = max_spd_by_lateral_error;
    }
  }

  expect_angle = limitRoadwheelAngleBySpeed(expect_angle, ego_spd_ms,
                                             max_side_accel_);
  expect_speed =
      limitSpeedByCurrentRoadwheelAngle(expect_speed, road_wheel_angle);

  const float curvature_search_distance =
      ego_spd_ms * ego_spd_ms / 2.0f + 10.0f;
  const auto curvature_info = maxCurvatureInRange(
      *local_path, local_path->vehicle_pose_index, curvature_search_distance);
  float max_speed_by_curve =
      maxTolarateSpeedByCurvature(curvature_info.first, max_side_accel_);
  if (max_speed_by_curve < vehicle_params_.min_speed) {
    max_speed_by_curve = vehicle_params_.min_speed;
  }
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
  result_.YawErr = yaw_err;
  return result_;
}

// 跟踪点 Marker 发布由框架提供，学生无需修改。
void PurePursuitTrackingController::publishTrackPointMarker(
    const Point &track_point, const Pose ego_pose) {
  if (pub_track_point_.getNumSubscribers() == 0) {
    return;
  }

  static visualization_msgs::Marker marker;
  static bool first = true;
  if (first) {
    first = false;
    marker.header.frame_id = "gps_link";
    marker.action = marker.ADD;
    marker.ns = "track";
    marker.id = 0;
    marker.type = marker.SPHERE;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.4;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.9;
    marker.lifetime = ros::Duration(0.5);
  }

  const Point point = global2local(ego_pose, track_point);
  marker.pose.position.x = point.x;
  marker.pose.position.y = point.y;
  pub_track_point_.publish(marker);
}
