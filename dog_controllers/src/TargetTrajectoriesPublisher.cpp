#include "TargetTrajectoriesPublisher.h"
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Geometry>

namespace dog_controllers
{

  TargetTrajectoriesPublisher::TargetTrajectoriesPublisher(const rclcpp::NodeOptions &options)
      : Node("target_trajectories_publisher", options), DEFAULT_JOINT_STATE_(12)
  {
    RCLCPP_INFO(this->get_logger(), "\033[1;36m====================================================\033[0m");
    RCLCPP_INFO(this->get_logger(), "\033[1;36m[ 初始化开始 ] 🚀 TargetTrajectoriesPublisher\033[0m");

    std::string pkg_share_path = ament_index_cpp::get_package_share_directory("dog_bringup");
    std::string taskFile = pkg_share_path + "/config/cdut_dog/description/task.info";
    std::string referenceFile = pkg_share_path + "/config/cdut_dog/description/reference.info";

    loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT_);
    loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE_);
    loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY_);
    loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY_);
    loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET_);

    RCLCPP_INFO(this->get_logger(), "\033[1;32m🎯 [TargetTrajectories] 轨迹生成参数已就绪:\033[0m");
    RCLCPP_INFO(this->get_logger(), "\033[1;32m  ├─ 期望站立高度 (COM)     : \033[0m%.3f m", COM_HEIGHT_);
    RCLCPP_INFO(this->get_logger(), "\033[1;32m  ├─ 规划线速度限制         : \033[0m%.3f m/s", TARGET_DISPLACEMENT_VELOCITY_);
    RCLCPP_INFO(this->get_logger(), "\033[1;32m  ├─ 规划角速度限制         : \033[0m%.3f rad/s", TARGET_ROTATION_VELOCITY_);
    RCLCPP_INFO(this->get_logger(), "\033[1;32m  ├─ MPC 预测时域 (Horizon) : \033[0m%.3f s", TIME_TO_TARGET_);
    RCLCPP_INFO(this->get_logger(), "\033[1;32m  └─ 默认关节状态维度       : \033[0m%ld 维向量", DEFAULT_JOINT_STATE_.size());

    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

    initTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(1),
        [this]()
        {
          targetTrajectoriesPublisher_ = std::make_unique<TargetTrajectoriesRosPublisher>(
              this->shared_from_this(), topicPrefix);
          referencePathPub_ = this->create_publisher<nav_msgs::msg::Path>("target_reference_path", 1);
          this->initTimer_->cancel();
        });

    observationSub_ = this->create_subscription<ocs2_msgs::msg::MpcObservation>(
        topicPrefix + "_mpc_observation", 1,
        [this](const ocs2_msgs::msg::MpcObservation::SharedPtr msg)
        {
          std::lock_guard<std::mutex> lock(latestObservationMutex_);
          latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
        });

    goalSub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
          RCLCPP_INFO(this->get_logger(), "\033[1;36m🚩 [Goal] 接收到新目标点: [x: %.2f, y: %.2f] Frame: %s\033[0m",
                      msg->pose.position.x, msg->pose.position.y, msg->header.frame_id.c_str());
          if (latestObservation_.time == 0.0)
            return;

          geometry_msgs::msg::PoseStamped poseInOdom;
          try
          {
            poseInOdom = tfBuffer_->transform(*msg, "odom", tf2::durationFromSec(0.1));
          }
          catch (const tf2::TransformException &ex)
          {
            RCLCPP_WARN(this->get_logger(), "TF Transform failed: %s", ex.what());
            RCLCPP_WARN(this->get_logger(), "-------TF超时----------TF超时-----------TF超时------------");
            return;
          }

          vector_t cmdGoal = vector_t::Zero(6);
          cmdGoal[0] = poseInOdom.pose.position.x;
          cmdGoal[1] = poseInOdom.pose.position.y;
          cmdGoal[2] = poseInOdom.pose.position.z;

          auto &q = poseInOdom.pose.orientation;
          Eigen::Vector3d rpy = Eigen::Quaterniond(q.w, q.x, q.y, q.z).toRotationMatrix().eulerAngles(0, 1, 2);
          cmdGoal[3] = rpy.z(); // Yaw
          cmdGoal[4] = rpy.y(); // Pitch
          cmdGoal[5] = rpy.x(); // Roll
          const auto trajectories = goalToTargetTrajectories(cmdGoal, latestObservation_);
          targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
          publishReferencePath(trajectories);
        });

    cmdVelSub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          if (latestObservation_.time == 0.0)
            return;

          vector_t cmdVel = vector_t::Zero(4);
          cmdVel[0] = msg->linear.x;
          cmdVel[1] = msg->linear.y;
          cmdVel[2] = msg->linear.z;
          cmdVel[3] = msg->angular.z;

          const auto trajectories = cmdVelToTargetTrajectories(cmdVel, latestObservation_);
          targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
        });
    RCLCPP_INFO(this->get_logger(), "\033[1;32m[ 初始化完成 ] ✅ TargetTrajectoriesPublisher\033[0m");
    RCLCPP_INFO(this->get_logger(), "\033[1;32m====================================================\033[0m");
  }

  scalar_t TargetTrajectoriesPublisher::estimateTimeToTarget(const vector_t &desiredBaseDisplacement)
  {
    const scalar_t rotationTime = std::abs(desiredBaseDisplacement(3)) / TARGET_ROTATION_VELOCITY_;
    const scalar_t displacement = desiredBaseDisplacement.head<2>().norm();
    const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY_;
    return std::max(rotationTime, displacementTime);
  }

  TargetTrajectories TargetTrajectoriesPublisher::targetPoseToTargetTrajectories(
      const vector_t &targetPose, const SystemObservation &observation, const scalar_t &targetReachingTime)
  {

    const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

    vector_t currentPose = observation.state.segment<6>(6);
    currentPose(2) = COM_HEIGHT_;
    currentPose(4) = 0;
    currentPose(5) = 0;

    vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
    stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE_;
    stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE_;

    const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));
    return {timeTrajectory, stateTrajectory, inputTrajectory};
  }

  TargetTrajectories TargetTrajectoriesPublisher::goalToTargetTrajectories(const vector_t &goal, const SystemObservation &observation)
  {
    const vector_t currentPose = observation.state.segment<6>(6);
    vector_t targetPose = vector_t::Zero(6);
    targetPose << goal(0), goal(1), COM_HEIGHT_, goal(3), 0, 0;

    const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
    return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  }

  TargetTrajectories TargetTrajectoriesPublisher::cmdVelToTargetTrajectories(const vector_t &cmdVel, const SystemObservation &observation)
  {
    const vector_t currentPose = observation.state.segment<6>(6);
    const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
    vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

    vector_t targetPose = vector_t::Zero(6);
    targetPose << currentPose(0) + cmdVelRot(0) * TIME_TO_TARGET_,
        currentPose(1) + cmdVelRot(1) * TIME_TO_TARGET_,
        COM_HEIGHT_,
        currentPose(3) + cmdVel(3) * TIME_TO_TARGET_, 0, 0;

    const scalar_t targetReachingTime = observation.time + TIME_TO_TARGET_;
    auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);

    trajectories.stateTrajectory[0].head(3) = cmdVelRot;
    trajectories.stateTrajectory[1].head(3) = cmdVelRot;
    return trajectories;
  }

  void TargetTrajectoriesPublisher::publishReferencePath(const TargetTrajectories &trajectories)
  {
    if (trajectories.stateTrajectory.empty())
      return;

    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = this->get_clock()->now();

    const vector_t &startState = trajectories.stateTrajectory.front();
    const vector_t &endState = trajectories.stateTrajectory.back();

    int numPoints = 20;
    for (int i = 0; i <= numPoints; ++i)
    {
      double alpha = static_cast<double>(i) / numPoints;

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = (1.0 - alpha) * startState(6) + alpha * endState(6);
      pose.pose.position.y = (1.0 - alpha) * startState(7) + alpha * endState(7);
      pose.pose.position.z = (1.0 - alpha) * startState(8) + alpha * endState(8);

      pose.pose.orientation.w = 1.0;

      path.poses.push_back(pose);
    }

    referencePathPub_->publish(path);
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(dog_controllers::TargetTrajectoriesPublisher)