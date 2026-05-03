#pragma once
#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <array>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "common/dog_data_bridge.hpp"
#include "common/debug_manager.hpp"
#include "state_estimation/KalmanFilterEstimator.hpp"
#include "state_estimation/StateEstimatorBase.hpp"
#include "state_estimation/TopicStateEstimator.hpp"
#include "wbc/WbcBase.hpp"
#include "wbc/WeightedWbc.hpp"
#include "nmpc/NmpcController.hpp"
#include "ocs2_legged_robot/LeggedRobotInterface.h"

namespace dog_controllers
{
    using namespace ocs2;
    using namespace ocs2::legged_robot;
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class DogNmpcWbcController : public controller_interface::ControllerInterface
    {
    public:
        DogNmpcWbcController() = default;
        virtual ~DogNmpcWbcController();
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        CallbackReturn on_init() override;
        CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::return_type update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        virtual void init_real_or_god();

    protected:
        std::unique_ptr<DogDataBridge> bridge_;
        std::unique_ptr<DebugManager> debug_manager_;
        std::shared_ptr<LeggedRobotInterface> robot_interface_;
        std::unique_ptr<StateEstimatorBase> state_estimator_;
        std::unique_ptr<WbcBase> wbc_;
        std::unique_ptr<NmpcController> nmpc_controller_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        std::shared_ptr<rclcpp::Node> ros_interface_node_;
        std::unique_ptr<std::thread> ros_interface_thread_;
        benchmark::RepeatedTimer mainLoopTimer_;
        bool isFirstUpdate_ = 1;
        bool isSim_ = false;
        bool useGaitContact_ = true;
        std::string urdfFile;
        std::string taskFile;
        std::string referenceFile;
        std::array<scalar_t, 3> standUpPhase1Start_{0.0, 0.0, 0.0};
        std::array<scalar_t, 3> standUpPhase1Goal_{0.0, 1.45, 0.0};
        std::array<scalar_t, 3> standUpPhase2Goal_{0.0, 1.9, -0.94};
        std::array<scalar_t, 3> standUpKp_{15.0, 15.0, 25.0};
        std::array<scalar_t, 3> standUpKd_{2.0, 2.0, 2.0};

        enum class ControlState
        {
            JOINT_STANDUP,
            NMPC_ACTIVE
        };
        ControlState currentState_ = ControlState::JOINT_STANDUP;
        scalar_t standUpTimer_ = 0.0;
        scalar_t standUpDuration_ = 2.0;
    };
    class DogNmpcWbcController_God : public DogNmpcWbcController
    {
        void init_real_or_god() override;
    };
}
