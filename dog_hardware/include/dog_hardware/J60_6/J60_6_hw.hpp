#pragma once

#include <atomic>
#include <thread>
#include <vector>
#include <string>
#include <array>
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
// 包含你的 SDK
#include "dog_hardware/J60_6/deep_motor_sdk.h"

namespace dog_hardware
{

    using CallbackReturn = hardware_interface::CallbackReturn;

    struct JointData
    {
        std::string name;
        int motor_id;
        double kp, kd, ff, pos_des, vel_des;
        double drv_temp, mtr_temp, err_code, pos, vel, eff;
        MotorCMD *cmd_obj;
        MotorDATA *data_obj;
    };

    struct CanBusWorker
    {
        std::string interface;
        DrMotorCan *can_handle = nullptr;
        std::vector<JointData *> joints;
        std::thread thread_handle;
        std::atomic<bool> is_running{false};
        std::atomic<bool> is_online{false};
    };

    class J60_6_hw : public hardware_interface::SystemInterface
    {
    public:
        CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
        hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

    private:
        // 4路 CAN
        std::array<CanBusWorker, 4> can_workers_;
        // 1路 IMU
        std::thread imu_thread_;
        std::atomic<bool> imu_running_{false};

        struct
        {
            std::atomic<double> ori[4], ang_vel[3], lin_acc[3];
        } shared_imu_;

        void CanWorkerFunc(int bus_idx);
        void ImuWorkerFunc(std::string port);

        std::vector<JointData> all_joints_;
    };

} // namespace dog_hardware