#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <stdint.h>
#include <string>
#include <array>
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"

#include "dog_hardware/J60_6/deep_motor_sdk.h"

namespace dog_hardware
{

    using CallbackReturn = hardware_interface::CallbackReturn;

    struct JointData
    {
        std::string name;
        int motor_id;
        double kp = 0.0, kd = 0.0, ff = 0.0, pos_des = 0.0, vel_des = 0.0;
        double drv_temp = 0.0, mtr_temp = 0.0, err_code = 0.0, pos = 0.0, vel = 0.0, eff = 0.0;
        MotorCMD *cmd_obj;
        MotorDATA *data_obj;
        MotorDATA *temp_data_obj;
        std::mutex cmd_mutex_;
        std::mutex data_mutex_;
        double joint_sign;
    };

    struct CanBusWorker
    {
        std::string interface;
        DrMotorCan *can_handle = nullptr;
        std::mutex can_mutex_;
        std::vector<JointData *> joints;
        std::thread thread_handle;
        std::atomic<bool> is_running{true};
    };
    struct ImuData
    {
        double ori[4];     // x, y, z, w
        double ang_vel[3]; // x, y, z
        double lin_acc[3]; // x, y, z
    };
    class J60_6_hw : public hardware_interface::SystemInterface
    {
    public:
        ~J60_6_hw() override = default;
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
        std::vector<std::unique_ptr<JointData>> all_joints_;

        // 1路 IMU
        std::thread imu_thread_;
        std::atomic<bool> imu_running_{false};
        ImuData Imudata, temp_ImuData;
        std::mutex imu_mutex_;
        float fAcc[3], fGyro[3], fq[4];

        double dummy_contact[4] = {0.0, 0.0, 0.0, 0.0};

        bool overall_success = true;

        void CanWorkerFunc(int bus_idx);
        void ImuWorkerFunc(std::string port);
    };

} // namespace dog_hardware