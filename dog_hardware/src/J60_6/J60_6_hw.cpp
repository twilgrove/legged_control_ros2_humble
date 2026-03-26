#include "dog_hardware/J60_6/J60_6_hw.hpp"

namespace dog_hardware
{

    CallbackReturn J60_6_hw::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::FAILURE;

        // 1. 初始化 4 条总线配置
        can_workers_[0].interface = "can_lf";
        can_workers_[1].interface = "can_lh";
        can_workers_[2].interface = "can_rf";
        can_workers_[3].interface = "can rh";

        // 2. 解析 URDF 中的关节并分配到对应的 CAN 总线
        for (const auto &joint : info.joints)
        {
            JointData jd;
            jd.name = joint.name;
            jd.motor_id = std::stoi(joint.parameters.at("motor_id"));
            int bus_id = std::stoi(joint.parameters.at("bus_id")); // 0-3

            jd.cmd_obj = MotorCMDCreate();
            jd.data_obj = MotorDATACreate();

            all_joints_.push_back(jd);
            if (bus_id >= 0 && bus_id < 4)
            {
                can_workers_[bus_id].joints.push_back(&all_joints_.back());
            }
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn J60_6_hw::on_activate(const rclcpp_lifecycle::State &)
    {
        // 启动 4 个 CAN 线程
        for (int i = 0; i < 4; ++i)
        {
            // 尝试创建硬件句柄，失败则报错
            can_workers_[i].can_handle = DrMotorCanCreate(can_workers_[i].interface.c_str(), false);
            if (!can_workers_[i].can_handle)
            {
                RCLCPP_ERROR(rclcpp::get_logger("J60_6_hw"), "无法打开设备: %s", can_workers_[i].interface.c_str());
                continue;
            }

            // 使能该总线下所有电机
            for (auto *j : can_workers_[i].joints)
            {
                SetNormalCMD(j->cmd_obj, j->motor_id, ENABLE_MOTOR);
                SendRecv(can_workers_[i].can_handle, j->cmd_obj, j->data_obj);
            }

            can_workers_[i].is_running = true;
            can_workers_[i].thread_handle = std::thread(&J60_6_hw::CanWorkerFunc, this, i);
        }

        // 启动 IMU 线程 (示例端口 /dev/ttyUSB0)
        imu_running_ = true;
        imu_thread_ = std::thread(&J60_6_hw::ImuWorkerFunc, this, "/dev/ttyUSB0");

        return CallbackReturn::SUCCESS;
    }

    // ================== 后台 IO 线程逻辑 ==================

    void J60_6_hw::CanWorkerFunc(int bus_idx)
    {
        auto &bus = can_workers_[bus_idx];
        uint32_t tick = 0;

        while (bus.is_running)
        {
            for (auto *j : bus.joints)
            {
                // 1. 每 1000 次循环（约1秒）穿插一次状态字获取
                if (tick % 1000 == 0)
                {
                    SetNormalCMD(j->cmd_obj, j->motor_id, GET_STATUS_WORD);
                    SendRecv(bus.can_handle, j->cmd_obj, j->data_obj);
                }

                // 2. 正常控制指令 (CONTROL_MOTOR)
                // 这里 j->cmd_obj 在 write() 中已经被主线程更新过了
                int ret = SendRecv(bus.can_handle, j->cmd_obj, j->data_obj);

                if (ret != kNoSendRecvError)
                {
                    bus.is_online = false; // 掉线逻辑
                }
                else
                {
                    bus.is_online = true;
                }
            }
            tick++;
            // 这里的频率由 SendRecv 的阻塞时间及电机数量决定
            // 3个电机串行约耗时 0.6ms-0.9ms，正好适配 1kHz
        }
    }

    void J60_6_hw::ImuWorkerFunc(std::string port)
    {
        // 优雅读取逻辑：伪代码
        while (imu_running_)
        {
            // 1. 打开串口并检查
            // 2. read() 获取数据流
            // 3. 寻找帧头，校验解析
            // 4. 使用 shared_imu_.ori[0].store(val) 原子存入数据
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    // ================== ROS 2 主线程读写 (1kHz) ==================

    hardware_interface::return_type J60_6_hw::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // 极速拷贝：从 IO 线程更新过的 data_obj 拷贝到 StateInterface 绑定的变量
        for (auto &j : all_joints_)
        {
            j.pos = j.data_obj->position_;
            j.vel = j.data_obj->velocity_;
            j.eff = j.data_obj->torque_;
            j.err_code = (double)j.data_obj->error_;
            // 温度标志位切换
            if (j.data_obj->flag_ == kMotorTempFlag)
                j.mtr_temp = j.data_obj->temp_;
            else
                j.drv_temp = j.data_obj->temp_;
        }
        // 拷贝 IMU 数据...
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type J60_6_hw::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // 极速拷贝：将 Controller 算出的指令填入发送缓冲区
        for (auto &j : all_joints_)
        {
            // 更新下一轮 IO 线程要发送的指令
            SetMotionCMD(j.cmd_obj, j.motor_id, CONTROL_MOTOR,
                         j.pos_des, j.vel_des, j.ff, j.kp, j.kd);
        }
        return hardware_interface::return_type::OK;
    }

    CallbackReturn J60_6_hw::on_deactivate(const rclcpp_lifecycle::State &)
    {
        for (int i = 0; i < 4; ++i)
        {
            bus.is_running = false;
            if (bus.thread_handle.joinable())
                bus.thread_handle.join();
            // 失能电机...
        }
        return CallbackReturn::SUCCESS;
    }

} // namespace dog_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dog_hardware::J60_6_hw, hardware_interface::SystemInterface)