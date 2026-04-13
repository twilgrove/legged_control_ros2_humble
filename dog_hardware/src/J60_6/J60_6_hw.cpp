#include "dog_hardware/J60_6/J60_6_hw.hpp"

#include "dog_hardware/IWT603/wit_c_sdk.h"
#include "dog_hardware/IWT603/REG.h"
#include <time.h>
#include <sys/time.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <sys/stat.h>
#include <errno.h>
#include <termios.h>

// ===================== 数据更新标志位定义 =====================
#define ACC_UPDATE 0x01
#define GYRO_UPDATE 0x02
#define ANGLE_UPDATE 0x04
#define QUAT_UPDATE 0x08
#define READ_UPDATE 0x80

// ===================== 静态辅助变量与函数 =====================
const double PI = 3.1415926536;
static volatile char s_cDataUpdate = 0;
const double DEG_TO_RAD = M_PI / 180.0;

static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
    for (uint32_t i = 0; i < uiRegNum; i++)
    {
        switch (uiReg)
        {
        case AZ:
            s_cDataUpdate |= ACC_UPDATE;
            break;
        case GZ:
            s_cDataUpdate |= GYRO_UPDATE;
            break;
        case Yaw:
            s_cDataUpdate |= ANGLE_UPDATE;
            break;
        case q3:
            s_cDataUpdate |= QUAT_UPDATE;
            break;
        default:
            s_cDataUpdate |= READ_UPDATE;
            break;
        }
        uiReg++;
    }
}

static int set_nonblock(int fd)
{
    int flags = fcntl(fd, F_GETFL, 0);
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static int is_device_exist(const char *dev)
{
    struct stat st;
    return stat(dev, &st) == 0;
}

static bool is_can_interface_exist(const char *can_name)
{
    char path[128];
    snprintf(path, sizeof(path), "/sys/class/net/%s", can_name);

    struct stat st;
    return stat(path, &st) == 0; // 存在返回 1，不存在返回 0
}

bool is_can_interface_up(const std::string &interface)
{
    struct ifreq ifr;
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0)
        return false;

    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);

    if (ioctl(sock, SIOCGIFFLAGS, &ifr) < 0)
    {
        close(sock);
        return false; // 网卡可能不存在
    }
    close(sock);
    return (ifr.ifr_flags & IFF_UP) && (ifr.ifr_flags & IFF_RUNNING);
}
bool reset_can_interface(const std::string &interface, int bitrate = 1000000)
{
    // 1. 构造 Shell 指令
    std::string cmd_down = "sudo ip link set " + interface + " down";
    std::string cmd_up = "sudo ip link set " + interface + " up type can bitrate " + std::to_string(bitrate);
    std::string cmd_tx = "sudo ip link set " + interface + " txqueuelen 1000";

    // 2. 执行 Down
    if (std::system(cmd_down.c_str()) != 0)
    {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 3. 执行 Up
    if (std::system(cmd_up.c_str()) != 0)
    {
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 4. 设置 txqueuelen
    std::system(cmd_tx.c_str());

    return true;
}
// 定义一个全局或静态的清理标记
static std::atomic<bool> g_force_exit{false};

namespace dog_hardware
{

    CallbackReturn J60_6_hw::on_init(const hardware_interface::HardwareInfo &info)
    {
        if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
            return CallbackReturn::FAILURE;

        RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"), "\033[1;36m====================================================\033[0m");
        RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"), "\033[1;36m[ 初始化开始 ] 🚀 J60_6_hw\033[0m");
        // 1. 初始化 4 条总线配置
        can_workers_[0].interface = "can_lf";
        can_workers_[1].interface = "can_lh";
        can_workers_[2].interface = "can_rf";
        can_workers_[3].interface = "can_rh";

        // 2. 解析 URDF 中的关节并分配到对应的 CAN 总线
        all_joints_.reserve(12);
        for (const auto &joint : info.joints)
        {
            auto jd = std::make_unique<JointData>();
            jd->name = joint.name;

            // --- 确定 Motor ID ---
            if (joint.name.find("HAA") != std::string::npos)
                jd->motor_id = 0;
            else if (joint.name.find("HFE") != std::string::npos)
                jd->motor_id = 1;
            else if (joint.name.find("KFE") != std::string::npos)
                jd->motor_id = 2;
            else
                continue;

            // --- 确定 Bus ID ---
            int bus_id = -1;
            if (joint.name.find("LF") != std::string::npos)
                bus_id = 0;
            else if (joint.name.find("LH") != std::string::npos)
                bus_id = 1;
            else if (joint.name.find("RF") != std::string::npos)
                bus_id = 2;
            else if (joint.name.find("RH") != std::string::npos)
                bus_id = 3;

            // --- 设置 joint_sign (方向修正) ---
            jd->joint_sign = 1.0; // 默认为正

            // A. 右侧(RF, RH)的 HFE(1) 和 KFE(2) 取反 (镜像安装)
            if ((bus_id == 2 || bus_id == 3) && (jd->motor_id == 1 || jd->motor_id == 2))
            {
                jd->joint_sign = -1.0;
            }

            // B. 后侧(LH, RH)的 HAA(0) 取反 (前后对称安装)
            if ((bus_id == 1 || bus_id == 3) && (jd->motor_id == 0))
            {
                jd->joint_sign = -1.0;
            }

            // --- 初始化对象 ---
            jd->cmd_obj = MotorCMDCreate();
            jd->data_obj = MotorDATACreate();
            jd->temp_data_obj = MotorDATACreate();

            all_joints_.push_back(std::move(jd));
            if (bus_id >= 0 && bus_id < 4)
                can_workers_[bus_id].joints.push_back(all_joints_.back().get());
        }
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn J60_6_hw::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"), "\033[1;36m[ 使能开始 ] 🚀 J60_6_hw\033[0m");

        for (int i = 0; i < 4; ++i)
        {
            if (!is_can_interface_exist(can_workers_[i].interface.c_str()))
            {
                RCLCPP_ERROR(rclcpp::get_logger("J60_6_hw"), "❌ 未找到 CAN 接口: %s", can_workers_[i].interface.c_str());
                continue;
            }
            else
            {
                if (!reset_can_interface(can_workers_[i].interface))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("J60_6_hw"), "❌ 激活 CAN 接口 %s 失败", can_workers_[i].interface.c_str());
                    continue;
                }
                else
                {
                    RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"), "✅ 激活 CAN 接口 %s 成功", can_workers_[i].interface.c_str());
                    can_workers_[i].can_handle = DrMotorCanCreate(can_workers_[i].interface.c_str(), false);
                }
            }

            if (can_workers_[i].can_handle)
            {
                RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"), "成功打开设备: %s", can_workers_[i].interface.c_str());

                for (auto *j : can_workers_[i].joints)
                {
                    int ret = 0;
                    bool joint_success = false;

                    // --- 1. 尝试设置零点 (SET_HOME) ---
                    for (int retry = 0; retry < 5; ++retry)
                    {
                        SetNormalCMD(j->cmd_obj, j->motor_id, SET_HOME);
                        ret = SendRecv(can_workers_[i].can_handle, j->cmd_obj, j->data_obj, 1000);
                        if (ret == kNoSendRecvError)
                        {
                            RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"), "✅ 电机 [%s] 设零成功 (尝试 %d 次)", j->name.c_str(), retry + 1);
                            joint_success = true;
                            break;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }

                    if (!joint_success)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("J60_6_hw"), "❌ 电机 [%s] 设零失败! %s", j->name.c_str(), SendRecvErrorStr(ret));
                        overall_success = false;
                        can_workers_[i].is_running = false;
                        continue;
                    }

                    // --- 2. 尝试使能电机 (ENABLE_MOTOR) ---
                    joint_success = false;
                    for (int retry = 0; retry < 5; ++retry)
                    {
                        SetNormalCMD(j->cmd_obj, j->motor_id, ENABLE_MOTOR);
                        ret = SendRecv(can_workers_[i].can_handle, j->cmd_obj, j->data_obj, 1000);
                        if (ret == kNoSendRecvError)
                        {
                            RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"), "✅ 电机 [%s] 使能成功 (尝试 %d 次)", j->name.c_str(), retry + 1);
                            joint_success = true;
                            break;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }

                    if (!joint_success)
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("J60_6_hw"), "❌ 电机 [%s] 使能失败! %s", j->name.c_str(), SendRecvErrorStr(ret));
                        can_workers_[i].is_running = false;
                        overall_success = false;
                    }
                }
            }
            else
            {
                RCLCPP_ERROR(rclcpp::get_logger("J60_6_hw"), "无法打开设备: %s", can_workers_[i].interface.c_str());
                can_workers_[i].is_running = false;
                overall_success = false;
                continue;
            }

            if (can_workers_[i].is_running)
                can_workers_[i].thread_handle = std::thread(&J60_6_hw::CanWorkerFunc, this, i);
        }

        if (is_device_exist("/dev/ttyimu"))
        {
            RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"), "✅成功找到 IMU 设备: /dev/ttyimu");
            imu_running_ = true;
            imu_thread_ = std::thread(&J60_6_hw::ImuWorkerFunc, this, "/dev/ttyimu");
        }
        else
        {
            RCLCPP_ERROR(rclcpp::get_logger("J60_6_hw"), "未找到 IMU 设备: /dev/ttyimu");
            overall_success = false;
        }

        //  if (!overall_success)
        //     return CallbackReturn::FAILURE;

        RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"), "\033[1;32m[ 初始化完成 ] ✅ J60_6_hw\033[0m");
        RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"), "\033[1;32m====================================================\033[0m");

        return CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> J60_6_hw::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> states;
        for (auto &j : all_joints_)
        {
            states.emplace_back(j->name, "position", &j->pos);
            states.emplace_back(j->name, "velocity", &j->vel);
            states.emplace_back(j->name, "effort", &j->eff);
            states.emplace_back(j->name, "driver_tem", &j->drv_temp);
            states.emplace_back(j->name, "motor_tem", &j->mtr_temp);
            states.emplace_back(j->name, "error_code", &j->err_code);
        }

        states.emplace_back("imu_sensor", "orientation.x", &Imudata.ori[0]);
        states.emplace_back("imu_sensor", "orientation.y", &Imudata.ori[1]);
        states.emplace_back("imu_sensor", "orientation.z", &Imudata.ori[2]);
        states.emplace_back("imu_sensor", "orientation.w", &Imudata.ori[3]);
        states.emplace_back("imu_sensor", "angular_velocity.x", &Imudata.ang_vel[0]);
        states.emplace_back("imu_sensor", "angular_velocity.y", &Imudata.ang_vel[1]);
        states.emplace_back("imu_sensor", "angular_velocity.z", &Imudata.ang_vel[2]);
        states.emplace_back("imu_sensor", "linear_acceleration.x", &Imudata.lin_acc[0]);
        states.emplace_back("imu_sensor", "linear_acceleration.y", &Imudata.lin_acc[1]);
        states.emplace_back("imu_sensor", "linear_acceleration.z", &Imudata.lin_acc[2]);

        states.emplace_back("LF_contact_sensor", "contact", &dummy_contact[0]);
        states.emplace_back("RF_contact_sensor", "contact", &dummy_contact[1]);
        states.emplace_back("LH_contact_sensor", "contact", &dummy_contact[2]);
        states.emplace_back("RH_contact_sensor", "contact", &dummy_contact[3]);

        return states;
    }

    std::vector<hardware_interface::CommandInterface> J60_6_hw::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> commands;
        for (auto &j : all_joints_)
        {
            commands.emplace_back(j->name, "position", &j->pos_des);
            commands.emplace_back(j->name, "velocity", &j->vel_des);
            commands.emplace_back(j->name, "kp", &j->kp);
            commands.emplace_back(j->name, "kd", &j->kd);
            commands.emplace_back(j->name, "effort", &j->ff);
        }
        return commands;
    }

    void J60_6_hw::CanWorkerFunc(int bus_idx)
    {
        auto &bus = can_workers_[bus_idx];
        uint32_t t1s = 0, error_cnt = 0;
        int rett = kNoSendRecvError;
        auto last_time = std::chrono::steady_clock::now();
        while (bus.is_running)
        {
            t1s++;
            for (auto *j : bus.joints)
            {
                MotorCMD temp_cmd;
                {
                    std::lock_guard<std::mutex> lock(j->cmd_mutex_);
                    temp_cmd = *(j->cmd_obj);
                }

                if (t1s % 1000 == 0)
                {
                    SetNormalCMD(&temp_cmd, j->motor_id, GET_STATUS_WORD);
                    rett = SendRecv(bus.can_handle, &temp_cmd, j->temp_data_obj);
                }
                else
                    rett = SendRecv(bus.can_handle, &temp_cmd, j->temp_data_obj);

                if (rett == kNoSendRecvError)
                {
                    std::lock_guard<std::mutex> lock(j->data_mutex_);
                    *(j->data_obj) = *(j->temp_data_obj);
                }
                else
                {
                    RCLCPP_ERROR(rclcpp::get_logger("J60_6_hw"),
                                 "电机 %s 通信失败", j->name.c_str());
                    error_cnt++;
                    std::lock_guard<std::mutex> lock(j->data_mutex_);
                    j->data_obj->error_ = 44;
                }
            }

            if (t1s % 100 == 0 && error_cnt >= 20)
            {
                error_cnt = 0;
                if (!is_can_interface_up(bus.interface))
                {
                    RCLCPP_ERROR(rclcpp::get_logger("J60_6_hw"),
                                 "🚨 CAN 网卡 %s 不在系统里！已掉线！",
                                 bus.interface.c_str());
                    DrMotorCanDestroy(bus.can_handle);
                    bus.can_handle = nullptr;
                    return;
                    /*USB转CAN模块会卡死，无法重启
                    // // 销毁旧句柄
                    // if (bus.can_handle)
                    // {
                    //     DrMotorCanDestroy(bus.can_handle);
                    //     bus.can_handle = nullptr;
                    // }

                    // // 等待网卡恢复
                    // while (!is_can_interface_exist(bus.interface.c_str()) && bus.is_running)
                    // {
                    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    //     RCLCPP_WARN(rclcpp::get_logger("J60_6_hw"),
                    //                 "等待 CAN 网卡 %s 恢复...", bus.interface.c_str());
                    // }

                    // // 重新创建 CAN
                    // if (is_can_interface_exist(bus.interface.c_str()) && reset_can_interface(bus.interface))
                    // {
                    //     bus.can_handle = DrMotorCanCreate(bus.interface.c_str(), false);
                    //     RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"),
                    //                 "✅ CAN 网卡 %s 恢复，重新创建成功", bus.interface.c_str());

                    //     // 重新使能电机
                    //     for (auto *j : bus.joints)
                    //     {
                    //         std::lock_guard<std::mutex> cmd_lock(j->cmd_mutex_);
                    //         SetNormalCMD(j->cmd_obj, j->motor_id, ENABLE_MOTOR);
                    //         rett = SendRecv(bus.can_handle, j->cmd_obj, j->data_obj);
                    //         if (rett == kNoSendRecvError)
                    //         {
                    //             RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"),
                    //                         "电机 %s 重新使能", j->name.c_str());
                    //         }
                    //         else
                    //         {
                    //             RCLCPP_ERROR(rclcpp::get_logger("J60_6_hw"),
                    //                          "电机 %s 重新使能失败!!! 错误: %s", j->name.c_str(), SendRecvErrorStr(rett));
                    //         }
                    //     }
                    // }*/
                }
            }

            auto now = std::chrono::steady_clock::now();
            auto next_time = last_time + std::chrono::milliseconds(1);

            if (now < next_time)
                std::this_thread::sleep_until(next_time);

            last_time = next_time;
        }
    }

    void J60_6_hw::ImuWorkerFunc(std::string port)
    {
        WitInit(WIT_PROTOCOL_NORMAL, 0x50);
        WitRegisterCallBack(SensorDataUpdata);

        int fd = -1;
        int epfd = epoll_create1(0);
        struct epoll_event ev, events[1];

        while (imu_running_)
        {
            // 1. 检查设备及断线重连
            if (!is_device_exist(port.c_str()))
            {
                if (fd >= 0)
                {
                    epoll_ctl(epfd, EPOLL_CTL_DEL, fd, nullptr);
                    close(fd);
                    fd = -1;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }

            if (fd < 0)
            {
                fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
                if (fd < 0)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    continue;
                }

                struct termios opt;
                tcgetattr(fd, &opt);
                cfsetispeed(&opt, B921600);
                cfsetospeed(&opt, B921600);
                opt.c_cflag &= ~CSIZE;
                opt.c_cflag |= CS8;
                opt.c_cflag &= ~PARENB;
                opt.c_cflag &= ~CSTOPB;
                opt.c_cflag |= CREAD | CLOCAL;
                opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
                opt.c_iflag &= ~(INPCK | ISTRIP | ICRNL | INLCR | IXON | IXOFF);
                opt.c_oflag &= ~OPOST;
                tcsetattr(fd, TCSANOW, &opt);
                set_nonblock(fd);

                ev.events = EPOLLIN | EPOLLET;
                ev.data.fd = fd;
                epoll_ctl(epfd, EPOLL_CTL_ADD, fd, &ev);
                RCLCPP_INFO(rclcpp::get_logger("J60_6_hw"), "成功打开 IMU 设备: %s", port.c_str());
            }

            // 2. 等待数据读取
            int n = epoll_wait(epfd, events, 1, 100);
            if (n > 0)
            {
                char buf[128];
                ssize_t len;
                while ((len = ::read(fd, buf, sizeof(buf))) > 0)
                {
                    for (ssize_t i = 0; i < len; i++)
                        WitSerialDataIn(buf[i]);
                }
                if (len < 0 && (errno == EIO || errno == EBADF))
                {
                    epoll_ctl(epfd, EPOLL_CTL_DEL, fd, nullptr);
                    close(fd);
                    fd = -1;
                    continue;
                }
            }

            // 3. 更新类成员 ImuData
            if (s_cDataUpdate)
            {
                std::lock_guard<std::mutex> imu_lock(imu_mutex_);
                // 加速度 (m/s^2 )
                temp_ImuData.lin_acc[0] = sReg[AX] / 32768.0f * 16.0f * 9.81f;
                temp_ImuData.lin_acc[1] = sReg[AY] / 32768.0f * 16.0f * 9.81f;
                temp_ImuData.lin_acc[2] = sReg[AZ] / 32768.0f * 16.0f * 9.81f;

                // 角速度 (rad/s)
                temp_ImuData.ang_vel[0] = sReg[GX] / 32768.0f * 2000.0f * DEG_TO_RAD;
                temp_ImuData.ang_vel[1] = sReg[GY] / 32768.0f * 2000.0f * DEG_TO_RAD;
                temp_ImuData.ang_vel[2] = sReg[GZ] / 32768.0f * 2000.0f * DEG_TO_RAD;

                // 四元数 (w, x, y, z)
                temp_ImuData.ori[0] = sReg[q1] / 32768.0f; // x
                temp_ImuData.ori[1] = sReg[q2] / 32768.0f; // y
                temp_ImuData.ori[2] = sReg[q3] / 32768.0f; // z
                temp_ImuData.ori[3] = sReg[q0] / 32768.0f; // w

                s_cDataUpdate = 0;
            }
        }

        if (fd >= 0)
        {
            epoll_ctl(epfd, EPOLL_CTL_DEL, fd, nullptr);
            close(fd);
        }
        close(epfd);
    }

    hardware_interface::return_type J60_6_hw::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        {
            std::lock_guard<std::mutex> imu_lock(imu_mutex_);
            Imudata = temp_ImuData;
        }
        for (auto &j : all_joints_)
        {

            j->pos = j->data_obj->position_ * j->joint_sign;
            j->vel = j->data_obj->velocity_ * j->joint_sign;
            j->eff = j->data_obj->torque_ * j->joint_sign;

            j->err_code = (double)j->data_obj->error_;
            if (j->data_obj->flag_ == kMotorTempFlag)
                j->mtr_temp = j->data_obj->temp_;
            else if (j->data_obj->flag_ == kDriverTempFlag)
                j->drv_temp = j->data_obj->temp_;
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type J60_6_hw::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        for (auto &j : all_joints_)
        {
            std::lock_guard<std::mutex> cmd_lock(j->cmd_mutex_);
            SetMotionCMD(j->cmd_obj,
                         j->motor_id,
                         CONTROL_MOTOR,
                         j->pos_des * j->joint_sign,
                         j->vel_des * j->joint_sign,
                         j->ff * j->joint_sign,
                         j->kp,
                         j->kd);
        }

        return hardware_interface::return_type::OK;
    }

    CallbackReturn J60_6_hw::on_deactivate(const rclcpp_lifecycle::State &)
    {
        for (int i = 0; i < 4; ++i)
        {
            can_workers_[i].is_running = false;
            if (can_workers_[i].thread_handle.joinable())
                can_workers_[i].thread_handle.join();
        }
        imu_running_ = false;
        if (imu_thread_.joinable())
            imu_thread_.join();

        for (auto &j : all_joints_)
        {
            if (j->cmd_obj)
                MotorCMDDestroy(j->cmd_obj);
            if (j->data_obj)
                MotorDATADestroy(j->data_obj);
            if (j->temp_data_obj)
                MotorDATADestroy(j->temp_data_obj);
        }

        for (auto &bus : can_workers_)
        {
            if (bus.can_handle)
            {
                DrMotorCanDestroy(bus.can_handle);
                bus.can_handle = nullptr;
            }
        }
        return CallbackReturn::SUCCESS;
    }

} // namespace dog_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dog_hardware::J60_6_hw, hardware_interface::SystemInterface)