
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <linux/can/raw.h>
#include <sys/time.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "dog_hardware/J60_6/can_protocol.h"

enum SendRecvRet
{
    //*******************************
    // SendRecvRet: SendRecv函数的返回值
    //*******************************
    //正常时返回0
    //发送长度错误返回-1
    //接收超时错误返回-2
    //接收epoll错误返回-3
    //接收长度错误返回-4
    // ID未匹配返回-5
    kNoSendRecvError = 0,
    kSendLengthError = -1,
    kRecvTimeoutError = -2,
    kRecvEpollError = -3,
    kRecvLengthError = -4,
    kRecvIdNotMatchError = -5
};

//检查SendRecv函数返回值
// CAN收发错误码 -> 字符串
const char *SendRecvErrorStr(int code)
{
    switch (code)
    {
    case kNoSendRecvError:
        return "正常";
    case kSendLengthError:
        return "发送长度错误";
    case kRecvTimeoutError:
        return "接收超时";
    case kRecvEpollError:
        return "接收Epoll错误";
    case kRecvLengthError:
        return "接收长度错误";
    case kRecvIdNotMatchError:
        return "接收ID未匹配";
    default:
        return "未知错误";
    }
}

enum MotorErrorType
{
    //*******************************
    // MotorErrorType: SendRecv函数的返回值
    //*******************************
    //全为0: 无错误
    // bit 0: 过压标志位
    // bit 1: 欠压标志位
    // bit 2: 过流标志位
    // bit 3: 关节过温标志位
    // bit 4: 驱动板过温标志位
    // bit 5: Can超时标志位
    // bit 6: 通讯失败标志位

    kMotorNoError = 0,
    kOverVoltage = (0x01 << 0),
    kUnderVoltage = (0x01 << 1),
    kOverCurrent = (0x01 << 2),
    kMotorOverTemp = (0x01 << 3),
    kDriverOverTemp = (0x01 << 4),
    kCanTimeout = (0x01 << 5),
    kCommFailure = (0x01 << 6)

};

//检查关节状态返回值
const char *CheckMotorErrorStr(uint16_t code)
{
    switch (code)
    {
    case kMotorNoError:
        return "无故障";
    case kOverVoltage:
        return "过压故障";
    case kUnderVoltage:
        return "欠压故障";
    case kOverCurrent:
        return "过流故障";
    case kMotorOverTemp:
        return "电机本体超温";
    case kDriverOverTemp:
        return "驱动器超温";
    case kCanTimeout:
        return "CAN通信超时";
    case kCommFailure:
        return "通信失败";
    default:
        return "未知故障";
    }
}

//存储电机返回的数据
typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
    float position_;
    float velocity_;
    float torque_;
    bool flag_;
    float temp_;
    uint16_t error_;
} MotorDATA;

//创建MotorDATA实例
MotorDATA *MotorDATACreate()
{
    MotorDATA *motor_data = (MotorDATA *)malloc(sizeof(MotorDATA));
    motor_data->error_ = kMotorNoError;
    return motor_data;
}

//销毁MotorDATA实例
void MotorDATADestroy(MotorDATA *motor_data)
{
    free(motor_data);
}

//存储发向电机的数据
typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
    float position_;
    float velocity_;
    float torque_;
    float kp_;
    float kd_;
} MotorCMD;

//创建MotorCMD实例
MotorCMD *MotorCMDCreate()
{
    MotorCMD *motor_cmd = (MotorCMD *)malloc(sizeof(MotorCMD));
    return motor_cmd;
}

//往MotorCMD写入普通命令
void SetNormalCMD(MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd)
{
    motor_cmd->motor_id_ = motor_id;
    motor_cmd->cmd_ = cmd;
}

//往MotorCMD写入控制命令
void SetMotionCMD(MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd, float position, float velocity, float torque, float kp, float kd)
{
    motor_cmd->motor_id_ = motor_id;
    motor_cmd->cmd_ = cmd;
    motor_cmd->position_ = position;
    motor_cmd->velocity_ = velocity;
    motor_cmd->torque_ = torque;
    motor_cmd->kp_ = kp;
    motor_cmd->kd_ = kd;
}

//销毁MotorCMD实例
void MotorCMDDestroy(MotorCMD *motor_cmd)
{
    free(motor_cmd);
}

//将MotorCMD中的float数据转换为CAN协议中发送的uint数据
void FloatsToUints(const MotorCMD *param, uint8_t *data)
{
    uint16_t _position = FloatToUint(param->position_, POSITION_MIN, POSITION_MAX, SEND_POSITION_LENGTH);
    uint16_t _velocity = FloatToUint(param->velocity_, VELOCITY_MIN, VELOCITY_MAX, SEND_VELOCITY_LENGTH);
    uint16_t _torque = FloatToUint(param->torque_, TORQUE_MIN, TORQUE_MAX, SEND_TORQUE_LENGTH);
    uint16_t _kp = FloatToUint(param->kp_, KP_MIN, KP_MAX, SEND_KP_LENGTH);
    uint16_t _kd = FloatToUint(param->kd_, KD_MIN, KD_MAX, SEND_KD_LENGTH);
    data[0] = _position;
    data[1] = _position >> 8;
    data[2] = _velocity;
    data[3] = ((_velocity >> 8) & 0x3f) | ((_kp & 0x03) << 6);
    data[4] = _kp >> 2;
    data[5] = _kd;
    data[6] = _torque;
    data[7] = _torque >> 8;
}

//将CAN协议中收到的uint数据转换为MotorDATA中的float数据
void UintsToFloats(const struct can_frame *frame, MotorDATA *data)
{
    const ReceivedMotionData *pcan_data = (const ReceivedMotionData *)frame->data;
    data->position_ = UintToFloat(pcan_data->position, POSITION_MIN, POSITION_MAX, RECEIVE_POSITION_LENGTH);
    data->velocity_ = UintToFloat(pcan_data->velocity, VELOCITY_MIN, VELOCITY_MAX, RECEIVE_VELOCITY_LENGTH);
    data->torque_ = UintToFloat(pcan_data->torque, TORQUE_MIN, TORQUE_MAX, RECEIVE_TORQUE_LENGTH);
    data->flag_ = (bool)pcan_data->temp_flag;
    if (data->flag_ == kMotorTempFlag)
    {
        data->temp_ = UintToFloat(pcan_data->temperature, MOTOR_TEMP_MIN, MOTOR_TEMP_MAX, RECEIVE_TEMP_LENGTH);
    }
    else
    {
        data->temp_ = UintToFloat(pcan_data->temperature, DRIVER_TEMP_MIN, DRIVER_TEMP_MAX, RECEIVE_TEMP_LENGTH);
    }
}

//结合motor_id和cmd形成CAN协议中的id
uint16_t FormCanId(uint8_t cmd, uint8_t motor_id)
{
    return (cmd << CAN_ID_SHIFT_BITS) | motor_id;
}

//根据MotorCMD进行所发送can帧的填充
void MakeSendFrame(const MotorCMD *cmd, struct can_frame *frame_ret)
{
    frame_ret->can_id = FormCanId(cmd->cmd_, cmd->motor_id_);
    switch (cmd->cmd_)
    {
    case ENABLE_MOTOR:
        frame_ret->can_dlc = SEND_DLC_ENABLE_MOTOR;
        break;

    case DISABLE_MOTOR:
        frame_ret->can_dlc = SEND_DLC_DISABLE_MOTOR;
        break;

    case SET_HOME:
        frame_ret->can_dlc = SEND_DLC_SET_HOME;
        break;

    case ERROR_RESET:
        frame_ret->can_dlc = SEND_DLC_ERROR_RESET;
        break;

    case CONTROL_MOTOR:
        frame_ret->can_dlc = SEND_DLC_CONTROL_MOTOR;
        FloatsToUints(cmd, frame_ret->data);
        break;

    case GET_STATUS_WORD:
        frame_ret->can_dlc = SEND_DLC_GET_STATUS_WORD;
        break;

    default:
        break;
    }
}

// 解析接收的CAN帧数据
void ParseRecvFrame(const struct can_frame *frame_ret, MotorDATA *data)
{
    uint32_t frame_id = frame_ret->can_id;
    uint32_t cmd = (frame_id >> CAN_ID_SHIFT_BITS) & 0x3f;
    uint32_t motor_id = frame_id & 0x0f;

    data->motor_id_ = motor_id;
    data->cmd_ = cmd;

    switch (cmd)
    {
    case ENABLE_MOTOR:
        // printf("[信息] 电机ID：%d 使能成功\r\n", (uint32_t)motor_id);
        break;

    case DISABLE_MOTOR:
        printf("[信息] 电机ID：%d 失能成功\r\n", (uint32_t)motor_id);
        break;

    case SET_HOME:
        // printf("[信息] 电机ID：%d 设置零点成功\r\n", (uint32_t)motor_id);
        break;

    case ERROR_RESET:
        printf("[信息] 电机ID：%d 清除故障成功\r\n", (uint32_t)motor_id);
        break;

    case CONTROL_MOTOR:
        UintsToFloats(frame_ret, data);
        break;

    case GET_STATUS_WORD:
        data->error_ = (frame_ret->data[0] << 8) | frame_ret->data[1];
        break;

    default:
        printf("[警告] 接收到未定义的命令帧\r\n");
        break;
    }
}

// DrMotorCan类，用于保存can的相关配置和资源
typedef struct
{
    bool is_show_log_;
    int can_socket_;
    int epoll_fd_;
    pthread_mutex_t rw_mutex;
} DrMotorCan;

// 创建并初始化电机CAN总线设备
DrMotorCan *DrMotorCanCreate(const char *can_name, bool is_show_log)
{
    int flags;
    struct ifreq ifr;
    struct sockaddr_can addr;
    struct epoll_event event;

    // 1. 分配内存并初始化结构体
    DrMotorCan *can = (DrMotorCan *)malloc(sizeof(DrMotorCan));
    if (can == NULL)
        return NULL;
    can->can_socket_ = -1;
    can->epoll_fd_ = -1;
    can->is_show_log_ = is_show_log;

    // 2. 初始化锁
    if (pthread_mutex_init(&can->rw_mutex, NULL) != 0)
    {
        free(can);
        return NULL;
    }

    // 3. 创建 Socket
    if ((can->can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
    {
        printf("[错误] CAN套接字创建失败\n");
        goto failed;
    }

    // 4. 设置非阻塞
    flags = fcntl(can->can_socket_, F_GETFL, 0);
    if (flags == -1 || fcntl(can->can_socket_, F_SETFL, flags | O_NONBLOCK) == -1)
    {
        printf("[错误] 设置非阻塞模式失败\n");
        goto failed;
    }

    // 5. 绑定设备
    strcpy(ifr.ifr_name, can_name);
    if (ioctl(can->can_socket_, SIOCGIFINDEX, &ifr) < 0)
    {
        printf("[错误] 获取网卡索引失败\n");
        goto failed;
    }

    addr.can_ifindex = ifr.ifr_ifindex;
    addr.can_family = AF_CAN;
    if (bind(can->can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
    {
        printf("[错误] 绑定CAN设备失败\n");
        goto failed;
    }

    // 6. 创建 epoll
    can->epoll_fd_ = epoll_create1(0);
    if (can->epoll_fd_ == -1)
    {
        printf("[错误] 创建epoll实例失败\n");
        goto failed;
    }

    // 7. 加入监听
    event.events = EPOLLIN;
    event.data.fd = can->can_socket_;
    if (epoll_ctl(can->epoll_fd_, EPOLL_CTL_ADD, can->can_socket_, &event) == -1)
    {
        printf("[错误] 添加epoll监听失败\n");
        goto failed;
    }

    return can;

failed:
    if (can->can_socket_ >= 0)
        close(can->can_socket_);
    if (can->epoll_fd_ >= 0)
        close(can->epoll_fd_);
    pthread_mutex_destroy(&can->rw_mutex);
    free(can);
    return NULL;
}

//销毁DrMotorCan实例
void DrMotorCanDestroy(DrMotorCan *can)
{
    if (can->epoll_fd_ >= 0)
    {
        close(can->epoll_fd_);
        can->epoll_fd_ = -1;
    }
    if (can->can_socket_ >= 0)
    {
        close(can->can_socket_);
        can->can_socket_ = -1;
    }

    pthread_mutex_destroy(&can->rw_mutex);
    free(can);
}

//使用DrMotorCan进行数据的发送和接收
int SendRecv(DrMotorCan *can, const MotorCMD *cmd, MotorDATA *data, int timeout_ms = 3)
{
    struct can_frame send_frame, recv_frame;
    MakeSendFrame(cmd, &send_frame);

    // 1. 清除旧缓存：防止断线重连后旧数据堆积导致的 ID 错位
    struct can_frame junk;
    while (read(can->can_socket_, &junk, sizeof(junk)) > 0)
        ;

    // 2. 发送指令
    pthread_mutex_lock(&can->rw_mutex);
    ssize_t nbytes1 = write(can->can_socket_, &send_frame, sizeof(send_frame));
    pthread_mutex_unlock(&can->rw_mutex);
    if (nbytes1 != sizeof(send_frame))
        return kSendLengthError;

    // 3. 等待并执行精准 ID 匹配
    struct epoll_event events;
    if (epoll_wait(can->epoll_fd_, &events, 1, timeout_ms) <= 0)
        return kRecvTimeoutError;

    pthread_mutex_lock(&can->rw_mutex);
    ssize_t nbytes2 = read(can->can_socket_, &recv_frame, sizeof(recv_frame));
    pthread_mutex_unlock(&can->rw_mutex);

    if (nbytes2 != sizeof(recv_frame))
        return kRecvLengthError;

    if ((recv_frame.can_id & 0x0F) != cmd->motor_id_)
        return kRecvIdNotMatchError;

    ParseRecvFrame(&recv_frame, data);
    return kNoSendRecvError;
}
