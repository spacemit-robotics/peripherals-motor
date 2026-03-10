#ifndef DAMIAO_H
#define DAMIAO_H

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "socketcan.h"

namespace damiao {
struct CanFrameStamp {
    can_frame frame;
    // ros::Time stamp;
};

#pragma pack(1)

/*!
 * @brief Motor Type 电机类型
 */
enum DM_Motor_Type {
    DM4310,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
    Num_Of_Motor
};

/**
 * @brief 电机控制模式枚举
 *
 * 支持 4 种控制模式，值为 CAN ID 的偏移量
 */
enum Control_Mode {
    MIT_MODE = 0x000,        ///< MIT 阻抗控制：支持位置、速度、力矩同时控制
    POS_VEL_MODE = 0x100,    ///< 位置+速度控制：同时指定目标位置和速度
    VEL_MODE = 0x200,        ///< 纯速度控制：只控制目标速度
    POS_FORCE_MODE = 0x300,  ///< 位置+力矩控制：位置控制+力矩限制
};

/**
 * @brief 控制模式代码（用于 switchControlMode）
 */
enum Control_Mode_Code {
    MIT = 1,        ///< MIT 模式
    POS_VEL = 2,    ///< 位置速度模式
    VEL = 3,        ///< 速度模式
    POS_FORCE = 4,  ///< 位置力矩模式
};

/*
 * @brief 寄存器列表 具体参考达妙手册
 */
enum DM_REG {
    UV_Value = 0,
    KT_Value = 1,
    OT_Value = 2,
    OC_Value = 3,
    ACC = 4,
    DEC = 5,
    MAX_SPD = 6,
    MST_ID = 7,
    ESC_ID = 8,
    TIMEOUT = 9,
    CTRL_MODE = 10,
    Damp = 11,
    Inertia = 12,
    hw_ver = 13,
    sw_ver = 14,
    SN = 15,
    NPP = 16,
    Rs = 17,
    LS = 18,
    Flux = 19,
    Gr = 20,
    PMAX = 21,
    VMAX = 22,
    TMAX = 23,
    I_BW = 24,
    KP_ASR = 25,
    KI_ASR = 26,
    KP_APR = 27,
    KI_APR = 28,
    OV_Value = 29,
    GREF = 30,
    Deta = 31,
    V_BW = 32,
    IQ_c1 = 33,
    VL_c1 = 34,
    can_br = 35,
    sub_ver = 36,
    u_off = 50,
    v_off = 51,
    k1 = 52,
    k2 = 53,
    m_off = 54,
    dir = 55,
    p_m = 80,
    xout = 81,
};

#pragma pack()

typedef struct {
    float Q_MAX;
    float DQ_MAX;
    float TAU_MAX;
} Limit_param;

// 电机PMAX DQMAX TAUMAX参数
extern Limit_param limit_param[Num_Of_Motor];

/**
 * @struct MotionParams
 * @brief 运动参数结构体 - 用于配置电机运动轨迹
 */
struct MotionParams {
    double total_revolutions;  ///< 总圈数
    double motion_time;        ///< 运动时间 (秒)
    double accel_time;         ///< 加速时间 (秒)
    float kp;                  ///< MIT模式位置刚度增益 (0-500)
    float kd;                  ///< MIT模式阻尼增益 (0-5)

    // 默认构造函数
    MotionParams() : total_revolutions(2.0), motion_time(5.0), accel_time(1.0), kp(50.0), kd(2.0) {
    }

    // 带参数构造函数
    MotionParams(double revs, double time, double accel, float kp_val, float kd_val)
        : total_revolutions(revs), motion_time(time), accel_time(accel), kp(kp_val), kd(kd_val) {
    }
};

/**
 * @struct DmActData
 * @brief 电机数据结构 - 存储电机的状态和命令
 */
struct DmActData {
    DM_Motor_Type motorType;              ///< 电机型号（DM4310、DM4340 等）
    Control_Mode mode;                    ///< 当前控制模式
    uint16_t can_id;                      ///< 电机 CAN ID
    uint16_t mst_id;                      ///< 主机 ID
    double pos, vel, effort;              ///< 当前状态：位置(rad)、速度(rad/s)、力矩(Nm)
    double cmd_pos, cmd_vel, cmd_effort;  ///< 命令值
    double kp, kd;                        ///< MIT 模式增益：位置刚度、阻尼
};

/**
 * @class Motor
 * @brief 单个电机对象 - 管理单个达妙电机的状态和参数
 *
 * 该类负责：
 * - 存储电机的状态信息（位置、速度、力矩）
 * - 管理电机的参数（寄存器值）
 * - 提供状态查询和参数设置接口
 */
class Motor {
private:
    uint16_t Can_id;            ///< 电机 CAN ID
    uint16_t Master_id;         ///< 主机 ID
    float state_q = 0.0;        ///< 当前位置 (rad)
    float state_dq = 0.0;       ///< 当前速度 (rad/s)
    float state_tau = 0.0;      ///< 当前力矩 (Nm)
    Limit_param limit_param{};  ///< 电机限制参数
    DM_Motor_Type Motor_Type;   ///< 电机型号
    Control_Mode mode;          ///< 当前控制模式

    union ValueUnion {
        float floatValue;
        uint32_t uint32Value;
    };

    struct ValueType {
        ValueUnion value;
        bool isFloat;
    };

    std::unordered_map<uint32_t, ValueType> param_map;  ///< 参数存储表

public:
    std::chrono::system_clock::time_point stamp;  ///< 时间戳
    double frequency;                             ///< 更新频率

    /**
     * @brief 构造函数
     * @param motor_type 电机型号
     * @param ctrl_mode 控制模式
     * @param can_id 电机 CAN ID
     * @param master_id 主机 ID
     */
    Motor(DM_Motor_Type motor_type, Control_Mode ctrl_mode, uint16_t can_id, uint16_t master_id);

    /**
     * @brief 接收电机反馈数据
     * @param q 位置 (rad)
     * @param dq 速度 (rad/s)
     * @param tau 力矩 (Nm)
     */
    void receive_data(float q, float dq, float tau);

    /// @brief 获取电机型号
    DM_Motor_Type GetMotorType() const {
        return this->Motor_Type;
    }

    /// @brief 获取当前控制模式
    Control_Mode GetMotorMode() const {
        return this->mode;
    }

    /// @brief 获取电机限制参数（最大位置、速度、力矩）
    Limit_param get_limit_param() {
        return limit_param;
    }

    /// @brief 获取主机 ID
    uint16_t GetMasterId() const {
        return this->Master_id;
    }

    /// @brief 获取电机 CAN ID
    uint16_t GetCanId() const {
        return this->Can_id;
    }

    /// @brief 获取当前位置 (rad)
    float Get_Position() const {
        return this->state_q;
    }

    /// @brief 获取当前速度 (rad/s)
    float Get_Velocity() const {
        return this->state_dq;
    }

    /// @brief 获取当前力矩 (Nm)
    float Get_tau() const {
        return this->state_tau;
    }

    /// @brief 设置控制模式
    void set_mode(Control_Mode value) {
        this->mode = value;
    }

    /// @brief 设置浮点参数
    void set_param(int key, float value);

    /// @brief 设置整型参数
    void set_param(int key, uint32_t value);

    /// @brief 获取浮点参数
    float get_param_as_float(int key) const;

    /// @brief 获取整型参数
    uint32_t get_param_as_uint32(int key) const;

    /// @brief 检查参数是否存在
    bool is_have_param(int key) const;
};

/**
 * @class Motor_Control
 * @brief 电机控制核心类 - 管理单条 CAN 总线上的所有电机
 *
 * 该类负责：
 * - 打开和管理 CAN 接口
 * - 创建和管理该总线上的所有 Motor 对象
 * - 实现各种控制模式（MIT、位置速度、纯速度等）
 * - 处理 CAN 帧的收发和解析
 * - 参数读写和模式切换
 *
 * 使用流程：
 * 1. 构造函数打开 CAN 接口
 * 2. 在控制循环中调用 read() 读取反馈
 * 3. 调用控制函数（control_mit/control_pos_vel/control_vel）
 * 4. 调用 write() 发送命令
 * 5. 析构函数关闭 CAN 接口
 */
class Motor_Control {
public:
    /**
     * @brief 构造函数 - 打开 CAN 接口
     * @param bus_name CAN 总线名称（如 "can0"）
     * @param data_ptr 指向电机数据结构的指针
     *
     * 该构造函数会：
     * - 打开指定的 CAN 接口
     * - 启动 CAN 接收线程
     * - 创建 Motor 对象并添加到管理列表
     */
    Motor_Control(std::string bus_name, std::unordered_map<uint16_t, DmActData>* data_ptr);

    /// @brief 析构函数 - 关闭 CAN 接口
    ~Motor_Control();

    /**
     * @brief 设置运动参数
     * @param params 运动参数结构体
     *
     * 用于从外部设置电机运动轨迹参数
     */
    void setMotionParams(const MotionParams& params) {
        motion_params_ = params;
    }

    /**
     * @brief 获取当前运动参数
     * @return 运动参数结构体
     */
    MotionParams getMotionParams() const {
        return motion_params_;
    }

    /**
     * @brief 从 CAN 总线读取所有电机的反馈数据
     *
     * 该函数会：
     * - 从接收缓冲区读取最新的 CAN 帧
     * - 解析电机的位置、速度、力矩
     * - 更新 Motor 对象的状态
     */
    void read();

    /**
     * @brief 向 CAN 总线发送所有电机的控制命令
     *
     * 该函数会：
     * - 遍历所有 Motor 对象
     * - 根据控制模式打包 CAN 帧
     * - 通过 SocketCAN 发送到电机
     */
    void write();

    /**
     * @brief CAN 帧接收回调函数
     * @param frame 接收到的 CAN 帧
     *
     * 该函数在 CAN 接收线程中被调用
     * 用于处理来自电机的反馈数据
     */
    void canframeCallback(const can_frame& frame);

    /**
     * @brief 刷新电机状态
     * @param motor 目标电机对象
     *
     * 从 DmActData 中读取最新的状态数据
     * 并更新到 Motor 对象
     */
    void refresh_motor_status(Motor& motor);

    /// @brief 失能所有电机（紧急停止）
    void disable_all();

    /**
     * @brief 设置当前位置为零点
     * @param DM_Motor 目标电机
     *
     * 用于电机零位标定
     */
    void set_zero_position(Motor& DM_Motor);

    /// @brief 使能所有电机
    void enable_all();

    /**
     * @brief MIT 阻抗控制
     * @param DM_Motor 目标电机
     * @param kp 位置刚度增益 (0-500)
     * @param kd 阻尼增益 (0-5)
     * @param q 目标位置 (rad)
     * @param dq 目标速度 (rad/s)
     * @param tau 前馈力矩 (Nm)
     *
     * 控制方程：τ = kp*(q_des-q) + kd*(dq_des-dq) + τ_ff
     * 最灵活的控制模式，适合力控交互任务
     */
    void control_mit(Motor& DM_Motor, float kp, float kd, float q, float dq, float tau);

    /**
     * @brief 位置+速度控制
     * @param DM_Motor 目标电机
     * @param pos 目标位置 (rad)
     * @param vel 目标速度 (rad/s)
     *
     * 同时指定位置和速度，电机内部 PID 跟踪
     * 适合轨迹跟踪任务
     */
    void control_pos_vel(Motor& DM_Motor, float pos, float vel);

    /**
     * @brief 纯速度控制
     * @param DM_Motor 目标电机
     * @param vel 目标速度 (rad/s)
     *
     * 只控制速度，最简单的控制模式
     * 适合恒速运动
     */
    void control_vel(Motor& DM_Motor, float vel);

    /**
     * @brief 接收电机参数反馈
     * @param data 参数数据指针
     *
     * 处理电机返回的参数读取结果
     */
    void receive_param(uint8_t* data);

    /**
     * @brief 添加电机到管理列表
     * @param DM_Motor 电机对象指针
     *
     * 用于动态添加电机
     */
    void addMotor(std::shared_ptr<Motor> DM_Motor);

    /**
     * @brief 读取电机内部寄存器参数
     * @param DM_Motor 目标电机
     * @param RID 寄存器 ID（参考 DM_REG 枚举）
     * @return 参数值
     *
     * 常用寄存器：
     * - MAX_SPD (6): 最大速度
     * - CTRL_MODE (10): 控制模式
     * - PMAX (21): 最大位置
     * - VMAX (22): 最大速度
     * - TMAX (23): 最大力矩
     */
    float read_motor_param(Motor& DM_Motor, uint8_t RID);

    /**
     * @brief 切换电机控制模式
     * @param DM_Motor 目标电机
     * @param mode 目标模式（MIT、POS_VEL、VEL、POS_FORCE）
     * @return true 切换成功，false 切换失败
     *
     * 支持的模式：
     * - MIT (1): MIT 阻抗控制
     * - POS_VEL (2): 位置+速度控制
     * - VEL (3): 纯速度控制
     * - POS_FORCE (4): 位置+力矩控制
     */
    bool switchControlMode(Motor& DM_Motor, Control_Mode_Code mode);

    /**
     * @brief 修改电机内部寄存器参数
     * @param DM_Motor 目标电机
     * @param RID 寄存器 ID
     * @param data 参数值
     * @return true 修改成功，false 修改失败
     *
     * @note 修改后需调用 save_motor_param() 才能永久保存
     */
    bool change_motor_param(Motor& DM_Motor, uint8_t RID, float data);

    /**
     * @brief 保存电机参数到 Flash
     * @param DM_Motor 目标电机
     *
     * 将所有修改的参数永久保存到电机内部 Flash
     * 电机默认参数不会自动写入 Flash，需要显式调用此函数
     *
     * @warning 频繁写 Flash 会缩短电机寿命，建议在调试阶段使用
     */
    void save_motor_param(Motor& DM_Motor);

    /**
     * @brief 修改电机软件限制参数
     * @param DM_Motor 目标电机
     * @param P_MAX 最大位置限制 (rad)
     * @param Q_MAX 最大速度限制 (rad/s)
     * @param T_MAX 最大力矩限制 (Nm)
     *
     * @note 这是软件限制，不是电机内部寄存器参数
     * 用于安全保护，防止电机超出安全范围
     */
    static void changeMotorLimit(Motor& DM_Motor, float P_MAX, float Q_MAX, float T_MAX);

    /**
     * @brief 获取电机管理映射表
     * @return 电机映射表的常量引用
     */
    const std::unordered_map<uint16_t, std::shared_ptr<Motor>>& get_motors() const {
        return motors;
    }

    /**
     * @brief 发送控制命令
     * @param id 电机 ID
     * @param cmd 命令字节
     */
    void send_control_cmd(uint16_t id, uint8_t cmd) {
        control_cmd(id, cmd);
    }

private:
    void control_cmd(uint16_t id, uint8_t cmd);

    void write_motor_param(Motor& DM_Motor, uint8_t RID, const uint8_t data[4]);
    static bool is_in_ranges(int number) {
        return (7 <= number && number <= 10) || (13 <= number && number <= 16) || (35 <= number && number <= 36);
    }

    static uint32_t float_to_uint32(float value) {
        return static_cast<uint32_t>(value);
    }

    static float uint32_to_float(uint32_t value) {
        return static_cast<float>(value);
    }
    static float uint8_to_float(const uint8_t data[4]) {
        uint32_t combined = (static_cast<uint32_t>(data[3]) << 24) | (static_cast<uint32_t>(data[2]) << 16) |
                            (static_cast<uint32_t>(data[1]) << 8) | static_cast<uint32_t>(data[0]);
        float result;
        memcpy(&result, &combined, sizeof(result));
        return result;
    }
    std::unordered_map<uint16_t, std::shared_ptr<Motor>> motors;

    std::unordered_map<uint16_t, DmActData>* data_ptr_;
    SocketCAN socket_can_;

    std::atomic<bool> read_write_save{false};
    std::vector<CanFrameStamp> read_buffer_;
    mutable std::mutex mutex_;

    // 运动参数
    MotionParams motion_params_;
};

};  // namespace damiao

#endif  // DAMIAO_H
