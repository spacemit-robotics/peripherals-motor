#ifndef DMHW_H
#define DMHW_H

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "damiao.h"

namespace damiao {
using sysclock = std::chrono::system_clock;
using duration = std::chrono::duration<double>;

/**
 * @struct MotorConfig
 * @brief 电机配置结构
 */
struct MotorConfig {
    std::string bus_name;       ///< CAN 总线名称（如 "can0"）
    uint16_t can_id;            ///< 电机 CAN ID
    uint16_t master_id;         ///< 主机 ID
    DM_Motor_Type motor_type;   ///< 电机型号（DM4310, DM4340 等）
    Control_Mode control_mode;  ///< 控制模式（MIT_MODE, POS_VEL_MODE 等）
};

/**
 * @class DmHW
 * @brief 硬件接口抽象层 - 管理所有 CAN 总线和电机
 *
 * 该类负责：
 * - 初始化 CAN 总线和电机
 * - 管理多个 Motor_Control 对象（每个对应一条 CAN 总线）
 * - 提供统一的读写接口
 *
 * 使用流程：
 * 1. 创建 DmHW 对象
 * 2. 调用 setCanBusThreadPriority() 设置线程优先级
 * 3. 调用 init() 初始化 CAN 总线
 * 4. 在控制循环中调用 read() 和 write()
 */
class DmHW {
public:
    // 周期性自动读取接口
    void startAutoRead(unsigned int period_ms);
    void stopAutoRead();
    /// @brief 默认构造函数
    DmHW() = default;

    /// @brief 析构函数：确保停止自动读取线程以避免程序退出时异常终止
    ~DmHW() {
        stopAutoRead();
    }

    /**
     * @brief 初始化 CAN 总线和电机
     * @param motor_configs 电机配置列表
     * @return true 初始化成功，false 初始化失败
     *
     * 该函数会：
     * - 为每条 CAN 总线创建一个 Motor_Control 对象
     * - 根据配置初始化电机数据结构
     * - 打开 CAN 接口并启动接收线程
     *
     * @note 必须在 setCanBusThreadPriority() 之后调用
     */
    bool init(const std::vector<MotorConfig>& motor_configs);

    /**
     * @brief 从所有 CAN 总线读取电机反馈数据
     * @param time 当前系统时间
     * @param period 控制周期
     *
     * 该函数会遍历所有 Motor_Control 对象，调用其 read() 方法
     * 从 CAN 总线读取电机的位置、速度、力矩等状态信息
     *
     * @note 在控制循环的 read 阶段调用
     */
    void read(const sysclock::time_point& time, const duration& period);

    /**
     * @brief 向所有 CAN 总线发送控制命令
     * @param time 当前系统时间
     * @param period 控制周期
     *
     * 该函数会遍历所有 Motor_Control 对象，调用其 write() 方法
     * 将控制命令（位置、速度、力矩等）发送到电机
     *
     * @note 在控制循环的 write 阶段调用
     */
    void write(const sysclock::time_point& time, const duration& period);

    /**
     * @brief 设置 CAN 接收线程的优先级
     * @param thread_priority 线程优先级 (0-99，越大优先级越高)
     *
     * 建议值：95（接近最高优先级，保证实时性）
     *
     * @note 必须在 init() 之前调用
     */
    void setCanBusThreadPriority(int thread_priority);

    /**
     * @brief 设置所有电机的运动参数
     * @param params 运动参数结构体
     *
     * 该函数会将运动参数设置到所有 Motor_Control 对象
     *
     * @note 可以在 init() 之后、控制循环开始之前调用
     */
    void setMotionParams(const MotionParams& params);

    /**
     * @brief 发送 MIT 控制命令给指定电机
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @param p 目标位置
     * @param v 目标速度
     * @param t 前馈力矩
     * @param kp 位置刚度
     * @param kd 速度阻尼
     */
    void sendMitCommand(const std::string& bus_name, uint16_t can_id, float p, float v, float t, float kp, float kd);

    /**
     * @brief 设置所有电机的当前位置为零点
     *
     * 该函数会遍历所有 Motor_Control 对象，调用其 set_zero_position() 方法
     * 将所有电机的当前位置设置为零点
     *
     * @note 通常在程序退出前调用，用于位置标定
     */
    void setZeroPosition();

    /**
     * @brief 平稳停止所有电机
     *
     * 该函数会让所有电机以当前位置为目标，速度设为0，平稳停止
     * 避免突然停止造成的抖动
     *
     * @note 在紧急停止或程序中断时调用
     */
    void emergencyStop();

    /**
     * @brief Disable all motors
     *
     * Explicitly sends the disable command (0xFD) to all motors.
     */
    void disable();

    /**
     * @brief 获取所有电机状态数据
     * @return 电机状态数据映射表
     */
    const std::unordered_map<std::string, std::unordered_map<uint16_t, damiao::DmActData>>& getActuatorData() const;

private:
    bool is_actuator_specified_ = false;
    int thread_priority_;

    // 用于周期性自动读取的线程控制变量
    bool read_running_ = false;
    std::thread read_thread_;

    std::vector<std::shared_ptr<damiao::Motor_Control>> motor_ports_{};

    std::unordered_map<std::string, std::unordered_map<uint16_t, damiao::DmActData>> bus_id2dm_data_{};
};

}  // namespace damiao

#endif  // DMHW_H
