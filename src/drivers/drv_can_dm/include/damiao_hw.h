#ifndef DAMIAO_HW_H
#define DAMIAO_HW_H

#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "damiao.h"

namespace damiao {

/**
 * @struct MotorConfig
 * @brief 电机配置结构
 */
struct MotorConfig {
    std::string bus_name;       ///< CAN 总线名称（如 "can0"）
    uint16_t can_id;            ///< 电机 CAN ID
    uint16_t master_id;         ///< 主机 ID
    DM_Motor_Type motor_type;   ///< 电机型号
    Control_Mode control_mode;  ///< 初始控制模式
};

/**
 * @class DamiaoHW
 * @brief 达妙电机硬件抽象层
 *
 * 功能：
 * 1. 多总线管理 - 支持同时管理多条 CAN 总线
 * 2. 多模式管理 - 支持运行时模式切换
 * 3. 模式化控制 - 为每种模式提供专用控制函数
 * 4. 参数调节 - 支持读写电机寄存器参数
 */
class DamiaoHW {
public:
    DamiaoHW() = default;
    ~DamiaoHW() { stopAutoRead(); }

    // ========== 初始化接口 ==========

    /**
     * @brief 初始化硬件接口
     * @param motor_configs 电机配置列表
     * @return true 成功，false 失败
     */
    bool init(const std::vector<MotorConfig>& motor_configs);

    /**
     * @brief 设置 CAN 接收线程优先级
     * @param priority 线程优先级 (0-99)
     */
    void setThreadPriority(int priority);

    // ========== 数据读写接口 ==========

    /**
     * @brief 读取所有电机状态
     */
    void read();

    /**
     * @brief 获取电机状态数据
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @return 电机数据指针，未找到返回 nullptr
     */
    DmActData* getMotorData(const std::string& bus_name, uint16_t can_id);

    // ========== 模式管理接口 ==========

    /**
     * @brief 切换电机控制模式
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @param mode 目标模式
     * @return true 成功，false 失败
     */
    bool switchMode(const std::string& bus_name, uint16_t can_id, Control_Mode_Code mode);

    /**
     * @brief 获取电机当前模式
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @return 当前模式
     */
    Control_Mode getCurrentMode(const std::string& bus_name, uint16_t can_id);

    // ========== 模式化控制接口 ==========

    /**
     * @brief MIT 阻抗控制
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @param pos 目标位置 (rad)
     * @param vel 目标速度 (rad/s)
     * @param torque 前馈力矩 (Nm)
     * @param kp 位置刚度 (0-500)
     * @param kd 阻尼系数 (0-5)
     */
    void controlMit(const std::string& bus_name, uint16_t can_id, float pos, float vel, float torque, float kp,
                    float kd);

    /**
     * @brief 位置+速度控制
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @param pos 目标位置 (rad)
     * @param vel 目标速度 (rad/s)
     */
    void controlPosVel(const std::string& bus_name, uint16_t can_id, float pos, float vel);

    /**
     * @brief 速度控制
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @param vel 目标速度 (rad/s)
     */
    void controlVel(const std::string& bus_name, uint16_t can_id, float vel);

    /**
     * @brief 位置+力矩控制（力位混控模式）
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @param pos 目标位置 (rad)
     * @param vel_limit 速度限制 (rad/s, 0-100)
     * @param current_limit 电流限制标幺值 (0-1.0)
     */
    void controlPosForce(const std::string& bus_name, uint16_t can_id, float pos, float vel_limit, float current_limit);

    // ========== 参数调节接口 ==========

    /**
     * @brief 读取电机寄存器参数
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @param reg_id 寄存器地址（参考 DM_REG 枚举）
     * @return 参数值（异步读取，需等待回调）
     */
    float readParam(const std::string& bus_name, uint16_t can_id, uint8_t reg_id);

    /**
     * @brief 写入电机寄存器参数
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @param reg_id 寄存器地址
     * @param value 参数值
     * @return true 成功，false 失败
     */
    bool writeParam(const std::string& bus_name, uint16_t can_id, uint8_t reg_id, float value);

    /**
     * @brief 保存电机参数到 Flash
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     */
    void saveParam(const std::string& bus_name, uint16_t can_id);

    /**
     * @brief 获取电机缓存的参数值
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @param reg_id 寄存器地址
     * @return 参数值，未找到返回 0
     */
    float getCachedParam(const std::string& bus_name, uint16_t can_id, uint8_t reg_id);

    // ========== 电机控制接口 ==========

    /**
     * @brief 使能所有电机
     */
    void enableAll();

    /**
     * @brief 失能所有电机
     */
    void disableAll();

    /**
     * @brief 设置电机零点
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     */
    void setZeroPosition(const std::string& bus_name, uint16_t can_id);

    /**
     * @brief 启动自动读取线程
     * @param period_ms 读取周期 (毫秒)
     */
    void startAutoRead(unsigned int period_ms);

    /**
     * @brief 停止自动读取线程
     */
    void stopAutoRead();

private:
    /**
     * @brief 查找电机控制器和电机对象
     * @param bus_name 总线名称
     * @param can_id 电机 ID
     * @return pair<Motor_Control*, Motor*>，未找到返回 {nullptr, nullptr}
     */
    std::pair<Motor_Control*, Motor*> findMotor(const std::string& bus_name, uint16_t can_id);

    int thread_priority_ = 95;

    // 总线名称 -> Motor_Control 映射
    std::unordered_map<std::string, std::shared_ptr<Motor_Control>> bus_controllers_;

    // 总线名称 -> (电机ID -> 电机数据) 映射
    std::unordered_map<std::string, std::unordered_map<uint16_t, DmActData>> bus_motor_data_;

    // 自动读取线程
    bool read_running_ = false;
    std::thread read_thread_;
};

}  // namespace damiao

#endif  // DAMIAO_HW_H
