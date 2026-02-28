#pragma once

#include "DmHW.h"

#include <chrono>
#include <thread>


namespace damiao
{
using clock = std::chrono::steady_clock;
using duration = std::chrono::duration<double>;

/**
 * @class DmHWLoop
 * @brief 实时控制循环管理器
 * 
 * 该类负责：
 * - 创建并管理实时控制线程（500Hz，2ms 周期）
 * - 执行控制循环：read → 控制 → write
 * - 监测周期超时并报警
 * - 使用 SCHED_FIFO 实时调度策略
 * 
 * 特性：
 * - 固定 500Hz 控制频率
 * - 线程优先级 95（接近最高）
 * - 周期超时阈值 2ms
 * - 精确的 sleep_until() 时间控制
 */
class DmHWLoop
{
public:

  /**
   * @brief 构造函数 - 启动实时控制线程
   * @param hardware_interface 硬件接口对象指针
   * 
   * 该构造函数会：
   * 1. 初始化控制参数（500Hz，2ms 周期）
   * 2. 调用 hardware_interface->init() 初始化硬件
   * 3. 创建实时线程，运行 update() 循环
   * 4. 设置线程为 SCHED_FIFO 实时调度
   * 
   * @note 构造函数中会阻塞直到线程启动完成
   */
  DmHWLoop(std::shared_ptr<DmHW> hardware_interface);

  /**
   * @brief 构造函数 - 使用自定义电机配置启动实时控制线程
   * @param hardware_interface 硬件接口对象指针
   * @param motor_configs 电机配置列表
   */
  DmHWLoop(std::shared_ptr<DmHW> hardware_interface, const std::vector<MotorConfig>& motor_configs);

  /**
   * @brief 析构函数 - 安全停止控制线程
   * 
   * 该析构函数会：
   * 1. 设置 loopRunning_ = false 停止循环
   * 2. 等待线程结束（join）
   * 3. 释放所有资源
   */
  ~DmHWLoop();

  /**
   * @brief 单次控制循环更新
   * 
   * 执行流程：
   * 1. 计算当前时间和周期
   * 2. 检测周期超时
   * 3. 调用 hardware_interface->read() 读取电机状态
   * 4. [用户可在此添加控制算法]
   * 5. 调用 hardware_interface->write() 发送控制命令
   * 6. 精确睡眠到下一个周期
   * 
   * @note 该函数在实时线程中以 500Hz 频率调用
   */
  void update();

private:

  // Settings
  double cycle_time_error_threshold_{};

  // Timing
  std::thread loop_thread_;
  std::atomic_bool loopRunning_;
  double loop_hz_{};
  duration elapsed_time_;
  clock::time_point last_time_;

  // Abstract Hardware Interface for your robot
  std::shared_ptr<DmHW> hardware_interface_;
};
}  // namespace rm_hw
