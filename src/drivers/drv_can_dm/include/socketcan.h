/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 3/3/21.
//

#ifndef SOCKETCAN_H
#define SOCKETCAN_H

#pragma once

#include <linux/can.h>
#include <net/if.h>
#include <pthread.h>

#include <chrono>
#include <iostream>
#include <string>

#include <boost/function.hpp>

namespace damiao {

/**
 * @class SocketCAN
 * @brief CAN 总线底层驱动 - 基于 Linux SocketCAN 接口
 *
 * 该类负责：
 * - 打开和关闭 CAN 接口
 * - 发送和接收 CAN 帧
 * - 管理 CAN 接收线程
 * - 处理 CAN 帧回调
 *
 * 使用流程：
 * 1. 调用 open() 打开 CAN 接口
 * 2. 调用 startReceiverThread() 启动接收线程
 * 3. 调用 write() 发送 CAN 帧
 * 4. 接收线程自动调用回调函数处理接收到的帧
 * 5. 调用 close() 关闭接口
 */
class SocketCAN {
private:
    ifreq interface_request_{};       ///< 网络接口请求结构体
    sockaddr_can address_{};          ///< CAN 地址结构体
    pthread_t receiver_thread_id_{};  ///< 接收线程 ID

public:
    int sock_fd_ = -1;                        ///< CAN socket 文件描述符
    bool terminate_receiver_thread_ = false;  ///< 接收线程终止标志
    bool receiver_thread_running_ = false;    ///< 接收线程运行状态

    SocketCAN() = default;
    ~SocketCAN();

    /**
     * @brief 记录限流错误日志
     * @param interface_name CAN 接口名称
     *
     * 用于避免日志过多
     */
    void log_throttled_error(const std::string& interface_name) const;

    /**
     * @brief 打开并绑定 CAN 接口
     * @param interface CAN 接口名称（例如 "can0"）
     * @param handler 接收回调函数，当收到 CAN 帧时被调用
     * @param thread_priority 接收线程优先级 (0-99)
     * @return true 打开成功，false 打开失败
     *
     * 该函数会：
     * - 创建 CAN socket
     * - 绑定到指定的 CAN 接口
     * - 设置非阻塞模式
     * - 保存回调函数指针
     *
     * @note 需要 root 权限或 CAP_NET_RAW 能力
     */
    bool open(const std::string& interface, boost::function<void(const can_frame& frame)> handler, int thread_priority);

    /**
     * @brief 关闭并解绑 CAN 接口
     *
     * 该函数会：
     * - 停止接收线程
     * - 关闭 socket
     * - 释放资源
     */
    void close();

    /**
     * @brief 检查 CAN 接口是否打开
     * @return true 接口已打开，false 接口已关闭
     */
    bool isOpen() const;

    /**
     * @brief 发送 CAN 帧
     * @param frame 指向 CAN 帧的指针
     *
     * 通过 CAN 总线发送数据帧到电机
     *
     * @note 该函数是非阻塞的
     */
    void write(can_frame* frame) const;

    /**
     * @brief 启动 CAN 接收线程
     * @param thread_priority 线程优先级 (0-99)
     * @return true 启动成功，false 启动失败
     *
     * 创建一个新线程，持续监听 CAN 总线
     * 当收到数据帧时，调用 reception_handler 回调函数
     *
     * @note 该函数在 open() 中自动调用
     */
    bool startReceiverThread(int thread_priority);
    /**
     * Pointer to a function which shall be called
     * when frames are being received from the CAN bus
     */
    boost::function<void(const can_frame& frame)> reception_handler;
};

}  // namespace damiao

#endif  // SOCKETCAN_H
