#include "damiao.h"

#include <signal.h>

#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include <boost/bind/bind.hpp>

namespace damiao {

Limit_param limit_param[Num_Of_Motor] = {
    {12.5, 30, 10},   // DM4310
    {12.5, 50, 10},   // DM4310_48V
    {12.5, 10, 28},   // DM4340
    {12.5, 10, 28},   // DM4340_48V
    {12.5, 45, 12},   // DM6006
    {12.5, 45, 20},   // DM8006
    {12.5, 45, 54},   // DM8009
    {12.5, 25, 200},  // DM10010L
    {12.5, 20, 200},  // DM10010
    {12.5, 280, 1},   // DMH3510
    {12.5, 45, 10},   // DMH6215
    {12.5, 45, 10}    // DMG6220
};

Motor::Motor(DM_Motor_Type motor_type, Control_Mode ctrl_mode, uint16_t can_id, uint16_t master_id)
    : Motor_Type(motor_type), mode(ctrl_mode), Master_id(master_id), Can_id(can_id) {
    this->limit_param = damiao::limit_param[motor_type];
}

void Motor::receive_data(float q, float dq, float tau) {
    this->state_q = q;
    this->state_dq = dq;
    this->state_tau = tau;
}

void Motor::set_param(int key, float value) {
    ValueType v{};
    v.value.floatValue = value;
    v.isFloat = true;
    param_map[key] = v;
}

void Motor::set_param(int key, uint32_t value) {
    ValueType v{};
    v.value.uint32Value = value;
    v.isFloat = false;
    param_map[key] = v;
}

float Motor::get_param_as_float(int key) const {
    auto it = param_map.find(key);
    if (it != param_map.end()) {
        if (it->second.isFloat) {
            return it->second.value.floatValue;
        } else {
            return 0;
        }
    }
    return 0;
}

uint32_t Motor::get_param_as_uint32(int key) const {
    auto it = param_map.find(key);
    if (it != param_map.end()) {
        if (!it->second.isFloat) {
            return it->second.value.uint32Value;
        } else {
            return 0;
        }
    }
    return 0;
}

bool Motor::is_have_param(int key) const {
    return param_map.find(key) != param_map.end();
}

/******一个can，一个Motor_Control**********************/
Motor_Control::Motor_Control(std::string bus_name, std::unordered_map<uint16_t, DmActData>* data_ptr)
    : data_ptr_(data_ptr) {
    for (auto it = data_ptr_->begin(); it != data_ptr_->end(); ++it) {  // 遍历该bus下的所有电机
        std::shared_ptr<Motor> motor =
            std::make_shared<Motor>(it->second.motorType, it->second.mode, it->second.can_id, it->second.mst_id);
        addMotor(motor);
    }
    int thread_priority = 95;
    while (!socket_can_.open(
        bus_name, boost::bind(&Motor_Control::canframeCallback, this, boost::placeholders::_1), thread_priority))

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    enable_all();  // 使能该接口下的所有电机
    // usleep(1000000);//1s
    std::cout << "Motor_Control init success!" << std::endl;
}

Motor_Control::~Motor_Control() {
    std::cout << "enter ~Motor_Control" << std::endl;

    disable_all();  // 使能该接口下的所有电机
}

/**
 * @brief add motor to class 添加电机
 * @param DM_Motor : motor object 电机对象
 */
void Motor_Control::addMotor(std::shared_ptr<Motor> DM_Motor) {
    motors.insert({DM_Motor->GetCanId(), DM_Motor});
    motors.insert({DM_Motor->GetMasterId(), DM_Motor});
}

void Motor_Control::enable_all() {
    for (auto& it : motors) {
        for (int j = 0; j < 5; j++) {
            control_cmd(it.second->GetCanId() + it.second->GetMotorMode(), 0xFC);
            usleep(2000);
        }
    }
}

void Motor_Control::disable_all() {
    for (auto& it : motors) {
        for (int j = 0; j < 5; j++) {
            control_cmd(it.second->GetCanId() + it.second->GetMotorMode(), 0xFD);
            usleep(2000);
        }
    }
}

/*
 * @description: read motor register param
 * 读取电机内部寄存器参数，具体寄存器列表请参考达妙的手册
 * @param DM_Motor: motor object 电机对象
 * @param RID: register id 寄存器ID  example: damiao::UV_Value
 * @return: motor param 电机参数 如果没查询到返回的参数为0
 */
float Motor_Control::read_motor_param(Motor& DM_Motor, uint8_t RID) {
    read_write_save = true;  // 发送读参数命令，返回的数据和常规返回的不一样，需要单独处理
    uint16_t id = DM_Motor.GetCanId();
    uint8_t id_low = id & 0xff;
    uint8_t id_high = (id >> 8) & 0xff;
    can_frame frame{};
    frame.can_id = 0x7FF;
    frame.can_dlc = 8;

    frame.data[0] = id_low;
    frame.data[1] = id_high;
    frame.data[2] = 0x33;
    frame.data[3] = RID;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    socket_can_.write(&frame);
    return 0;
}

/*
 * @description: save all param to motor flash 保存电机的所有参数到flash里面
 * @param DM_Motor: motor object 电机对象
 * 电机默认参数不会写到flash里面，需要进行写操作
 */
void Motor_Control::save_motor_param(Motor& DM_Motor) {
    uint16_t id = DM_Motor.GetCanId();
    uint16_t mode = DM_Motor.GetMotorMode();
    control_cmd(id + mode, 0xFD);  // 失能
    usleep(10000);
    read_write_save = true;  // 发送保存参数命令，返回的数据和常规返回的不一样，需要单独处理

    uint8_t id_low = id & 0xff;
    uint8_t id_high = (id >> 8) & 0xff;

    can_frame frame{};
    frame.can_id = 0x7FF;
    frame.can_dlc = 8;

    frame.data[0] = id_low;
    frame.data[1] = id_high;
    frame.data[2] = 0xAA;
    frame.data[3] = 0x01;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;
    socket_can_.write(&frame);
    usleep(100000);
}

// 读电机反馈命令
void Motor_Control::refresh_motor_status(Motor& motor) {
    uint8_t can_low = motor.GetCanId() & 0xff;          // id low 8 bit
    uint8_t can_high = (motor.GetCanId() >> 8) & 0xff;  // id high 8 bit

    can_frame frame{};
    frame.can_id = 0x7FF;
    frame.can_dlc = 4;
    frame.data[0] = can_low;
    frame.data[1] = can_high;
    frame.data[2] = 0xCC;
    frame.data[3] = 0x00;

    socket_can_.write(&frame);
}

void Motor_Control::control_cmd(uint16_t id, uint8_t cmd) {
    can_frame frame{};
    frame.can_id = id;
    frame.can_dlc = 8;

    frame.data[0] = 0xff;
    frame.data[1] = 0xff;
    frame.data[2] = 0xff;
    frame.data[3] = 0xff;
    frame.data[4] = 0xff;
    frame.data[5] = 0xff;
    frame.data[6] = 0xff;
    frame.data[7] = cmd;
    socket_can_.write(&frame);
}

void Motor_Control::write_motor_param(Motor& DM_Motor, uint8_t RID, const uint8_t data[4]) {
    read_write_save = true;  // 发送写参数命令，返回的数据和常规返回的不一样，需要单独处理

    uint16_t id = DM_Motor.GetCanId();
    uint8_t can_low = id & 0xff;
    uint8_t can_high = (id >> 8) & 0xff;

    can_frame frame{};
    frame.can_id = 0x7FF;
    frame.can_dlc = 8;

    frame.data[0] = can_low;
    frame.data[1] = can_high;
    frame.data[2] = 0x55;
    frame.data[3] = RID;
    frame.data[4] = data[0];
    frame.data[5] = data[1];
    frame.data[6] = data[2];
    frame.data[7] = data[3];
    socket_can_.write(&frame);
}

void Motor_Control::set_zero_position(Motor& DM_Motor) {
    control_cmd(DM_Motor.GetCanId() + DM_Motor.GetMotorMode(), 0xFE);
}

/* 电机控制函数

    DM_Motor: 目标电机对象
    kp: 位置刚度增益 (0-500)
    kd: 阻尼增益 (0-5)
    q: 目标位置 (rad)
    dq: 目标速度 (rad/s)
    tau: 前馈力矩 (Nm)

*/

void Motor_Control::control_mit(Motor& DM_Motor, float kp, float kd, float q, float dq, float tau) {
    // 位置、速度和扭矩采用线性映射的关系将浮点型数据转换成有符号的定点数据
    static auto float_to_uint = [](float x, float xmin, float xmax, uint8_t bits) -> uint16_t {
        float span = xmax - xmin;
        float data_norm = (x - xmin) / span;

        // Clamping to prevent overflow
        if (data_norm < 0)
            data_norm = 0;
        if (data_norm > 1)
            data_norm = 1;

        uint16_t data_uint = data_norm * ((1 << bits) - 1);
        return data_uint;
    };
    uint16_t id = DM_Motor.GetCanId();
    if (motors.find(id) == motors.end()) {
        std::cerr << "[Error] In control_mit,no motor with id " << DM_Motor.GetCanId() << " is registered."
                << std::endl;
        std::exit(-1);  // 终止程序，返回非 0 表示错误
    }
    auto& m = motors[id];
    uint16_t kp_uint = float_to_uint(kp, 0, 500, 12);
    uint16_t kd_uint = float_to_uint(kd, 0, 5, 12);
    Limit_param limit_param_cmd = m->get_limit_param();
    uint16_t q_uint = float_to_uint(q, -limit_param_cmd.Q_MAX, limit_param_cmd.Q_MAX, 16);
    uint16_t dq_uint = float_to_uint(dq, -limit_param_cmd.DQ_MAX, limit_param_cmd.DQ_MAX, 12);
    uint16_t tau_uint = float_to_uint(tau, -limit_param_cmd.TAU_MAX, limit_param_cmd.TAU_MAX, 12);

    can_frame frame{};
    frame.can_id = id + MIT_MODE;
    frame.can_dlc = 8;
    frame.data[0] = (q_uint >> 8) & 0xff;
    frame.data[1] = q_uint & 0xff;
    frame.data[2] = dq_uint >> 4;
    frame.data[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf);
    frame.data[4] = kp_uint & 0xff;
    frame.data[5] = kd_uint >> 4;
    frame.data[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf);
    frame.data[7] = tau_uint & 0xff;

    socket_can_.write(&frame);
}

void Motor_Control::control_pos_vel(Motor& DM_Motor, float pos, float vel) {
    uint16_t id = DM_Motor.GetCanId();
    if (motors.find(id) == motors.end()) {
        std::cerr << "[Error] In control_pos_vel,no motor with id " << DM_Motor.GetCanId() << " is registered."
                << std::endl;
        std::exit(-1);  // 终止程序，返回非 0 表示错误
    }
    can_frame frame{};
    frame.can_id = id + POS_VEL_MODE;
    frame.can_dlc = 8;
    uint8_t *pbuf, *vbuf;
    pbuf = reinterpret_cast<uint8_t*>(&pos);
    vbuf = reinterpret_cast<uint8_t*>(&vel);

    frame.data[0] = *pbuf;
    frame.data[1] = *(pbuf + 1);
    frame.data[2] = *(pbuf + 2);
    frame.data[3] = *(pbuf + 3);
    frame.data[4] = *vbuf;
    frame.data[5] = *(vbuf + 1);
    frame.data[6] = *(vbuf + 2);
    frame.data[7] = *(vbuf + 3);

    socket_can_.write(&frame);
}

void Motor_Control::control_vel(Motor& DM_Motor, float vel) {
    uint16_t id = DM_Motor.GetCanId();
    if (motors.find(id) == motors.end()) {
        std::cerr << "[Error] In control_vel,no motor with id " << DM_Motor.GetCanId() << " is registered."
                << std::endl;
        std::exit(-1);  // 终止程序，返回非 0 表示错误
    }
    can_frame frame{};
    frame.can_id = id + VEL_MODE;
    frame.can_dlc = 4;
    uint8_t* vbuf;
    vbuf = reinterpret_cast<uint8_t*>(&vel);

    frame.data[0] = *vbuf;
    frame.data[1] = *(vbuf + 1);
    frame.data[2] = *(vbuf + 2);
    frame.data[3] = *(vbuf + 3);

    socket_can_.write(&frame);
}

void Motor_Control::receive_param(uint8_t* data) {
    uint16_t canID = (static_cast<uint16_t>(data[1]) << 8) | data[0];
    uint8_t RID = data[3];
    if (motors.find(canID) == motors.end()) {
        std::cerr << "[Error] In receive_param,no motor with id " << canID << " is registered." << std::endl;
        std::exit(-1);  // 终止程序，返回非 0 表示错误
    }
    if (is_in_ranges(RID)) {
        uint32_t data_uint32 = (static_cast<uint32_t>(data[7]) << 24) | (static_cast<uint32_t>(data[6]) << 16) |
                                (static_cast<uint32_t>(data[5]) << 8) | data[4];
        motors[canID]->set_param(RID, data_uint32);
        if (RID == 10) {
            if (data_uint32 == 1) {
                motors[canID]->set_mode(MIT_MODE);
            } else if (data_uint32 == 2) {
                motors[canID]->set_mode(POS_VEL_MODE);
            } else if (data_uint32 == 3) {
                motors[canID]->set_mode(VEL_MODE);
            } else if (data_uint32 == 4) {
                motors[canID]->set_mode(POS_FORCE_MODE);
            }
        }
    } else {
        float data_float = uint8_to_float(data + 4);
        motors[canID]->set_param(RID, data_float);
    }
}

/*
 * @description: switch control mode 切换电机控制模式
 * @note: 模式切换需要写 flash, 在高速模式下避免模式切换, 掉电后模式会丢失
 * @param DM_Motor: motor object 电机对象
 * @param mode: control mode 控制模式 like:damiao::MIT_MODE,
 * damiao::POS_VEL_MODE, damiao::VEL_MODE, damiao::POS_FORCE_MODE
 */
bool Motor_Control::switchControlMode(Motor& DM_Motor, Control_Mode_Code mode) {
    uint8_t write_data[4] = {(uint8_t)mode, 0x00, 0x00, 0x00};
    uint8_t RID = 10;
    write_motor_param(DM_Motor, RID, write_data);
    if (motors.find(DM_Motor.GetCanId()) == motors.end()) {
        std::cerr << "[Error] In switchControlMode,no motor with id " << DM_Motor.GetCanId() << " is registered."
                << std::endl;
        std::exit(-1);  // 终止程序
        return false;
    }

    return true;
}

/*
 * @description: change motor param 修改电机内部寄存器参数
 * 具体寄存器列表请参考达妙手册
 * @param DM_Motor: motor object 电机对象
 * @param RID: register id 寄存器ID
 * @param data: param data
 * 参数数据,大部分数据是float类型，其中如果是uint32类型的数据也可以直接输入整型的就行，函数内部有处理
 * @return: bool true or false  是否修改成功
 */
bool Motor_Control::change_motor_param(Motor& DM_Motor, uint8_t RID, float data) {
    if (is_in_ranges(RID)) {
        // 居然传进来的是整型的范围 救一下
        uint32_t data_uint32 = float_to_uint32(data);
        uint8_t* data_uint8;
        data_uint8 = reinterpret_cast<uint8_t*>(&data_uint32);
        write_motor_param(DM_Motor, RID, data_uint8);
    } else {
        // is float
        uint8_t* data_uint8;
        data_uint8 = reinterpret_cast<uint8_t*>(&data);
        write_motor_param(DM_Motor, RID, data_uint8);
    }
    if (motors.find(DM_Motor.GetCanId()) == motors.end()) {
        std::cerr << "[Error] In change_motor_param,no motor with id " << DM_Motor.GetCanId() << " is registered."
                << std::endl;
        std::exit(-1);  // 终止程序，返回非 0 表示错误
        return false;
    }
    return true;
}

/*
 * @description: change motor limit param
 * 修改电机限制参数，这个修改的不是电机内部的寄存器参数，而是电机的限制参数
 * @param DM_Motor: motor object 电机对象
 * @param P_MAX: position max 位置最大值
 * @param Q_MAX: velocity max 速度最大值
 * @param T_MAX: torque max 扭矩最大值
 */
void Motor_Control::changeMotorLimit(Motor& DM_Motor, float P_MAX, float Q_MAX, float T_MAX) {
    limit_param[DM_Motor.GetMotorType()] = {P_MAX, Q_MAX, T_MAX};
}

void Motor_Control::write() {
    static bool initialized = false;
    static std::chrono::time_point<std::chrono::system_clock> start_time;
    static std::unordered_map<int, float> initial_positions;

    // 兼容用例传入 轨迹规划：运动圈数、加减速时间、运动总时间、kp、kd （平滑差）

    // 首次运行时记录起始时间和初始位置
    if (!initialized) {
        start_time = std::chrono::system_clock::now();
        for (const auto& m : *data_ptr_) {
            initial_positions[m.first] = m.second.pos;
        }
        initialized = true;
    }

    // 计算运行时间
    auto now = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = now - start_time;
    double t = elapsed.count();

    // 使用传入的运动参数
    const double total_revolutions = motion_params_.total_revolutions;
    const double total_angle = total_revolutions * 2.0 * M_PI;
    const double motion_time = motion_params_.motion_time;

    for (const auto& m : *data_ptr_) {  // 遍历该can接口下的所有电机
        int motor_id = m.first;         // 这里指的是can_id
        if (motors.find(motor_id) == motors.end()) {
            std::cerr << "[Error] In write,no motor with id " << motor_id << " is registered." << std::endl;
            std::exit(-1);  // 终止程序，返回非 0 表示错误
        }
        auto& it = motors[motor_id];

        float target_pos, target_vel;

        if (t < motion_time) {
            // 使用S型曲线（平滑的梯形速度曲线）实现平滑运动
            double accel_time = motion_params_.accel_time;
            double decel_time = motion_params_.accel_time;  // 减速时间等于加速时间
            double decel_start = motion_time - decel_time;
            double const_time = decel_start - accel_time;  // 匀速阶段时间

            // 计算最大速度（匀速阶段的速度）
            double v_max = total_angle / (motion_time - 0.5 * accel_time - 0.5 * decel_time);

            if (t < accel_time) {
                // 加速阶段：使用三次多项式实现平滑加速（S曲线）
                double s = t / accel_time;  // 归一化时间 [0, 1]
                double s2 = s * s;
                double s3 = s2 * s;
                // 使用3次多项式：3s^2 - 2s^3，保证加速度连续
                double smooth_factor = 3 * s2 - 2 * s3;
                target_vel = v_max * smooth_factor;
                // 位置是速度的积分
                target_pos = initial_positions[motor_id] + v_max * accel_time * (s3 - 0.5 * s2 * s2);
            } else if (t < decel_start) {
                // 匀速阶段
                double accel_distance = v_max * accel_time * 0.5;  // 加速阶段的距离
                target_vel = v_max;
                target_pos = initial_positions[motor_id] + accel_distance + v_max * (t - accel_time);
            } else {
                // 减速阶段：使用三次多项式实现平滑减速（S曲线）
                double t_decel = t - decel_start;  // 减速阶段经过的时间
                double s = t_decel / decel_time;   // 归一化时间 [0, 1]
                double s2 = s * s;
                double s3 = s2 * s;
                // 使用3次多项式：1 - 3s^2 + 2s^3，从1平滑降到0
                double smooth_factor = 1.0 - 3 * s2 + 2 * s3;
                target_vel = v_max * smooth_factor;
                // 计算减速阶段的位置
                double accel_distance = v_max * accel_time * 0.5;
                double const_distance = v_max * const_time;
                double decel_distance = v_max * decel_time * (s - s3 + 0.5 * s2 * s2);
                target_pos = initial_positions[motor_id] + accel_distance + const_distance + decel_distance;
            }

            // MIT 模式：位置 + 速度控制，使用传入的 kp 和 kd 参数
            control_mit(*it, motion_params_.kp, motion_params_.kd, target_pos, target_vel, 0);

            // 打印进度信息（每0.5秒打印一次）
            static double last_print_time = 0;
            if (t - last_print_time > 0.5) {
                // std::cout << "[Motor " << motor_id << "] Progress: "
                //           << (t/motion_time*100.0) << "%" << std::endl;
                last_print_time = t;
            }
        } else {
            // 运动完成，设置零点并准备退出
            static bool motion_completed = false;

            if (!motion_completed) {
                // 第一次进入完成状态，保持位置稳定一段时间
                target_pos = initial_positions[motor_id] + total_angle;
                target_vel = 0;
                control_mit(*it, motion_params_.kp, motion_params_.kd, target_pos, 0, 0);

                /*不置零点，根据 MIT 协议公式，目标位置与与实际位置的差值小于 P_MAX，参数配置得当电机会正常动作*/
                // 等待一段时间后设置零点
                /// std::this_thread::sleep_for(std::chrono::milliseconds(500));
                // set_zero_position(*it);

                // std::cout << "[Motor Control] Motion completed! " << std::endl;
                motion_completed = true;
            } else {
                // Keep holding position
                target_pos = initial_positions[motor_id] + total_angle;
                control_mit(*it, motion_params_.kp, motion_params_.kd, target_pos, 0, 0);
            }
        }
    }
}

void Motor_Control::read() {
    for (auto& m : *data_ptr_) {      // 遍历该can接口下的所有电机
        uint16_t motor_id = m.first;  // 这里指的是can_id
        if (motors.find(motor_id) == motors.end()) {
            std::cerr << "[Error] In read,no motor with id " << motor_id << " is registered." << std::endl;
            std::exit(-1);  // 终止程序，返回非 0 表示错误
        }
        auto& it = motors[motor_id];

        m.second.pos = it->Get_Position();
        m.second.vel = it->Get_Velocity();
        m.second.effort = it->Get_tau();
        // std::cerr<<"MotorType: "<<it->GetMotorType()<<std::endl;
        std::cerr << "pos: " << m.second.pos << " vel: " << m.second.vel << " effort: " << m.second.effort << std::endl;
    }
}

void Motor_Control::canframeCallback(const can_frame& frame) {
    // 使用std::lock_guard自动管理mutex_的锁定和解锁，以确保线程安全
    // 当guard对象被创建时，它会自动锁定mutex_；当guard对象被销毁（例如，离开作用域时），它会自动解锁mutex_
    std::lock_guard<std::mutex> guard(mutex_);
    // CanFrameStamp can_frame_stamp{ .frame = frame, .stamp =
    // std::chrono::system_clock::now() };
    CanFrameStamp can_frame_stamp{.frame = frame};
    read_buffer_.push_back(can_frame_stamp);
    // 注意：由于std::lock_guard的作用域是函数体内部，当frameCallback函数返回时，
    // guard对象会被销毁，自动解锁mutex_，因此不需要手动解锁
    static auto uint_to_float = [](uint16_t x, float xmin, float xmax, uint8_t bits) -> float {
        float span = xmax - xmin;
        float data_norm = static_cast<float>(x) / ((1 << bits) - 1);
        float data = data_norm * span + xmin;
        return data;
    };

    for (const auto& frame_stamp : read_buffer_) {
        can_frame frame = frame_stamp.frame;
        uint16_t canID = (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[0];

        if (read_write_save == true &&
            motors.find(canID) != motors.end()) {  // 这是发送保存参数或者写参数或者读参数返回的数据
            if (frame.data[2] == 0x33 || frame.data[2] == 0x55 ||
                frame.data[2] == 0xAA) {  // 发的是读参数或写参数命令，返回对应寄存器参数
                if (frame.data[2] == 0x33 || frame.data[2] == 0x55) {
                    receive_param(&frame.data[0]);
                }
                read_write_save == false;
            }
        } else {  // 这是正常返回的位置速度力矩数据
            uint16_t q_uint = (static_cast<uint16_t>(frame.data[1]) << 8) | frame.data[2];
            uint16_t dq_uint = (static_cast<uint16_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
            uint16_t tau_uint = (static_cast<uint16_t>(frame.data[4] & 0xf) << 8) | frame.data[5];

            if (motors.find(frame.can_id) == motors.end()) {
                std::cerr << "[Debug] Received frame with ID " << std::hex << frame.can_id << " but not found in map."
                        << std::dec << std::endl;
                return;
            }
            auto m = motors[frame.can_id];
            Limit_param limit_param_receive = m->get_limit_param();
            float receive_q = uint_to_float(q_uint, -limit_param_receive.Q_MAX, limit_param_receive.Q_MAX, 16);
            float receive_dq = uint_to_float(dq_uint, -limit_param_receive.DQ_MAX, limit_param_receive.DQ_MAX, 12);
            float receive_tau = uint_to_float(tau_uint, -limit_param_receive.TAU_MAX, limit_param_receive.TAU_MAX, 12);
            m->receive_data(receive_q, receive_dq, receive_tau);
            // m->frequency = 1. / (frame_stamp.stamp -  m->stamp).toSec();

            // m->stamp = frame_stamp.stamp;
        }
    }
    read_buffer_.clear();
}
}  // namespace damiao
