/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-07 10:59:49
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-06-24 17:21:03
 * @FilePath     : \gxnu_hushi_ec\modules\motor\DJIMotor\DJIMotor.hpp
 * @Description  : 大疆电机驱动
 */
#pragma once
#include "../motor_def.hpp"
#include <array>

#include "bsp/can/stm32_can.hpp"
#include "modules/daemon/daemon.hpp"
#include "modules/motor/IMotor.hpp"

/**
 * @class DJIMotor
 * @brief 大疆电机类, 仅计算电机输出电流, 不控制电机, 使用MotorController类控制电机
 * @todo 完善注释, 添加PID前馈控制
 */
class DJIMotor : public IMotor
{
public:
    struct Measure
    {
        uint16_t last_encoder   = 0;  ///< 上次编码器值(0-8191)
        uint16_t encoder        = 0;  ///< 编码器值(0-8191)
        int16_t  speed          = 0;  ///< 电机转速(RPM)
        int16_t  torque_current = 0;  ///< 电机电流
        uint8_t  temperature    = 0;  ///< 电机温度
        float    speed_dps      = 0;  ///< 电机转速(degree/s)
        float    angle          = 0;  ///< 电机单圈角度(degree)
        float    total_angle    = 0;  ///< 总转动角度(degree)

        int32_t total_round = 0;  ///< 电机转动圈数
    };

    DJIMotor(CAN_HandleTypeDef* _hcan,  
             uint16_t           _motor_id,
             MotorType          _motor_type,
             MotorPIDSetting    _setting,
             MotorPID           _pid_config)
             : hcan_(_hcan), motor_id_(_motor_id), motor_type_(_motor_type), setting_(_setting), pid_config_(_pid_config) {
            // 原有构造函数逻辑（注册电机、设置发送组等）
            if (motor_count_ < MAX_DJIMOTOR_COUNT) {
                registered_motors_[motor_count_++] = this;
            }  
            // 初始化PID误差成员（避免首次计算异常）
            pid_config_.pid_angle_.err = 0.0f;
            pid_config_.pid_angle_.err_last = 0.0f;
            pid_config_.pid_angle_.err_sum = 0.0f;
            pid_config_.pid_speed_.err = 0.0f;
            pid_config_.pid_speed_.err_last = 0.0f;
            pid_config_.pid_speed_.err_sum = 0.0f;
    }  // 构造函数闭合大括号（已补充）

    void                  decode(const uint8_t* buf, const uint8_t len);
    void                  offlineCallback();
    // 只保留public声明：无实现，分号结尾
    int16_t calculateOutputCurrent(int16_t target_speed_rpm);  
    std::function<void()> calculateCallback = nullptr;  // 测试用回调函数

    static void DJIMotorControl();

    Measure measure_;  ///< 电机测量数据

    // 新增：核心控制函数（位置控制+限位）
    void motorControl() {
        if (!enable_) return;  // 电机未使能，直接返回
        
        // 步骤1：软件限位处理
        processAngleLimit();
        
        // 步骤2：位置环PID计算（目标角度→速度指令）
        float target_speed_dps = calculatePosPID();
        
        // 步骤3：速度单位转换（dps→RPM）
        int16_t target_speed_rpm = static_cast<int16_t>(target_speed_dps / 6.0f);
        
        // 步骤4：调用速度环PID计算电流
        int16_t output_current = calculateOutputCurrent(target_speed_rpm);
        
        // 步骤5：CAN发送电流指令
        writeControlData(output_current);
    }

    // 新增：写入CAN控制帧
    void writeControlData(int16_t current) {
        uint8_t idx = 0;
        if (hcan_ == &hcan1) {
            idx = (motor_type_ == MotorType::GM6020) ? 2 : 0;
        } else if (hcan_ == &hcan2) {
            idx = (motor_type_ == MotorType::GM6020) ? 6 : 4;
        }
        idx += (motor_id_ - 1) / 4;
        
        uint8_t offset = ((motor_id_ - 1) % 4) * 2;
        if (motor_type_ == MotorType::GM6020) {
            control_data_[idx].data[offset] = (current >> 8) & 0xFF;
            control_data_[idx].data[offset + 1] = current & 0xFF;
        } else {
            control_data_[idx].data[offset] = (current >> 8) & 0xFF;
            control_data_[idx].data[offset + 1] = current & 0xFF;
        }
    }

private:
    CAN_HandleTypeDef* hcan_;
    uint8_t            motor_id_;                    ///< 电机ID
    uint8_t            motor_tx_group_ = 0;          ///< 电机发送组ID
    inline static std::array<DJIMotor*, MAX_DJIMOTOR_COUNT> registered_motors_{nullptr};  ///< 电机数组
    inline static uint8_t                                   motor_count_ = 0;             ///< 电机数量
    
    // 位置环PID计算函数
    float calculatePosPID() {
        // 误差计算（目标角度-当前角度）
        float angle_err = target_angle_ - measure_.total_angle;
        
        // 处理角度环绕（360°闭环）
        if (angle_err > 180.0f) angle_err -= 360.0f;
        if (angle_err < -180.0f) angle_err += 360.0f;
        
        // PID计算
        float pid_out = pid_config_.pid_angle_.kp * angle_err 
                      + pid_config_.pid_angle_.ki * pid_config_.pid_angle_.err_sum 
                      + pid_config_.pid_angle_.kd * (angle_err - pid_config_.pid_angle_.err_last);
        
        // 速度限幅
        float max_speed_dps = 360.0f;
        if (pid_out > max_speed_dps) pid_out = max_speed_dps;
        if (pid_out < -max_speed_dps) pid_out = -max_speed_dps;
        
        // 更新PID误差历史
        pid_config_.pid_angle_.err_last = angle_err;
        pid_config_.pid_angle_.err_sum += angle_err;
        
        // 积分限幅（避免除零）
        float max_iout = 100.0f;
        if (pid_config_.pid_angle_.ki != 0) {
            if (pid_config_.pid_angle_.err_sum > max_iout / pid_config_.pid_angle_.ki) {
                pid_config_.pid_angle_.err_sum = max_iout / pid_config_.pid_angle_.ki;
            } else if (pid_config_.pid_angle_.err_sum < -max_iout / pid_config_.pid_angle_.ki) {
                pid_config_.pid_angle_.err_sum = -max_iout / pid_config_.pid_angle_.ki;
            }
        } else {
            pid_config_.pid_angle_.err_sum = 0;
        }
        
        return pid_out;
    }

    // 软件限位处理函数
    void processAngleLimit() {
        if (!enable_limit_) return;
        
        if (target_angle_ < min_angle_) {
            target_angle_ = min_angle_;
            printf("DJIMotor ID%d: 超出下限！目标=%.1f°，允许范围=%.1f~%.1f°\n", 
                   motor_id_, target_angle_, min_angle_, max_angle_);
        } else if (target_angle_ > max_angle_) {
            target_angle_ = max_angle_;
            printf("DJIMotor ID%d: 超出上限！目标=%.1f°，允许范围=%.1f~%.1f°\n", 
                   motor_id_, target_angle_, min_angle_, max_angle_);
        }
    }

    /**
     * @brief 存储DJI电机控制数据的静态数组
     */
    inline static std::array<ICAN::ClassicPacket, 8> control_data_{
        // CAN1:
        ICAN::ClassicPacket{0x200, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x1FF, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x1FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x2FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        // CAN2:
        ICAN::ClassicPacket{0x200, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x1FF, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x1FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
        ICAN::ClassicPacket{0x2FE, ICAN::Type::STANDARD, {0, 0, 0, 0, 0, 0, 0, 0}},
    };  
};  