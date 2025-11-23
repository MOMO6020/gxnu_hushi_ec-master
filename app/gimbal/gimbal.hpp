#pragma once
#include "app/app_def.hpp"                          //应用定义
#include "bsp/log/log.hpp"                          //日志系统
#include "bsp/system/mutex.hpp"                     //互斥锁
#include "bsp/uart/stm32_uart.hpp"                  //UART驱动
#include "modules/imu/ins_task.h"                   //IMU数据
#include "modules/motor/DJIMotor/DJIMotor.hpp"      //DJI电机
// #include "modules/motor/DMMotor/DMMotor.hpp"        //DM电机
#include "modules/umt/Message.hpp"                  //UMT消息系统
#include "stm32f4xx_hal_uart.h"                     //HAL UART
#include <format>                                   //格式化输出
#include <memory>                                   //智能指针
//云台
// For Test:
UARTHandle_t              uart6 = nullptr;
extern UART_HandleTypeDef huart6;                   //外部声明的HAL UART句柄

class Gimbal                              
{
public:
    Gimbal(attitude_t* imu_data, std::shared_ptr<float> gimbal_yaw_motor_angle_ptr) : imu_data_(imu_data)   //初始化IMU数据指针
    {
        uart6 = STM32UART_Init(&huart6);                //初始化UART6
        if (gimbal_yaw_motor_angle_ptr == nullptr)      //检查yaw角度指针有效性
        {
            while (true)                                //无效指针进入死循环报错
            {
                LOGERROR("Gimbal", "gimbal_yaw_motor_angle_ptr is nullptr");
                STM32TimeDWT::Delay(1000);
            }
        }
        gimbal_yaw_motor_angle_ptr_ = gimbal_yaw_motor_angle_ptr;       //保存yaw角度指针
        extern CAN_HandleTypeDef hcan2;                                 //获取CAN1句柄
        motor_yaw_ = std::make_shared<DJIMotor>(
            &hcan2, 1, MotorType::GM6020,
            MotorPIDSetting{
                CloseloopType::ANGLE_LOOP,
                static_cast<CloseloopType>(CloseloopType::ANGLE_LOOP | CloseloopType::SPEED_LOOP),
                false,
                FeedbackType::EXTERNAL,
                FeedbackType::EXTERNAL,
            },
            MotorPID{
                .pid_angle_ = PIDController(
                    PIDConfig{30.0f, 25.5f, 0.95f, 500.0f, 0.0f, PIDImprovement::PID_Integral_Limit, 200.0f}),
                .pid_speed_ = PIDController(
                    PIDConfig{30.0f, 22.0f, 0.0f, 16384.0f, 0.0f, PIDImprovement::PID_Integral_Limit, 8000.0f}),
                .pid_angle_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->YawTotalAngle),
                .pid_speed_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->Gyro[2]),
            });
//----------创建DM电机(M2006),作为yaw轴----------------//       
        // motor_yaw_ = std::make_shared<DMMotor>(                   
        //     &hcan1, 0x001, 0x000, MotorType::M2006,
        //     MotorPIDSetting{
        //         .outer_loop              = CloseloopType::ANGLE_LOOP,
        //         .close_loop              = CloseloopType::ANGLE_AND_SPEED_LOOP,
        //         .reverse                 = false,
        //         .external_angle_feedback = FeedbackType::EXTERNAL,
        //         .external_speed_feedback = FeedbackType::EXTERNAL,
        //     },
        //     MotorPID{                                           //PID参数
        //         .pid_angle_ = PIDController(
        //             PIDConfig{3.0f, 0.0f, 0.0f, 600.0f, 3.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
        //         .pid_speed_ = PIDController(
        //             PIDConfig{0.01f, 0.0f, 0.0f, 10.0f, 3.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
        //         .pid_angle_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->YawTotalAngle),
        //         .pid_speed_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->Gyro[2]),
        //     });
//----------创建DJI电机作为pitch轴 (GM6020)--------------//
        motor_pitch_ = std::make_shared<DJIMotor>(
            &hcan2, 2, MotorType::GM6020,
            MotorPIDSetting{
                CloseloopType::ANGLE_LOOP,
                static_cast<CloseloopType>(CloseloopType::ANGLE_LOOP | CloseloopType::SPEED_LOOP),
                false,
                FeedbackType::EXTERNAL,
                FeedbackType::EXTERNAL,
            },
            MotorPID{
                .pid_angle_ = PIDController(
                    PIDConfig{28.0f, 22.5f, 0.55f, 500.0f, 0.0f, PIDImprovement::PID_Integral_Limit, 200.0f}),
                .pid_speed_ = PIDController(
                    PIDConfig{30.0f, 22.0f, 0.0f, 16384.0f, 0.0f, PIDImprovement::PID_Integral_Limit, 8000.0f}),
                .pid_angle_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->Pitch),
                .pid_speed_feedback_ptr_ = std::shared_ptr<float>(&imu_data_->Gyro[0]),
            });
//----------绑定UIM消息订阅--------------------------//
        gimbal_cmd_sub_.bind("gimbal_cmd");
    }
//----------任务函数--------------------------------//
    void GimbalTask()
    {
        try
        {
            gimbal_cmd cmd = gimbal_cmd_sub_.pop();       //从消息队列获取云台指令
            if (cmd.force_stop)                      //强制停止处理
            {
                motor_yaw_->disable();
                motor_pitch_->disable();
            }
            else                                    //启用电机
            {
                motor_yaw_->enable();
                motor_pitch_->enable();
                motor_yaw_->setRef(cmd.yaw);
                motor_pitch_->setRef(cmd.pitch);
            }
            // LOGINFO("Gimbal", std::format("Encoder: {:<4}", motor_yaw_->measure_.encoder));
            *gimbal_yaw_motor_angle_ptr_ = motor_yaw_->measure_.angle;      //更新yaw电机角度到共享指针
        }
        catch (umt::MessageError& e)            //如果消息队列为空，则等待1个tick（避免空循环消耗CPU）
        {
            vTaskDelay(1);
        }
    }

private:
    umt::Subscriber<gimbal_cmd> gimbal_cmd_sub_;
    std::shared_ptr<DJIMotor>    motor_yaw_                  = nullptr;
    std::shared_ptr<DJIMotor>   motor_pitch_                = nullptr;
    std::shared_ptr<float>      gimbal_yaw_motor_angle_ptr_ = nullptr;
    attitude_t*                 imu_data_                   = nullptr;
};