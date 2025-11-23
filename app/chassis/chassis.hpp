/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-15 13:34:24
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-22 14:24:25
 * @FilePath     : \FrameworkA_FGJ\app\chassis\chassis.hpp
 * @Description  : 底盘任务
 */
#pragma once
#include "app/app_def.hpp"
#include "bsp/log/log.hpp"
#include "modules/motor/DJIMotor/DJIMotor.hpp"
#include "modules/pid/PIDController.hpp"
#include "modules/umt/Message.hpp"
#include <cmath>
#include <format>
#include <memory>

#define HALF_TRACK_WIDTH (CHASSIS_WHEEL_LATERAL / 2.0f)  ///< 轮距的一半
#define HALF_WHEEL_BASE (CHASSIS_WHEEL_BASE / 2.0f)      ///< 轴距的一半
#define DEGREE_TO_RADIAN (M_PI / 180.0f)                 //定义弧度

class Chassis
{
public:
    Chassis()
    {
        extern CAN_HandleTypeDef hcan1;         //声明外部CAN句柄，用于电机通信
//----------创建左前电机对象-----------------------// 
        motor_lf = std::make_shared<DJIMotor>(
            &hcan1, 1, MotorType::M3508,
            MotorPIDSetting{
                CloseloopType::SPEED_LOOP,      //外环为速度环
                CloseloopType::SPEED_LOOP,      //内环为速度环（但这里可能应该是电流环，但配置为速度环，注意：通常M3508使用速度环控制，内环由电机内部处理）
                false,                          //不使用前馈，前馈未写
                FeedbackType::INTERNAL,         //反馈使用内部反馈（电机反馈）
                FeedbackType::INTERNAL,         //反馈二（未使用）
            },
            //具体PID参数(kp,ki,kd,输出限幅（对应电流值，最大16384对应20A),输入死区,使用积分限幅,变速积分)
            MotorPID{
                .pid_speed_ = PIDController(
                    PIDConfig{4.5f, 0.0f, 0.0f, 16384.0f, 10.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
            });
//----------创建右前电机---------------------------//
        motor_rf = std::make_shared<DJIMotor>(
            &hcan1, 2, MotorType::M3508,
            MotorPIDSetting{
                CloseloopType::SPEED_LOOP,
                CloseloopType::SPEED_LOOP,
                false,
                FeedbackType::INTERNAL,
                FeedbackType::INTERNAL,
            },
            MotorPID{
                .pid_speed_ = PIDController(
                    PIDConfig{4.5f, 0.0f, 0.0f, 16384.0f, 10.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
            });
//----------创建左后电机---------------------------//          
        motor_lb = std::make_shared<DJIMotor>(
            &hcan1, 3, MotorType::M3508,
            MotorPIDSetting{
                CloseloopType::SPEED_LOOP,
                CloseloopType::SPEED_LOOP,
                false,
                FeedbackType::INTERNAL,
                FeedbackType::INTERNAL,
            },
            MotorPID{
                .pid_speed_ = PIDController(
                    PIDConfig{4.5f, 0.0f, 0.0f, 16384.0f, 10.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
            });
//----------创建右后电机---------------------------//
        motor_rb = std::make_shared<DJIMotor>(
            &hcan1, 4, MotorType::M3508,
            MotorPIDSetting{
                CloseloopType::SPEED_LOOP,
                CloseloopType::SPEED_LOOP,
                false,
                FeedbackType::INTERNAL,
                FeedbackType::INTERNAL,
            },
            MotorPID{
                .pid_speed_ = PIDController(
                    PIDConfig{4.5f, 0.0f, 0.0f, 16384.0f, 10.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
            });
//----------订阅名为"chassis_cmd"的消息---------------------------//
        chassis_cmd_sub_.bind("chassis_cmd");
    }
//----------底盘任务函数-----------------------------------------//
    void ChassisTask()
    {
        chassis_cmd cmd;        //创建一个底盘命令对象
        try
        {
            cmd = chassis_cmd_sub_.pop();        //从消息队列中取出最新的底盘命令（非阻塞，如果队列为空则抛出异常）
            if (cmd.mode == chassis_mode::CHASSIS_STOP)     //如果模式是停止，则禁用所有电机
            {
                motor_lf->disable();
                motor_rf->disable();
                motor_lb->disable();
                motor_rb->disable();
            }
            else                                            //否则使能所有电机                  
            {
                motor_lf->enable();
                motor_rf->enable();
                motor_lb->enable();
                motor_rb->enable();
            }

            switch (cmd.mode)                               //根据模式设置底盘旋转速度
            {
                case chassis_mode::CHASSIS_NO_FOLLOW: chassis_vr = 0; break;    //无跟随，旋转速度为0
                case chassis_mode::CHASSIS_ROTATE: chassis_vr = 100; break;     //旋转模式，设定一个固定旋转速度（100，单位可能是度/秒或弧度/秒？）
                case chassis_mode::CHASSIS_FOLLOW_GIMBAL:                       // 跟随云台模式：旋转速度与偏移角度相关（非线性）
                    chassis_vr = 0.5 * cmd.offset_angle * std::abs(cmd.offset_angle);  
                default: break;
            }
//计算偏移角度的正弦和余弦（用于将云台坐标系的命令转换到底盘坐标系）
//注意:偏移角度cmd.offset_angle是云台相对于底盘的角度（单位：度），转换为弧度
            sin_theta  = std::sin(cmd.offset_angle * DEGREE_TO_RADIAN);
            cos_theta  = std::cos(cmd.offset_angle * DEGREE_TO_RADIAN);
            chassis_vx = cmd.vx * cos_theta - cmd.vy * sin_theta;     //旋转矩阵：[cos, -sin; sin, cos] * [vx; vy]
            chassis_vy = cmd.vx * sin_theta + cmd.vy * cos_theta;

            // MecanumCalculate();
            OmniCalculate();  // TODO: 需要根据底盘类型选择计算方式,此处选择全向轮

            // TODO: 输出限制，实际得求出最大速度，在PID输出添加
            //设置四个电机的目标速度
            motor_lf->setRef(vt_lf);
            motor_rf->setRef(vt_rf);
            motor_lb->setRef(vt_lb);
            motor_rb->setRef(vt_rb);
        }
        catch (umt::MessageError& e)        //如果消息队列为空，则等待1个tick（避免空循环消耗CPU）
        {
            vTaskDelay(1);
        }
    }

private:
    umt::Subscriber<chassis_cmd> chassis_cmd_sub_;      // UMT消息订阅器
    std::shared_ptr<DJIMotor>    motor_lf = nullptr;
    std::shared_ptr<DJIMotor>    motor_rf = nullptr;
    std::shared_ptr<DJIMotor>    motor_lb = nullptr;
    std::shared_ptr<DJIMotor>    motor_rb = nullptr;    //电机对象
//临时变量计算
    float sin_theta  = 0;              
    float cos_theta  = 0;          //云台三角函数
    float chassis_vr = 0;          //底盘旋转速度
    float chassis_vx = 0;
    float chassis_vy = 0;          //底盘坐标系线速度
    float vt_lf      = 0;          
    float vt_rf      = 0;
    float vt_lb      = 0;
    float vt_rb      = 0;          //电机目标速度
//旋转中心计算（含云台偏移补偿）(左下、右下、左上、右上)
#define LF_CENTER                                                                                                            \
    ((HALF_TRACK_WIDTH + CHASSIS_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CHASSIS_GIMBAL_OFFSET_Y) * DEGREE_TO_RADIAN)
#define RF_CENTER                                                                                                     \
    ((HALF_TRACK_WIDTH - CHASSIS_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CHASSIS_GIMBAL_OFFSET_Y) * DEGREE_TO_RADIAN)
#define LB_CENTER                                                                                                      \
    ((HALF_TRACK_WIDTH + CHASSIS_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CHASSIS_GIMBAL_OFFSET_Y) * DEGREE_TO_RADIAN)
#define RB_CENTER                                                                                                      \
    ((HALF_TRACK_WIDTH - CHASSIS_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CHASSIS_GIMBAL_OFFSET_Y) * DEGREE_TO_RADIAN)
//麦轮运动学分解
    void MecanumCalculate()
    {
        vt_lf = chassis_vx + chassis_vy - chassis_vr * LF_CENTER;
        vt_rf = chassis_vx - chassis_vy - chassis_vr * RF_CENTER;
        vt_lb = -chassis_vx + chassis_vy - chassis_vr * LB_CENTER;
        vt_rb = -chassis_vx - chassis_vy - chassis_vr * RB_CENTER;
    }
//全向轮运动学分解
    void OmniCalculate()
    {
        vt_lf = chassis_vx + chassis_vy + chassis_vr * LF_CENTER;
        vt_rf = chassis_vx - chassis_vy + chassis_vr * RF_CENTER;
        vt_lb = -chassis_vx - chassis_vy + chassis_vr * LB_CENTER;
        vt_rb = -chassis_vx + chassis_vy + chassis_vr * RB_CENTER;
    }
};