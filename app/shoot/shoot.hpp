#pragma once
#include "app/app_def.hpp"
#include "bsp/log/log.hpp"
#include "modules/motor/DJIMotor/DJIMotor.hpp"
#include "modules/motor/DMMotor/DMMotor.hpp"       
#include "modules/pid/PIDController.hpp"
#include "modules/umt/Message.hpp"
#include <cmath>
#include <format>
#include <memory>



class Shoot
{
public:
    Shoot()
    {
        // extern CAN_HandleTypeDef hcan1;         //声明外部CAN句柄，用于电机通信
        extern CAN_HandleTypeDef hcan1;         //声明外部CAN句柄，用于电机通信
//----------创建左摩擦轮电机对象-----------------------// 
        motor_l_rub = std::make_shared<DJIMotor>(
            &hcan1, 5, MotorType::M2006,
            MotorPIDSetting{
                CloseloopType::SPEED_LOOP,      //外环为速度环
                CloseloopType::SPEED_LOOP,      //内环为速度环（但这里可能应该是电流环，但配置为速度环，注意：通常M3508使用速度环控制，内环由电机内部处理）
                false,                          //不使用前馈，前馈未写
                FeedbackType::INTERNAL,         //反馈使用内部反馈（电机反馈）
                FeedbackType::INTERNAL,         //反馈二（未使用）
            },
            //具体PID参数(kp,ki,kd,输出限幅（对应电流值，最大16384对应20A),积分分离阈值(当误差大于10时，不进行积分),使用积分限幅,积分限幅值)
            MotorPID{
                .pid_speed_ = PIDController(
                    PIDConfig{4.5f, 0.0f, 0.0f, 16384.0f, 10.0f, PIDImprovement::PID_Integral_Limit, 4000.0f}),
            });
//----------创建右摩擦轮电机---------------------------//
        motor_r_rub = std::make_shared<DJIMotor>(
            &hcan1, 6, MotorType::M2006,
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
//----------创建拨弹电机---------------------------//          
        motor_pb = std::make_shared<DJIMotor>(
            &hcan1, 7 , MotorType::M2006,
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
//---------发射任务消息--------------------------//
       shoot_cmd_sub_.bind("shoot_cmd");
    }

//----------底盘任务函数-----------------------------------------//
    void ShootTask()
    {
        shoot_cmd cmd;        //创建一个发射命令对象
        try
        {    cmd = shoot_cmd_sub_.pop(); 
            if (cmd.mode == shoot_mode::SHOOT_OFF)     //如果模式是停止，则禁用所有电机
            {
                motor_l_rub->disable();
                motor_r_rub->disable();
                motor_pb->disable();
            }
            else                      //使能所有电机  
                {
                motor_l_rub->enable();
                motor_r_rub->enable();
                motor_pb->enable();
                }
            // {
            //     motor_l_rub->enable();
            //     motor_r_rub->enable();
            //     motor_pb->enable();
            //   else if (cmd.mode == shoot_mode::FRICTION_ON)     //使能摩擦轮电机                  
            // {
            //     motor_l_rub->enable();
            //     motor_r_rub->enable();
            //     motor_pb->disable();

            // }
            //   else  if (cmd.mode == shoot_mode::SHOOT_PB_ON)     //使能拨弹电机                  
            // {
            //     motor_l_rub->disable();
            //     motor_r_rub->disable();
            //     motor_pb->enable();

            // }
            // else  if(cmd.mode == shoot_mode::SHOOT_ON)           //使能所有电机                  
            // {
            //     motor_l_rub->enable();
            //     motor_r_rub->enable();
            //     motor_pb->enable();

            // }
           //TODO射击模式打开->单独开摩擦轮、单独开拨弹电机->单发、三发、连发模式选择
           //TODO卡弹自动关闭射击模式可通过积分累计进行
           //TODO后续将拨弹轮改为定位置控制实现单发、三发和连发
            switch (cmd.mode)                               //根据模式设置摩擦轮和拨弹轮速度，
            {
                case shoot_mode::SHOOT_ON    : {shoot_v_rub = 0;shoot_v_pb=0;} break;
                case shoot_mode::FRICTION_ON :  shoot_v_rub =  cmd.v_rub_max;  break;         //开摩擦轮
                case shoot_mode::SHOOT_PB_ON :  shoot_v_pb  =  cmd.v_pb_max;   break;         //开拨弹电机,按遥控器逐渐增加速度//TODO未修改
                case shoot_mode::LOAD_REVERSE : shoot_v_pb  =  cmd.v_rub_opposite;   break;   //拨弹反转
                default: break;
            }
            //设置速度
            motor_l_rub->setRef(shoot_v_rub);       //左右摩擦轮
            motor_r_rub->setRef(-shoot_v_rub);      //摩擦轮转动方向应该相反
            motor_pb->setRef(shoot_v_pb);           //拨弹轮
        }
        catch (umt::MessageError& e)        //如果消息队列为空，则等待1个tick（避免空循环消耗CPU）
        {
            vTaskDelay(1);
        }
    }

private:
    umt::Subscriber<shoot_cmd>   shoot_cmd_sub_;      // UMT消息订阅器
    std::shared_ptr<DJIMotor>    motor_l_rub = nullptr;
    std::shared_ptr<DJIMotor>    motor_r_rub = nullptr;
    std::shared_ptr<DJIMotor>    motor_pb = nullptr;      //电机对象   
//临时变量计算
    float shoot_v_rub = 0;          //摩擦轮速度
    float shoot_v_pb = 0;          //拨弹电机速度

};