/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-06-24 16:47:58
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-06-24 17:27:01
 * @FilePath     : \gxnu_hushi_ec\modules\motor\IMotor.hpp
 * @Description  : 电机接口类
 */
#pragma once
#include "bsp/log/log.hpp"
#include "modules/daemon/daemon.hpp"
#include "motor_def.hpp"

//TODO可以用重定义修改电机id,不用到函数里面修改，如下
//define chassis_lf_id = 1;
//define chassis_rf_id = 2;

class IMotor
{
public:
     /////////////////////////////////////////////////////////////////////////加
     // 新增位置控制接口
     void setTargetAngle(float angle) { target_angle_ = angle; }//设置目标角度（外部调用接口）
     //获取当前实际角度（用于调试/反馈）
     float getCurrentAngle() { return current_angle_; }
     
     // 软件限位设置（外部可配置角度范围）
     void setAngleLimit(float min_angle, float max_angle) {
         min_angle_ = min_angle;
         max_angle_ = max_angle;
         enable_limit_ = true;
     }
     void disableAngleLimit() { enable_limit_ = false; }
     ///////////////////////////////////////////////////////////////////////////

    IMotor(MotorType motor_type, MotorPIDSetting setting, MotorPID pidControllers)
        : motor_type_(motor_type), setting_(setting), pidControllers_(pidControllers)
    {
        daemon_ = registerDaemon(10, 10, std::bind(&IMotor::offlineCallback, this));
    }
    void setRef(float ref) { pid_ref_ = ref; };  ///< 设置电机参考值，即速度
    void setFeedback(CloseloopType loop, FeedbackType type, std::shared_ptr<float> feedback_ptr = nullptr)      //设置反馈源
    {
        if (loop == CloseloopType::ANGLE_LOOP)
        {
            if (type == FeedbackType::EXTERNAL)
            {
                if (feedback_ptr == nullptr)
                {
                    LOGERROR("Motor", "Feedback pointer is null");
                    return;
                }
                pidControllers_.pid_angle_feedback_ptr_ = feedback_ptr;
            }
            setting_.external_angle_feedback = type;
        }
        else if (loop == CloseloopType::SPEED_LOOP)
        {
            if (type == FeedbackType::EXTERNAL)
            {
                if (feedback_ptr == nullptr)
                {
                    LOGERROR("Motor", "Feedback pointer is null");
                    return;
                }
                pidControllers_.pid_speed_feedback_ptr_ = feedback_ptr;
            }
            setting_.external_speed_feedback = type;
        }
        else
            LOGERROR("Motor", "Invalid loop type");
    }
    void setOuterloop(CloseloopType loop) { setting_.outer_loop = loop; };  ///< 设置电机外环闭环类型
    void setCloseLoop(CloseloopType loop) { setting_.close_loop = loop; };  ///< 设置电机闭环类型
    void disable() { enable_ = false; };                                    ///< 禁用电机
    void enable() { enable_ = true; };                                      ///< 启用电机

    virtual void offlineCallback() = 0;  ///< 电机离线回调函数, 当电机离线时调用

protected:
    MotorType       motor_type_;      ///< 电机类型
    MotorPIDSetting setting_;         ///< 电机PID设置
    MotorPID        pidControllers_;  ///< 电机PID控制

    bool  enable_      = true;  ///< 电机使能标志, true表示使能, false表示禁用
    float reduce_rate_ = 1.0f;  ///< 电机减速比
    float pid_ref_     = 0;     ///< PID参考值
    float pid_out_     = 0;     ///< PID输出值

    /////////////////////////////////////////////////////////////////////////////////////////添加
    float target_angle_ = 0.0f;       // 目标角度(度)
    float current_angle_ = 0.0f;      // 当前角度(度)
    float min_angle_ = -180.0f;       // 最小限位角度
    float max_angle_ = 180.0f;        // 最大限位角度
    bool enable_limit_ = false;       // 是否限位限位使能
    //////////////////////////////////////////////////////////////////////////////////////////

    DaemonPtr daemon_;            ///< 电机离线检测守护进程
    bool      is_online = false;  ///< 电机在线状态标志, true表示在线, false表示离线
};