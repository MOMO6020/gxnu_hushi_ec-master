/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-11 21:11:16
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-11 23:18:06
 * @FilePath     : \FrameworkA_FGJ\app\robotcmd\robotcmd.hpp
 * @Description  : 机器人控制中心, 所有的控制命令都在这里处理,
 * 并将控制命令分发到各个模块
 */
#pragma once
#include "app/app_def.hpp"       //应用层定义
#include <cstdint>               
#include <cstdlib>               //基础工具库
#include <format>                //C++20格式化库
#include <main.h>
#include <memory>                //智能指针

#include "bsp/log/log.hpp"       //系统日志等
#include "bsp/system/mutex.hpp"
#include "bsp/system/time.hpp"

#include "modules/controller/controller.hpp"    //遥控器
#include "modules/imu/ins_task.h"
#include "modules/umt/umt.hpp"                  //线程通信
#include "modules/vision/vision.hpp"            //视觉
#include "usart.h"                //串口驱动

#define YAW_ALIGEN_ANGLE (YAW_CHASSIS_ALIGEN_ENCODER * 360.0f / 8192.0f)  ///< 云台与地盘对其时电机编码器值对应的角度
#define PITCH_HORIZON_ANGLE (PITCH_HORIZON_ENCODER * 360.0f / 8192.0f)    ///< 云台水平时电机编码器值对应的角度

class RobotCMD
{
public:
    RobotCMD(attitude_t* imu_data, std::shared_ptr<float> gimbal_yaw_motor_angle_ptr)
        : _gimbal_yaw_motor_angle_ptr(gimbal_yaw_motor_angle_ptr), _controller(&huart3), imu_data_(imu_data)
    {
        if (_gimbal_yaw_motor_angle_ptr == nullptr)         //校验是否为空指针
        {
            while (true)
            {
                LOGERROR("RobotCMD", "gimbal_yaw_motor_angle_ptr_ is nullptr");
                STM32TimeDWT::Delay(1000);
            }
        }
        _vision = std::make_shared<VisionCommand>(&huart1);     //初始化视觉模块
        _gimbal_cmd_pub.bind("gimbal_cmd");                     //云台命令报文
        _chassis_cmd_pub.bind("chassis_cmd");                   //底盘命令报文
        _shoot_cmd_pub.bind("shoot_cmd");                       //发射命令报文        
    }

    void RobotCMDTask()                 //控制任务
    {
        if (_controller.is_connected)
        {
            if (_controller.rc_data.switch1 == 1)       // 遥控器左侧开关[上], 遥控器控制
                rcControlSet();
            else if (_controller.rc_data.switch1 == 3)  // 遥控器左侧开关[中], 键盘控制
                mouseKeySet();
            else if (_controller.rc_data.switch1 == 2)  // 遥控器左侧开关[下], 视觉控制
                visionSet();
            //计算偏移角度并发布命令
            chassis_cmd_msg.offset_angle = calculateOffsetAngle();
            _gimbal_cmd_pub.push(gimbal_cmd_msg);
            _chassis_cmd_pub.push(chassis_cmd_msg);
            _shoot_cmd_pub.push(shoot_cmd_msg);
        }
        else
        {
            __asm volatile("nop");      //空操作指令
        }
    }

private:
    umt::Publisher<gimbal_cmd>     _gimbal_cmd_pub;                         //云台命令发布
    umt::Publisher<chassis_cmd>    _chassis_cmd_pub;                        //底盘命令发布
    umt::Publisher<shoot_cmd>     _shoot_cmd_pub;                            //射击命令发布

    std::shared_ptr<float>         _gimbal_yaw_motor_angle_ptr = nullptr;   //云台yaw电机角度，真实角度，（量化后返回值时相对角度）
    Controller                     _controller;                             //遥控器对象
    std::shared_ptr<VisionCommand> _vision   = nullptr;                     //视觉模块
    attitude_t*                    imu_data_ = nullptr;                     //IMU数据

    gimbal_cmd  gimbal_cmd_msg;                                             //云台命令缓存
    chassis_cmd chassis_cmd_msg;                                            //底盘命令缓存
    shoot_cmd shoot_cmd_msg;                                             //发射命令缓存

    float calculateOffsetAngle()                  //偏移角计算，过零处理，此处‘YAW_CHASSIS_ALIGEN_THAN_4096’为1，为0则执行else
    {
#if YAW_CHASSIS_ALIGEN_THAN_4096                    
        if (*_gimbal_yaw_motor_angle_ptr > YAW_ALIGEN_ANGLE &&
            *_gimbal_yaw_motor_angle_ptr <= (YAW_ALIGEN_ANGLE + 180.0f))        //180~360之间，-180
            return *_gimbal_yaw_motor_angle_ptr - YAW_ALIGEN_ANGLE;
        else if (*_gimbal_yaw_motor_angle_ptr > (YAW_ALIGEN_ANGLE + 180.0f))
            return *_gimbal_yaw_motor_angle_ptr - YAW_ALIGEN_ANGLE - 360.0f;    //大于360.-360
        else                                                                    //0~180之间
            return *_gimbal_yaw_motor_angle_ptr - YAW_ALIGEN_ANGLE;             //将初始值置为0度，将角度范围量化为-180~+180
#else                                                                           
        if (*_gimbal_yaw_motor_angle_ptr > YAW_ALIGEN_ANGLE)                    //360-机械零点<360，未到零点
            return *_gimbal_yaw_motor_angle_ptr - YAW_ALIGEN_ANGLE;             //小于180，直接输出返回值，相对角度                
        else if (*_gimbal_yaw_motor_angle_ptr <= YAW_ALIGEN_ANGLE &&
                 *_gimbal_yaw_motor_angle_ptr >= (YAW_ALIGEN_ANGLE - 180.0f))   //-180~机械零点之间，输出相对角度
            return *_gimbal_yaw_motor_angle_ptr - YAW_ALIGEN_ANGLE;
        else                                                                    //<-360,输出减去360.
            return *_gimbal_yaw_motor_angle_ptr - YAW_ALIGEN_ANGLE + 360.0f;
#endif
    }

//通过遥控器的左侧拨杆选择操作模式 -> 上中下 -> 遥控器，键鼠 ，视觉
/*-------------------------遥控器控制 -> 选择底盘运行模式-----------------------------------*/ 
    void rcControlSet()                     
    {
        if (_controller.rc_data.switch2 == 1)  // 遥控器右侧开关[上]            //普通模式
        {
            chassis_cmd_msg.mode      = chassis_mode::CHASSIS_NO_FOLLOW;        //底盘模式选择
            gimbal_cmd_msg.force_stop = false;                                  //停止标志位失能
            gimbal_cmd_msg.yaw        = gimbal_cmd_msg.yaw - _controller.rc_data.channel2 * 0.0005f;        //左方X轴拨杆控制云台yaw轴
            gimbal_cmd_msg.pitch      = gimbal_cmd_msg.pitch + _controller.rc_data.channel3 * 0.0005f;      //左方Y轴拨杆控制云台pitch轴
            shoot_cmd_msg.mode        = shoot_mode::SHOOT_OFF;              //射击模式关闭
        }
        else if (_controller.rc_data.switch2 == 3)  // 遥控器右侧开关[中]       //底盘停止模式
        {
            chassis_cmd_msg.mode      = chassis_mode::CHASSIS_STOP;
            gimbal_cmd_msg.force_stop = true;
            /*==================射击=====================*/
            shoot_cmd_msg.mode        = shoot_mode::SHOOT_ON;                  //射击模式打开，只使能，不给速度
            if(_controller.rc_data.channel1>50 && _controller.rc_data.channel1<550)         //开摩擦轮
            {shoot_cmd_msg.mode       =shoot_mode::FRICTION_ON;
            /*==================*/
            if(_controller.rc_data.channel0>50 && _controller.rc_data.channel0<550)         //开拨弹轮
            {shoot_cmd_msg.mode    =shoot_mode::SHOOT_PB_ON;}
            else if(_controller.rc_data.channel0<-50 && _controller.rc_data.channel0>-550)  //反转拨弹轮
            {shoot_cmd_msg.mode    =shoot_mode::LOAD_REVERSE;}
            }
            else                                                                            //关拨弹
            {shoot_cmd_msg.mode       =shoot_mode::SHOOT_ON;}
            /*=============================================*/
        }
        else if (_controller.rc_data.switch2 == 2)  // 遥控器右侧开关[下]       //小陀螺模式，可以通过拨杆控制加速减速 
        {
            chassis_cmd_msg.mode      = chassis_mode::CHASSIS_ROTATE;
            gimbal_cmd_msg.force_stop = false;
            gimbal_cmd_msg.yaw        = gimbal_cmd_msg.yaw - _controller.rc_data.channel2 * 0.0005f;        //同上
            gimbal_cmd_msg.pitch      = gimbal_cmd_msg.pitch + _controller.rc_data.channel3 * 0.0005f;
            shoot_cmd_msg.mode        = shoot_mode::SHOOT_OFF;                  //射击模式关闭
        }
        chassis_cmd_msg.vx = _controller.rc_data.channel0 * 8.0f;           //右拨杆X轴控制底盘左右
        chassis_cmd_msg.vy = _controller.rc_data.channel1 * 8.0f;           //右拨杆Y轴控制底盘前后

        //TODO遥控器操控发射任务
    }
/*-------------------------键盘控制---------------------------------------------------------------*/ 
    // void mouseKeySet() { __asm volatile("nop"); }           //不执行操作
    void mouseKeySet()                                      //使用键鼠，初始默认电机全部使能
    { 
        // Controller::Key keydata;                                    //获取键盘按下信息
        int rotateflag = 0;                                         //开启陀螺仪标志位
        int chassis_a  = 200;                                       //底盘x，y轴初始速度
        int chassis_x  = 0;
        int chassis_y  = 0;
        /*-------------=========开关小陀螺模式下云台控制=========-------------*/
        if((_controller.mouse_data.keyboard == Controller::Key::KEY_Q) && (rotateflag == 0))                  //Q开小陀螺
        {   rotateflag                = 1;
            chassis_cmd_msg.mode      = chassis_mode::CHASSIS_ROTATE;
            gimbal_cmd_msg.force_stop = false;
            gimbal_cmd_msg.yaw        = gimbal_cmd_msg.yaw   - _controller.mouse_data.x * 0.0005f;      //鼠标X轴控制yaw轴
            gimbal_cmd_msg.pitch      = gimbal_cmd_msg.pitch + _controller.mouse_data.y * 0.0005f;      //鼠标Y轴控制pitch轴
        }
        if((_controller.mouse_data.keyboard == Controller::Key::KEY_E) && (rotateflag == 1))                  //E关小陀螺
        {   rotateflag                = 0;
            chassis_cmd_msg.mode      = chassis_mode::CHASSIS_NO_FOLLOW;              //底盘模式选择
            gimbal_cmd_msg.force_stop = false;                                        //云台停止标志位失能
            gimbal_cmd_msg.yaw        = gimbal_cmd_msg.yaw   - _controller.mouse_data.x * 0.0005f;       //鼠标X轴控制yaw轴
            gimbal_cmd_msg.pitch      = gimbal_cmd_msg.pitch + _controller.mouse_data.y * 0.0005f;       //鼠标Y轴控制pitch轴
        }
        /*-------------------==========射击任务=========------------------*/
        if(_controller.mouse_data.keyboard == Controller::Key::KEY_F)    
        {
            if(_controller.mouse_data.left_button == 1)
            {
            // shoot_cmd_msg.mode        = shoot_mode::SHOOT_ON;                  //射击模式开
            }
        }
        if(_controller.mouse_data.keyboard == Controller::Key::KEY_E)
        {
            if(_controller.mouse_data.left_button == 1)
            {
            // shoot_cmd_msg.mode        = shoot_mode::SHOOT_ON;                  //射击模式关
            }
        }
        /*-------------------==========底盘运动=========-------------------*/
        //后续可以添加时间戳来逐渐加速(V++)，防止初始速度过大超功率
        if((_controller.mouse_data.keyboard == Controller::Key::KEY_A) || (_controller.mouse_data.keyboard == Controller::Key::KEY_D) || (_controller.mouse_data.keyboard == Controller::Key::KEY_W)|| (_controller.mouse_data.keyboard == Controller::Key::KEY_S))     //判断按键是否按下
        {
            chassis_x = ((_controller.mouse_data.keyboard & Controller::Key::KEY_A)*chassis_a) - ((_controller.mouse_data.keyboard & Controller::Key::KEY_D)*chassis_a);    //将键盘数据按位取出
            chassis_y = ((_controller.mouse_data.keyboard & Controller::Key::KEY_W)*chassis_a) - ((_controller.mouse_data.keyboard & Controller::Key::KEY_S)*chassis_a);
        
            chassis_cmd_msg.vx = chassis_x * 8.0f;           //键盘AD控制底盘左右逐渐加速
            chassis_cmd_msg.vy = chassis_y * 8.0f;           //键盘WS控制底盘前后逐渐加速
        }
    }      
/*-------------------------视觉控制--------------------------------------------------------------*/ 
    void visionSet()                            
    {
        VisionCommand::RobotCmd visionCmd;
        static uint32_t         fps       = 0;
        static uint32_t         fps_count = 0;
        static uint32_t         time_cnt  = STM32TimeDWT::GetMilliseconds();        //微妙时间戳

        if (_controller.rc_data.switch2 == 3)  // 遥控器右侧开关[中]        //底盘停止模式
        {
            chassis_cmd_msg.mode      = chassis_mode::CHASSIS_STOP;
            gimbal_cmd_msg.force_stop = true;
        }
        else
        {
            gimbal_cmd_msg.yaw   = gimbal_cmd_msg.yaw - _controller.rc_data.channel2 * 0.0005f;
            gimbal_cmd_msg.pitch = gimbal_cmd_msg.pitch + _controller.rc_data.channel3 * 0.0005f;

            chassis_cmd_msg.mode =
                _controller.rc_data.switch2 == 2 ? chassis_mode::CHASSIS_ROTATE : chassis_mode::CHASSIS_NO_FOLLOW;
            gimbal_cmd_msg.force_stop = false;          //选择小陀螺模式或者通常模式

            if (xQueueReceive(_vision->queue, &visionCmd, 0) == pdTRUE)
            {
                if (visionCmd.shoot_mode != 0)
                {
                    gimbal_cmd_msg.yaw   = imu_data_->YawTotalAngle - visionCmd.yaw_angle;
                    gimbal_cmd_msg.pitch = imu_data_->Pitch + visionCmd.pitch_angle;
                    chassis_cmd_msg.vx   = _controller.rc_data.channel0 * 8.0f;
                    chassis_cmd_msg.vy   = _controller.rc_data.channel1 * 8.0f;
                    float yaw_vision     = visionCmd.yaw_angle;
                    taskENTER_CRITICAL();
                    LOGINFO("Vision", std::format("Y:{}, Yi:{}, Yv:{}, FPS:{}", gimbal_cmd_msg.yaw,
                                                  imu_data_->YawTotalAngle, yaw_vision, fps));
                    taskEXIT_CRITICAL();
                    fps_count++;
                }
            }
            if (STM32TimeDWT::GetMilliseconds() - time_cnt > 1000)
            {
                fps       = fps_count;
                fps_count = 0;
                time_cnt  = STM32TimeDWT::GetMilliseconds();
            }
        }
    }
};