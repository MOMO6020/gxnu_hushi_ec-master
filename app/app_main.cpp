#include "app_main.h"                   //主应用头文件
#include "app/chassis/chassis.hpp"      //底盘控制
#include "app/robotcmd/robotcmd.hpp"    //机器人命令
#include "bsp/system/mutex.hpp"         //互斥锁
#include "cmsis_os.h"                   //CMSIS-RTOS API

#include "bsp/log/log.hpp"              //日志系统
#include "bsp/system/time.hpp"          //时间相关
#include "bsp/serial/serial.h"          // 假设你有串口驱动

#include "gimbal/gimbal.hpp"            //云台控制
#include "modules/imu/ins_task.h"       //IMU任务
#include "modules/motor/motor.hpp"      //电机模块
#include <memory>                       //智能指针
//!<未测试
#include "shoot/shoot.hpp"              //发射结构      添加时间2025/07/07

#include "modules/motor/DJIMotor/DJIMotor.hpp"
#include "modules/motor/DMMotor/DMMotor.hpp"


// std::shared_ptr<float>    gimbal_yaw_motor_angle_ptr = nullptr;     //云台yaw轴共享指针（空指针）
// std::shared_ptr<RobotCMD> robotcmd                   = nullptr;     //机器人命令对象
// std::shared_ptr<Chassis>  chassis                    = nullptr;     //底盘对象
// std::shared_ptr<Gimbal>   gimbal                     = nullptr;     //云台对象
// attitude_t*               imu_data                   = nullptr;     //IMU数据指针

// std::shared_ptr<Shoot>     shoot                     = nullptr;     //发射结构 /添加时间2025/07/07


// 电机智能指针（核心控制对象）
std::shared_ptr<DJIMotor> gm6020 = nullptr;  // GM6020（大疆）
std::shared_ptr<DMMotor>  j4310 = nullptr;   // J4310（达妙）
std::shared_ptr<DMMotor>  j8009 = nullptr;   // J8009（达妙）

// 串口接收缓冲区（用于指令解析）
#define SERIAL_BUF_SIZE 64
uint8_t serial_rx_buf[SERIAL_BUF_SIZE] = {0};
uint16_t serial_rx_len = 0;

// void RobotCMDTask(void* arg __attribute__((unused)));               //机器人命令任务
// void ChassisTask(void* arg __attribute__((unused)));                //底盘任务
// void INSTask(void* arg __attribute__((unused)));                    //IMU任务
// void GimbalTask(void* arg __attribute__((unused)));                 //云台任务
// void ShootTask(void* arg __attribute__((unused)));                  //发射任务
void Motor_Control_Task(void *argument);
void Serial_Parse_Task(void *argument);  // 串口指令解析任务

// 串口发送格式化字符串（用于反馈）
void Serial_Send_Format(const char* format, ...) {
    char buf[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, strlen(buf), 100);  // 用串口1
}

// 串口接收中断回调（HAL库）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        if (serial_rx_len < SERIAL_BUF_SIZE - 1 && serial_rx_buf[serial_rx_len] != '\n') {
            serial_rx_len++;
        } else {
            // 收到换行符，标记指令结束
            serial_rx_buf[serial_rx_len] = '\0';
            serial_rx_len = 0;  // 重置缓冲区
        }
        // 重新开启中断接收
        HAL_UART_Receive_IT(&huart1, &serial_rx_buf[serial_rx_len], 1);
    }
}

// 串口指令解析（核心：解析"电机名:角度"指令）
void Serial_Parse_Command(uint8_t* buf) {
    char* motor_name = (char*)buf;
    char* angle_str = strchr((char*)buf, ':');
    if (angle_str == nullptr) {
        Serial_Send_Format("[错误] 指令格式错误！正确格式：电机名:角度（例:GM6020:30)\n");
        Serial_Send_Format("[支持] GM6020/J4310/J8009,角度支持小数(例:J4310:45.5)\n");
        return;
    }
    *angle_str = '\0';  // 分割电机名和角度
    angle_str++;
    float target_angle = atof(angle_str);  // 字符串转角度（度）

    // 控制对应电机
    if (strcmp(motor_name, "GM6020") == 0 || strcmp(motor_name, "gm6020") == 0) {
        if (gm6020) {
            gm6020->setTargetAngle(target_angle);
            Serial_Send_Format("[GM6020] 目标角度：%.1f° | 当前角度：%.1f° | 限位：%.1f°~%.1f°\n",
                target_angle, gm6020->measure_.total_angle,
                gm6020->min_angle_, gm6020->max_angle_);
        } else {
            Serial_Send_Format("[错误] GM6020未初始化!\n");
        }
    } else if (strcmp(motor_name, "J4310") == 0 || strcmp(motor_name, "j4310") == 0) {
        if (j4310) {
            j4310->setTargetAngle(target_angle);
            Serial_Send_Format("[J4310] 目标角度：%.1f° | 当前角度：%.1f° | 限位：%.1f°~%.1f°\n",
                target_angle, j4310->getCurrentAngle(),
                j4310->min_angle_, j4310->max_angle_);
        } else {
            Serial_Send_Format("[错误] J4310未初始化!\n");
        }
    } else if (strcmp(motor_name, "J8009") == 0 || strcmp(motor_name, "j8009") == 0) {
        if (j8009) {
            j8009->setTargetAngle(target_angle);
            Serial_Send_Format("[J8009] 目标角度：%.1f° | 当前角度：%.1f° | 限位：%.1f°~%.1f°\n",
                target_angle, j8009->getCurrentAngle(),
                j8009->min_angle_, j8009->max_angle_);
        } else {
            Serial_Send_Format("[错误] J8009未初始化!\n");
        }
    } else {
        Serial_Send_Format("[错误] 未知电机名：%s!支持GM6020/J4310/J8009\n", motor_name);
    }
}
void app_main()         
{
    taskENTER_CRITICAL();                //进入临界区(关闭中断，防止中断影响操作系统运行)
    RTTLog::init();                      //初始化实时日志系统
    STM32TimeDWT::DWT_Init(168);         //初始化高精度计时器(168MHz时钟)
    LOGINFO("Robot", "System Init");     //记录系统启动日志
    STM32CAN_Init();                     //初始化CAN总线
    StartDaemonTask();                   //启动守护任务
    StartMotorControlTask();             //启动电机控制任务

    // 初始化串口1（115200-8-N-1，用于指令接收）
    HAL_UART_Receive_IT(&huart1, serial_rx_buf, 1);
    Serial_Send_Format("串口控制初始化成功！\n");
// //创建全局共享对象
    gimbal_yaw_motor_angle_ptr = std::make_shared<float>(0.0f);     //云台(yaw)角度，初值为0
    chassis                    = std::make_shared<Chassis>();       //底盘实例
    imu_data                   = INS_Init();                        //初始化IMU并获取数据指针
    robotcmd                   = std::make_shared<RobotCMD>(imu_data, gimbal_yaw_motor_angle_ptr);  //命令模块
    gimbal                     = std::make_shared<Gimbal>(imu_data, gimbal_yaw_motor_angle_ptr);    //云台模块
    shoot                      = std::make_shared<Shoot>();       //发射任务 

    // 核心：初始化三个电机（CAN初始化后创建，避免空指针）
    // 1. GM6020（CAN2，ID=1，限位0~90度）
    gm6020 = std::make_shared<DJIMotor>(
        &hcan2, 1, MotorType::GM6020,
        {CloseloopType::ANGLE_LOOP, CloseloopType::SPEED_LOOP, false, FeedbackType::INTERNAL, FeedbackType::INTERNAL},
        {
            .pid_angle_ = {6.0f, 0.2f, 0.3f, 0.0f, 0.0f},  // 位置环PID（Kp=6, Ki=0.2, Kd=0.3）
            .pid_speed_ = {5.0f, 0.1f, 0.2f, 0.0f, 0.0f}   // 速度环PID
        }
    );
    gm6020->setAngleLimit(0.0f, 90.0f);  // 软件限位：只能转0~90度
    gm6020->enable_ = true;              // 使能电机

    // 2. J4310（CAN1，TX=0x205，RX=0x206，限位-45~45度）
    j4310 = std::make_shared<DMMotor>(
        &hcan1, 0x205, 0x206, MotorType::DM_J4310,
        {CloseloopType::ANGLE_LOOP, CloseloopType::SPEED_LOOP, false, FeedbackType::INTERNAL, FeedbackType::INTERNAL},
        {.pid_angle_ = {5.0f, 0.1f, 0.2f, 0.0f, 0.0f}}
    );
    j4310->setAngleLimit(-45.0f, 45.0f); // 软件限位：±45度
    j4310->enable_ = true;

    // 3. J8009（CAN1，TX=0x207，RX=0x208，限位-120~120度）
    j8009 = std::make_shared<DMMotor>(
        &hcan1, 0x207, 0x208, MotorType::DM_J8009,
        {CloseloopType::ANGLE_LOOP, CloseloopType::SPEED_LOOP, false, FeedbackType::INTERNAL, FeedbackType::INTERNAL},
        {.pid_angle_ = {4.5f, 0.1f, 0.2f, 0.0f, 0.0f}}
    );
    j8009->setAngleLimit(-120.0f, 120.0f); // 软件限位：±120度
    j8009->enable_ = true;
    
// //创建ROTS任务
    xTaskCreate(RobotCMDTask, "RobotCMDTask", 1024, nullptr, osPriorityNormal, nullptr);    //机器人命令任务，普通优先级，非最高非最低 
    xTaskCreate(ChassisTask, "ChassisTask", 1024, nullptr, osPriorityNormal, nullptr);      //底盘任务
    xTaskCreate(INSTask, "INSTask", 1024, nullptr, osPriorityNormal, nullptr);              //IMU任务
    xTaskCreate(GimbalTask, "GimbalTask", 1024, nullptr, osPriorityNormal, nullptr);        //云台任务

    xTaskCreate(ShootTask, "ShootTask", 1024, nullptr, osPriorityNormal, nullptr);         //发射任务

    taskEXIT_CRITICAL();                  //退出临界值(恢复中断)
    LOGINFO("Robot", "Robot Init");       //记录初始化完成日志
Serial_Send_Format("所有电机初始化完成！\n");
Serial_Send_Format("GM6020限位:0~90度 | J4310限位:-45~45度 | J8009限位:-120~120度\n");
}

void RobotCMDTask(void* arg __attribute__((unused)))        //机器人控制任务
{
    while (true)                
    {
        robotcmd->RobotCMDTask();           //执行处理任务
        vTaskDelay(2 / portTICK_PERIOD_MS); //延时2s
    }
}

void ChassisTask(void* arg __attribute__((unused)))         //底盘任务
{
    while (true)
    {
        chassis->ChassisTask();
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void INSTask(void* arg __attribute__((unused)))             //IMU任务
{
    while (true)
    {
        INS_Task();
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void GimbalTask(void* arg __attribute__((unused)))          //云台任务
{
    while (true)
    {
        gimbal->GimbalTask();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}


void ShootTask(void* arg __attribute__((unused)))          //射击任务
{
    while (true)
    {
        shoot->ShootTask();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }
}
// 电机控制任务（10ms周期，执行PID+CAN发送）
void Motor_Control_Task(void *argument) {
    while (1) {
        if (gm6020) gm6020->motorControl();  // 执行GM6020控制（限位→PID→CAN）
        if (j4310) j4310->motorControl();    // 执行J4310控制
        if (j8009) j8009->motorControl();    // 执行J8009控制
        vTaskDelay(10 / portTICK_PERIOD_MS); // 100Hz控制频率
    }
}

// 串口解析任务（循环检查指令，避免中断中处理耗时逻辑）
void Serial_Parse_Task(void *argument) {
    while (1) {
        if (serial_rx_buf[0] != '\0') {  // 检测到新指令
            Serial_Parse_Command(serial_rx_buf);  // 解析指令
            memset(serial_rx_buf, 0, SERIAL_BUF_SIZE);  // 清空缓冲区
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);  // 5ms检查一次
    }
}
