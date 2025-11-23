/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-08 13:00:44
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-06-24 19:32:53
 * @FilePath     : \gxnu_hushi_ec\modules\daemon\daemon.cpp
 * @Description  :
 */
#include "daemon.hpp"
#include "cmsis_os2.h"
#include <bsp/log/log.hpp>
#include <main.h>
#include <vector>

class Daemon : public IDaemon
{
public:
    Daemon(uint16_t timeout_cnt, uint16_t initial_cnt, std::function<void()> callback)
        : timeout_cnt_(timeout_cnt), current_cnt_(initial_cnt), callback_(callback)         //（超时计数值）、（初始计数值）、（超时回调函数）
    {
    }

    void setCallback(std::function<void()> callback) override { callback_ = callback; }     //设置回调函数

    void feed() override { current_cnt_ = timeout_cnt_; }                                   //喂狗操作

    void check()    //检查超时
    {
        if (current_cnt_ > 0)   //计数值大于0递减
            --current_cnt_;
        else if (callback_)     //计数值归零触发回调
            callback_();
    }

private:
    uint16_t              timeout_cnt_;     //超时计数值
    uint16_t              current_cnt_;     //当前计数值
    std::function<void()> callback_;        //超时回调函数
};

extern IWDG_HandleTypeDef            hiwdg;         //外部声明的独立看门狗句柄
std::vector<std::shared_ptr<Daemon>> daemon_list_;  //守护进程对象列表

void daemonTask(void* arg __attribute__((unused)))  //守护任务函数
{
    LOGINFO("Daemon", "Daemon task started");       //记录任务启动日志
    while (true)
    {
        // HAL_IWDG_Refresh(&hiwdg);                //刷新硬件看门狗
        for (auto& daemon : daemon_list_)           //执行超时检查
            daemon->check();        
        osDelay(DAEMON_PERIOD);                     //延时固定周期
    }
}

void StartDaemonTask()                              //守护进程启动函数
{
    // xTaskCreate(daemonTask, "DaemonTask", 512, nullptr, osPriorityNormal, nullptr);
    //定义线程属性
    osThreadAttr_t attr = {"DaemonTask", 0, NULL, 0, NULL, 512, osPriorityAboveNormal, 0, 0};  //任务名称，标准属性，栈大小，普通优先级，其他属性
    if (osThreadNew(daemonTask, nullptr, &attr) == nullptr)     //创建新任务
    {
        LOGERROR("Daemon", "Failed to create daemon task");     //错误日志
        while (true)                                            //创建失败进入死循环
            osDelay(1000);
    }
}

DaemonPtr registerDaemon(uint16_t timeout_cnt, uint16_t initial_cnt, std::function<void()> callback)
{   
    auto daemon = std::make_shared<Daemon>(timeout_cnt, initial_cnt, callback);     //创建守护进程对象
    daemon_list_.push_back(daemon);                                                 //添加到全局变量
    return daemon;                                          
}