/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-04-01 11:19:43
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-01 11:33:47
 * @FilePath     : \FrameworkC_CMake\bsp\system\mutex.cpp
 * @Description  : 互斥锁类, 基于FreeRTOS的信号量封装
 */
#include "mutex.hpp"

Mutex::Mutex() : mutex_(xSemaphoreCreateBinary())       //`xSemaphoreCreateBinary()`是FreeRTOS中创建二进制信号量的函数。有两种状态ful和empty
{                                                       //创建后默认为空(empty)，不可用，解锁互斥锁后可用(full)
    unlock();       //初始化后解锁互斥锁
}

Mutex::~Mutex()
{
    vSemaphoreDelete(mutex_);       //删除信号量资源
}

ErrorCode Mutex::lock()     //上锁
{
    if (xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE)
        return ErrorCode::FAILED;
    return ErrorCode::OK;
}

ErrorCode Mutex::tryLock()  //尝试加锁
{
    if (xSemaphoreTake(mutex_, 0) != pdTRUE)
        return ErrorCode::BUSY;
    return ErrorCode::OK;
}

void Mutex::unlock()        //解锁方法
{
    xSemaphoreGive(mutex_);
}

ErrorCode Mutex::trylockFromISR(bool in_isr)
{
    if (in_isr)                     //判断是否存在中断
    {
        BaseType_t px_higher_priority_task_woken = 0;
        BaseType_t x_return = xSemaphoreTakeFromISR(mutex_, &px_higher_priority_task_woken);    //中断专用API标记是否有高优先级任务呗唤醒
        if (x_return == pdPASS)     //如果有更高优先级的任务被唤醒
        {   //检查是否需要任务切换
            if (px_higher_priority_task_woken != pdFALSE)
                portYIELD_FROM_ISR(px_higher_priority_task_woken);      //请求在中断退出时进行任务切换
            return ErrorCode::OK;
        }
        else
            return ErrorCode::BUSY;
    }
    else
        return tryLock();       //不在中断，调用普通版本
}

void Mutex::unlockFromISR(bool in_isr)      //中断里面解锁互斥锁
{
    if (in_isr)
    {
        BaseType_t px_higher_priority_task_woken = 0;
        xSemaphoreGiveFromISR(mutex_, &px_higher_priority_task_woken);
        if (px_higher_priority_task_woken != pdFALSE)
            portYIELD_FROM_ISR(px_higher_priority_task_woken);
    }
    else
        unlock();
}