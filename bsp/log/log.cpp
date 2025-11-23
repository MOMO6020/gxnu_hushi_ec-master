/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-03-28 11:57:00
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-03-31 10:42:49
 * @FilePath     : \FrameworkC_CMake\bsp\log\bsp_log.cpp
 * @Description  : 日志系统
 */

#include "log.hpp"
#include "SEGGER_RTT.h"         //SEGGER实时传输库
#include "bsp/system/time.hpp"  //代码中未用

#define RTT_BUFFER 0            //指定使用RTT的0号缓冲区

void RTTLog::init(void)         //初始化RTT通信模块
{
    SEGGER_RTT_Init();
}
// [LEVEL] [@NODE_NAME] MESSAGE
void RTTLog::debug(const std::string& node_name, const std::string& message)        //DEBUG级别
{   //六个参数（缓冲区，格式化字符串，亮蓝色控制符，节点名称，日志内容，重置颜色控制符）
    SEGGER_RTT_printf(RTT_BUFFER, "%s[Debug] [@%s] %s\n%s", RTT_CTRL_TEXT_BRIGHT_BLUE, node_name.c_str(),
                      message.c_str(), RTT_CTRL_RESET);
}

void RTTLog::info(const std::string& node_name, const std::string& message)         //INFO级别
{   //亮绿色
    SEGGER_RTT_printf(RTT_BUFFER, "%s[Info] [@%s] %s\n%s", RTT_CTRL_TEXT_BRIGHT_GREEN, node_name.c_str(),
                     message.c_str(), RTT_CTRL_RESET);
}

void RTTLog::warning(const std::string& node_name, const std::string& message)      //WANRING级别
{   //亮黄色
    SEGGER_RTT_printf(RTT_BUFFER, "%s[Warning] [@%s] %s\n%s", RTT_CTRL_TEXT_BRIGHT_YELLOW, node_name.c_str(), message.c_str(),
                      RTT_CTRL_RESET);
}

void RTTLog::error(const std::string& node_name, const std::string& message)        //ERROR级别
{   //亮红色
    SEGGER_RTT_printf(RTT_BUFFER, "%s[Error] [@%s] %s\n%s", RTT_CTRL_TEXT_BRIGHT_RED, node_name.c_str(), message.c_str(),
                      RTT_CTRL_RESET);
}
