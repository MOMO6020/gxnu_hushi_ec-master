/*
 * @Author       : Notch-FGJ mail.fgj.com@gmail.com
 * @Date         : 2025-03-30 22:11:35
 * @LastEditors  : Notch-FGJ mail.fgj.com@gmail.com
 * @LastEditTime : 2025-04-15 15:46:20
 * @FilePath     : \FrameworkA_FGJ\bsp\can\stm32_can.cpp
 * @Description  : CAN总线封装
 */
#include "stm32_can.hpp"
#include "bsp/log/log.hpp"
#include "bsp/system/time.hpp"
#include "cmsis_os2.h"

class STM32CAN : public ICAN
{
public:
    STM32CAN(CAN_HandleTypeDef* _hcan) : hcan_(_hcan)
    {   //根据CAN实例确定使用哪个FIFO口
        if (hcan_->Instance == CAN1)  // CAN1 - FIFO0
            fifo_ = CAN_RX_FIFO0;
        else if (hcan_->Instance == CAN2)  // CAN2 - FIFO1
            fifo_ = CAN_RX_FIFO1;
        else
            while (true)
            {
                LOGERROR("CAN", "CAN Instance Error");
                osDelay(1000);
            }
        //激活对应的FIFO的消息挂起中断
        if (fifo_ == CAN_RX_FIFO0)
            HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);
        else
            HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO1_MSG_PENDING);
        //配置CAN过滤器
        CAN_FilterTypeDef can_filter;
        can_filter.FilterIdHigh         = 0;            //过滤器ID高16位
        can_filter.FilterIdLow          = 0;            //过滤器ID低16位
        can_filter.FilterMode           = CAN_FILTERMODE_IDMASK;    //掩码模式
        can_filter.FilterScale          = CAN_FILTERSCALE_32BIT;    //32位模式
        can_filter.FilterMaskIdHigh     = 0;            //掩码高16位
        can_filter.FilterMaskIdLow      = 0;            //掩码低16位
        can_filter.FilterFIFOAssignment = fifo_;        //绑定到FIFO
        can_filter.FilterActivation     = ENABLE;       //启用过滤器
        can_filter.FilterBank           = 0;            //起始过滤器组
        can_filter.SlaveStartFilterBank = 14;           //CAN2起始过滤器组
//这里初始化一个过滤器，使用32位的掩码模式，掩码全0表示接收所有消息。过滤器分配给之前确定的FIFO，并启用。
//注意这里设置了`SlaveStartFilterBank=14`，这是因为STM32F4/F7等系列中，CAN1和CAN2共享28个过滤器，CAN1
//使用0-13，CAN2使用14-27。这里配置的是第一个过滤器（FilterBank=0），对于CAN2，后面会重新设置FilterBank（在`updateRxFilter`中）。
        if (HAL_CAN_ConfigFilter(hcan_, &can_filter) != HAL_OK)     //配置过滤器错误处理
        {
            LOGERROR("CAN", "CAN Filter Config Failed");
            while (true)
            {
                osDelay(1000);
            }
        }
        if (HAL_CAN_Start(hcan_) != HAL_OK)     //启动CAN，启动失败，则循环
        {
            LOGERROR("CAN", "CAN Start Failed");
            while (true)
            {
                osDelay(1000);
            }
        }
        HAL_CAN_ActivateNotification(hcan_, CAN_IT_ERROR);      //错误中断
    }
    ~STM32CAN() override = default;     //析构函数，在 C++ 里，析构函数的名称是在类名前加上波浪线（~），其作用是在对象生命周期结束时释放资源。
    ErrorCode transmit(const ClassicPacket& packet, uint32_t timeout = 1000) override       //数据发送
    {
        CAN_TxHeaderTypeDef tx_header;
        tx_header.DLC = sizeof(packet.data);    //固定DLC=8
        switch (packet.type)              // 根据packet.type设置IDE和RTR
        {
            case Type::STANDARD:                //标准帧
                tx_header.IDE = CAN_ID_STD;
                tx_header.RTR = CAN_RTR_DATA;
                break;
            case Type::EXTENDED:                //扩展帧
                tx_header.IDE = CAN_ID_EXT;
                tx_header.RTR = CAN_RTR_DATA;
                break;
            case Type::REMOTE_STANDARD:         //远程标准帧
                tx_header.IDE = CAN_ID_STD;
                tx_header.RTR = CAN_RTR_REMOTE;
                break;
            case Type::REMOTE_EXTENDED:         //远程扩展帧
                tx_header.IDE = CAN_ID_EXT;
                tx_header.RTR = CAN_RTR_REMOTE;
                break;
            default: return ErrorCode::INVALID; //无效参数
        }
        //设置标准ID和扩展ID，如果是扩展帧，则StdId设为0，ExtId设为包ID；如果是标准帧，则StdId设为包ID，ExtId设为0。
        tx_header.StdId              = (packet.type == Type::EXTENDED) ? 0 : packet.id;
        tx_header.ExtId              = (packet.type == Type::EXTENDED) ? packet.id : 0;
        tx_header.TransmitGlobalTime = DISABLE;
        //等待发送邮箱空闲
        uint64_t start_time          = STM32TimeDWT::GetMilliseconds();
        while (HAL_CAN_GetTxMailboxesFreeLevel(hcan_) == 0)
        {
            if (STM32TimeDWT::GetMilliseconds() - start_time > timeout)
            {
                LOGWARNING("CAN", "Transmit Timeout");
                return ErrorCode::TIMEOUT;
            }
            osDelay(1);  // 等待发送邮箱空闲
        }
        if (HAL_CAN_AddTxMessage(hcan_, &tx_header, packet.data, &tx_mailbox_) != HAL_OK)   //调用HAL函数发送消息
        {
            LOGERROR("CAN", "Transmit Failed");
            return ErrorCode::FAILED;
        }
        return ErrorCode::OK;
    }
    //设置接收回调
    ErrorCode setRxCallback(uint16_t rx_id, std::function<void(const uint8_t*, const uint8_t)> callback) override
    {
        if (callback == nullptr)
        {
            LOGERROR("CAN", "Callback is nullptr");
            return ErrorCode::INVALID;
        }
        callbackMap[rx_id] = callback;  //将ID和回调函数存入一个映射表（callbackMap），然后更新接收过滤器。
        updateRxFilter();
        LOGDEBUG("CAN", "Set Rx Callback: " + std::to_string(rx_id));
        return ErrorCode::OK;
    }
    std::unordered_map<uint16_t, std::function<void(const uint8_t*, const uint8_t)>> callbackMap;  ///< 回调函数映射表
private:
    // 仅存储STD_ID, 2个CAN共享28个过滤器, 每个过滤器2个ID(CAN2作为从机)
    void updateRxFilter()       //检查 ID 数量限制 (28 组过滤器)
    {
        if (callbackMap.size() > 28)
        {
            while (true)
            {
                LOGERROR("CAN", "CAN Filter Size Exceeded");
                osDelay(1000);
            }
        }
        // 配置过滤器为 16 位列表模式
        CAN_FilterTypeDef can_filter;
        can_filter.FilterMode           = CAN_FILTERMODE_IDLIST;
        can_filter.FilterScale          = CAN_FILTERSCALE_16BIT;    //每组存两个ID
        can_filter.SlaveStartFilterBank = 14;
        can_filter.FilterActivation     = ENABLE;
        if (fifo_ == CAN_RX_FIFO0)      //设置过滤器起始位置
        {
            can_filter.FilterFIFOAssignment = CAN_RX_FIFO0;
            can_filter.FilterBank           = 0;
        }
        else
        {
            can_filter.FilterFIFOAssignment = CAN_RX_FIFO1;
            can_filter.FilterBank           = 14;
        }
        //遍历回调表，每两个 ID 配置一组过滤器
        auto it = callbackMap.begin();
        while (it != callbackMap.end())
        {
            uint16_t first_id       = it->first;
            uint16_t second_id      = (++it == callbackMap.end()) ? 0 : it->first;      //不足两个用0填充
            can_filter.FilterIdHigh = first_id << 5;        //ID 左移 5 位对齐寄存器 (STID[10:0] 占位 [15:5])
            can_filter.FilterIdLow  = second_id << 5;
            HAL_CAN_ConfigFilter(hcan_, &can_filter);
            can_filter.FilterBank++;                //移动到下一组
            if (it != callbackMap.end())            //双步迭代
                ++it;
        }
        HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_ActivateNotification(hcan_, CAN_IT_RX_FIFO1_MSG_PENDING);
    }

private:
    CAN_HandleTypeDef* hcan_;
    uint32_t           fifo_;
    uint32_t           tx_mailbox_;
};

std::shared_ptr<STM32CAN> __can_handle_1 = nullptr;
std::shared_ptr<STM32CAN> __can_handle_2 = nullptr;
//获取实例句柄
CANHandle_t STM32CAN_GetInstance(CAN_HandleTypeDef* _hcan)
{
    auto& handle = (_hcan->Instance == CAN1) ? __can_handle_1 : __can_handle_2;
    return handle;
}

void STM32CAN_Init()
{
    extern CAN_HandleTypeDef hcan1;
    extern CAN_HandleTypeDef hcan2;
    //初始化CAN实例
    if (__can_handle_1 == nullptr)
    {
        __can_handle_1 = std::make_shared<STM32CAN>(&hcan1);
        LOGINFO("CAN", "CAN1 Init Success");
    }
    if (__can_handle_2 == nullptr)
    {
        __can_handle_2 = std::make_shared<STM32CAN>(&hcan2);
        LOGINFO("CAN", "CAN2 Init Success");
    }
}
//FIFO0中断（CAN1）
extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             fifo0_buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, fifo0_buffer) != HAL_OK)
    {
        LOGERROR("CAN", "Receive Failed");
        return;
    }
    auto it = __can_handle_1->callbackMap.find(rx_header.StdId);        //查找回调并触发
    if (it != __can_handle_1->callbackMap.end())
    {
        it->second(fifo0_buffer, rx_header.DLC);
    }
}
//FIFO0中断（CAN2）
extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t             fifo1_buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, fifo1_buffer) != HAL_OK)
    {
        LOGERROR("CAN", "Receive Failed");
        return;
    }
    auto it = __can_handle_2->callbackMap.find(rx_header.StdId);
    if (it != __can_handle_2->callbackMap.end())
    {
        it->second(fifo1_buffer, rx_header.DLC);
    }
}
//错误中断
extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan)
{
    if (hcan->ErrorCode != HAL_CAN_ERROR_NONE)
    {
        LOGERROR("CAN", "CAN Error: " + std::to_string(hcan->ErrorCode));
    }
}