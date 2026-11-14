/**
 * @file dvc_vofa.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief vofa
 * @version 0.1
 * @date 2025-09-22 0.1 新建
 *
 * @copyright USTC-RoboWalker (c) 2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_powermeter.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 功率计UART初始化
 *
 * @param huart 指定的UART
 * @param __Frame_Header 帧头标识符
 */
void Class_Powermeter_UART::Init(Struct_UART_Manage_Object *huart, const uint8_t &__Frame_Header)
{
    UART_Manage_Object = huart;

    Frame_Header = __Frame_Header;
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 * @param Length 接收数据长度
 */
void Class_Powermeter_UART::UART_RxCpltCallback(const uint8_t *Rx_Data, const uint16_t &Length)
{
    if (Length != sizeof(Struct_Powermeter_UART_Rx_Data) || Rx_Data == nullptr) { return; }
    auto *previous = UART_Manage_Object->Rx_Buffer_Ready;
    UART_Manage_Object->Rx_Buffer_Ready = const_cast<uint8_t *>(Rx_Data);
    Data_Process();
    UART_Manage_Object->Rx_Buffer_Ready = previous;
    ++Flag;
}

/**
 * @brief TIM定时器中断定期检测功率计是否存活
 *
 */
void Class_Powermeter_UART::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否有接收过功率计数据
    if (Flag == Pre_Flag)
    {
        // 功率计掉线处理
        Powermeter_Status = Powermeter_Status_DISABLE;
        Rx_Data.Current = 0.0f;
        Rx_Data.Voltage = 0.0f;
        Rx_Data.Power = 0.0f;
    }
    else
    {
        // 功率计在线
        Powermeter_Status = Powermeter_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Powermeter_UART::Data_Process()
{
    Struct_Powermeter_UART_Rx_Data *tmp_buffer = (Struct_Powermeter_UART_Rx_Data *) UART_Manage_Object->Rx_Buffer_Ready;

    // 帧头错误直接返回
    if (tmp_buffer->Frame_Header != Frame_Header)
    {
        return;
    }

    // 数据处理过程
    memcpy(&Rx_Data, &((uint8_t *)tmp_buffer)[1], sizeof(Struct_Powermeter_Rx_Data));
}

/**
 * @brief 功率计CAN初始化
 *
 * @param hcan 指定的CAN
 */
void Class_Powermeter_CAN::Init(Struct_CAN_Manage_Object *hcan)
{
    CAN_Manage_Object = hcan;
}

/**
 * @brief CAN通信接收回调函数
 *
 */
void Class_Powermeter_CAN::CAN_RxCpltCallback()
{
    if (CAN_Manage_Object->Rx_Header.DataLength != 8) { return; }
    Flag += 1;
    Data_Process();
}

/**
 * @brief TIM定时器中断定期检测功率计是否存活
 *
 */
void Class_Powermeter_CAN::TIM_100ms_Alive_PeriodElapsedCallback()
{
    // 判断该时间段内是否有接收过功率计数据
    if (Flag == Pre_Flag)
    {
        // 功率计掉线处理
        Powermeter_Status = Powermeter_Status_DISABLE;
        Rx_Data.Current = 0.0f;
        Rx_Data.Voltage = 0.0f;
        Rx_Data.Power = 0.0f;
    }
    else
    {
        // 功率计在线
        Powermeter_Status = Powermeter_Status_ENABLE;
    }
    Pre_Flag = Flag;
}

/**
 * @brief 数据处理过程
 *
 */
void Class_Powermeter_CAN::Data_Process()
{
    Struct_Powermeter_CAN_Rx_Data *tmp_buffer = (Struct_Powermeter_CAN_Rx_Data *) CAN_Manage_Object->Rx_Buffer;

    // 数据处理过程
    memcpy(&Rx_Data, tmp_buffer, sizeof(Struct_Powermeter_CAN_Rx_Data));
    Rx_Data.Power = Rx_Data.Current * Rx_Data.Voltage;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/