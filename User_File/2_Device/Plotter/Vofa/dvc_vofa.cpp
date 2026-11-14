#include <cstdlib>
#include <cmath>
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

#include "dvc_vofa.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief vofa初始化
 *
 * @param __huart 指定的UART
 * @param __Rx_Variable_Assignment_Num 接收指令字典的数量
 * @param __Rx_Variable_Assignment_List 接收指令字典列表
 * @param __Frame_Tail 帧尾标识符
 */
void Class_Vofa_UART::Init(Struct_UART_Manage_Object *huart, const uint8_t &__Rx_Variable_Assignment_Num, const char **__Rx_Variable_Assignment_List, const uint32_t &__Frame_Tail)
{
    UART_Manage_Object = huart;

    Rx_Variable_Num = __Rx_Variable_Assignment_Num;
    Rx_Variable_List = const_cast<char **>(__Rx_Variable_Assignment_List);
    Frame_Tail = __Frame_Tail;
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 * @param Length 接收数据长度
 */
void Class_Vofa_UART::UART_RxCpltCallback(const uint8_t *Rx_Data, const uint16_t &Length)
{
    if (Rx_Data == nullptr || Length == 0 || Length > 511) { return; }
    auto *previous = UART_Manage_Object->Rx_Buffer_Ready;
    UART_Manage_Object->Rx_Buffer_Ready = const_cast<uint8_t *>(Rx_Data);
    Data_Process(Length);
    UART_Manage_Object->Rx_Buffer_Ready = previous;
}

/**
 * @brief TIM定时器中断增加数据到发送缓冲区
 *
 */
void Class_Vofa_UART::TIM_1ms_Write_PeriodElapsedCallback()
{
    Output();

    UART_Transmit_Data(UART_Manage_Object, Tx_Buffer, Data_Number * sizeof(float) + sizeof(uint32_t));
}

/**
 * @brief 数据处理过程
 *
 * @param Length 接收数据长度
 */
void Class_Vofa_UART::Data_Process(const uint16_t &Length)
{
    int flag;
    flag = _Judge_Variable_Name(Length);
    _Judge_Variable_Value(Length, flag);
}

/**
 * @brief 判断指令变量名
 *
 * @param Length 接收数据长度
 * @return uint8_t 指令数值位置的指针, 也就是"variable=value#"中v的坐标
 */
uint8_t Class_Vofa_UART::_Judge_Variable_Name(const uint16_t &Length)
{
    Variable_Index = -1;
    const auto *data = UART_Manage_Object->Rx_Buffer_Ready;
    uint16_t end = 0;
    while (end < Length && end < VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH - 1 && data[end] != '=') { ++end; }
    if (end >= Length || data[end] != '=' || Rx_Variable_List == nullptr) { return Length; }
    for (uint8_t index = 0; index < Rx_Variable_Num; ++index) {
        const char *name = Rx_Variable_List[index];
        if (name != nullptr && strlen(name) == end && memcmp(name, data, end) == 0) { Variable_Index = index; break; }
    }
    return end + 1;
}

/**
 * @brief 判断指令变量数值
 *
 * @param Length 接收数据长度
 * @param flag 指令数值位置的指针
 */
void Class_Vofa_UART::_Judge_Variable_Value(const uint16_t &Length, int flag)
{
    if (Variable_Index < 0 || flag < 0 || flag >= Length) { return; }
    char number[64]{};
    const auto *data = UART_Manage_Object->Rx_Buffer_Ready;
    size_t length = 0;
    while (flag < Length && data[flag] != '#' && length < sizeof(number) - 1) { number[length++] = data[flag++]; }
    if (flag >= Length || data[flag] != '#' || length == 0) { Variable_Index = -1; return; }
    char *end = nullptr;
    const float value = std::strtof(number, &end);
    if (end != number + length || !std::isfinite(value)) { Variable_Index = -1; return; }
    Variable_Value = value;
}

/**
 * @brief vofa数据输出到UART发送缓冲区
 *
 */
void Class_Vofa_UART::Output()
{
    uint8_t *tmp_buffer = Tx_Buffer;

    memset(tmp_buffer, 0, UART_BUFFER_SIZE);

    for (int i = 0; i < Data_Number; i++)
    {
        memcpy(tmp_buffer + i * sizeof(uint32_t), Data[i], sizeof(uint32_t));
    }

    memcpy(tmp_buffer + Data_Number * sizeof(uint32_t), &Frame_Tail, sizeof(uint32_t));
}

/**
 * @brief vofa初始化
 *
 * @param __Rx_Variable_Assignment_Num
 * @param __Rx_Variable_Assignment_List
 * @param __Frame_Tail
 */
void Class_Vofa_USB::Init(Struct_USB_Manage_Object *driver, const uint8_t &__Rx_Variable_Assignment_Num, const char **__Rx_Variable_Assignment_List, const uint32_t &__Frame_Tail)
{
    USB_Manage_Object = driver;

    Rx_Variable_Num = __Rx_Variable_Assignment_Num;
    Rx_Variable_List = const_cast<char **>(__Rx_Variable_Assignment_List);
    Frame_Tail = __Frame_Tail;
}

/**
 * @brief USB通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 * @param Length 接收数据长度
 */
void Class_Vofa_USB::USB_RxCallback(const uint8_t *Rx_Data, const uint16_t &Length)
{
    if (Rx_Data == nullptr || Length == 0 || Length > 511) { return; }
    auto *previous = USB_Manage_Object->Rx_Buffer_Ready;
    USB_Manage_Object->Rx_Buffer_Ready = const_cast<uint8_t *>(Rx_Data);
    Data_Process(Length);
    USB_Manage_Object->Rx_Buffer_Ready = previous;
}

/**
 * @brief TIM定时器中断增加数据到发送缓冲区
 *
 */
void Class_Vofa_USB::TIM_1ms_Write_PeriodElapsedCallback()
{
    Output();

    USB_Transmit_Data(USB_Manage_Object, Tx_Buffer, Data_Number * sizeof(float) + sizeof(uint32_t));
}

/**
 * @brief 数据处理过程
 *
 * @param Length 接收数据长度
 */
void Class_Vofa_USB::Data_Process(const uint16_t &Length)
{
    int flag;
    flag = _Judge_Variable_Name(Length);
    _Judge_Variable_Value(Length, flag);
}

/**
 * @brief 判断指令变量名
 *
 * @param Length 接收数据长度
 * @return uint8_t 指令数值位置的指针, 也就是"variable=value#"中v的坐标
 */
uint8_t Class_Vofa_USB::_Judge_Variable_Name(const uint16_t &Length)
{
    Variable_Index = -1;
    const auto *data = USB_Manage_Object->Rx_Buffer_Ready;
    uint16_t end = 0;
    while (end < Length && end < VOFA_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH - 1 && data[end] != '=') { ++end; }
    if (end >= Length || data[end] != '=' || Rx_Variable_List == nullptr) { return Length; }
    for (uint8_t index = 0; index < Rx_Variable_Num; ++index) {
        const char *name = Rx_Variable_List[index];
        if (name != nullptr && strlen(name) == end && memcmp(name, data, end) == 0) { Variable_Index = index; break; }
    }
    return end + 1;
}

/**
 * @brief 判断指令变量数值
 *
 * @param Length 接收数据长度
 * @param flag 指令数值位置的指针
 */
void Class_Vofa_USB::_Judge_Variable_Value(const uint16_t &Length, int flag)
{
    if (Variable_Index < 0 || flag < 0 || flag >= Length) { return; }
    char number[64]{};
    const auto *data = USB_Manage_Object->Rx_Buffer_Ready;
    size_t length = 0;
    while (flag < Length && data[flag] != '#' && length < sizeof(number) - 1) { number[length++] = data[flag++]; }
    if (flag >= Length || data[flag] != '#' || length == 0) { Variable_Index = -1; return; }
    char *end = nullptr;
    const float value = std::strtof(number, &end);
    if (end != number + length || !std::isfinite(value)) { Variable_Index = -1; return; }
    Variable_Value = value;
}

/**
 * @brief vofa数据输出到USB发送缓冲区
 *
 */
void Class_Vofa_USB::Output()
{
    uint8_t *tmp_buffer = Tx_Buffer;

    memset(tmp_buffer, 0, USB_BUFFER_SIZE);

    for (int i = 0; i < Data_Number; i++)
    {
        memcpy(tmp_buffer + i * sizeof(uint32_t), Data[i], sizeof(uint32_t));
    }

    memcpy(tmp_buffer + Data_Number * sizeof(uint32_t), &Frame_Tail, sizeof(uint32_t));
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/