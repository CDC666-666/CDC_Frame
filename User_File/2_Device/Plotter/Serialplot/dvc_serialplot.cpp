#include <cstdlib>
#include <cmath>
/**
 * @file dvc_serialplot.cpp
 * @author yssickjgd (1345578933@qq.com)
 * @brief 串口绘图
 * @version 1.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2025-08-14 1.1 适配达妙MC02板, 并引入USB虚拟串口的串口绘图
 *
 * @copyright USTC-RoboWalker (c) 2023-2025
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "dvc_serialplot.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 串口绘图初始化
 *
 * @param __huart 指定的UART
 * @param __Rx_Variable_Assignment_Num 接收指令字典的数量
 * @param __Rx_Variable_Assignment_List 接收指令字典列表
 * @param __Data_Type 传输数据类型, 默认float
 * @param __Frame_Header 帧头标识符
 */
void Class_Serialplot_UART::Init(Struct_UART_Manage_Object *huart, const Enum_Serialplot_Checksum_8 &__Checksum_8, const uint8_t &__Rx_Variable_Assignment_Num, const char **__Rx_Variable_Assignment_List, const Enum_Serialplot_Data_Type &__Data_Type, const uint8_t &__Frame_Header)
{
    UART_Manage_Object = huart;

    Checksum_8 = __Checksum_8;
    Rx_Variable_Num = __Rx_Variable_Assignment_Num;
    Rx_Variable_List = const_cast<char **>(__Rx_Variable_Assignment_List);
    Tx_Data_Type = __Data_Type;
    Frame_Header = __Frame_Header;

    Tx_Buffer[0] = __Frame_Header;
}

/**
 * @brief UART通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 * @param Length 接收数据长度
 */
void Class_Serialplot_UART::UART_RxCpltCallback(const uint8_t *Rx_Data, const uint16_t &Length)
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
void Class_Serialplot_UART::TIM_1ms_Write_PeriodElapsedCallback()
{
    Output();

    size_t data_length = 1;
    if (Checksum_8 == Serialplot_Checksum_8_ENABLE)
    {
        data_length++;
    }

    switch (Tx_Data_Type)
    {
    case (Serialplot_Data_Type_UINT8):
    {
        data_length += Data_Number * sizeof(uint8_t);

        break;
    }
    case (Serialplot_Data_Type_UINT16):
    {
        data_length += Data_Number * sizeof(uint16_t);

        break;
    }
    case (Serialplot_Data_Type_UINT32):
    {
        data_length += Data_Number * sizeof(uint32_t);

        break;
    }
    case (Serialplot_Data_Type_INT8):
    {
        data_length += Data_Number * sizeof(int8_t);

        break;
    }
    case (Serialplot_Data_Type_INT16):
    {
        data_length += Data_Number * sizeof(int16_t);

        break;
    }
    case (Serialplot_Data_Type_INT32):
    {
        data_length += Data_Number * sizeof(int32_t);

        break;
    }
    case (Serialplot_Data_Type_FLOAT):
    {
        data_length += Data_Number * sizeof(float);

        break;
    }
    case (Serialplot_Data_Type_DOUBLE):
    {
        data_length += Data_Number * sizeof(double);

        break;
    }
    }

    UART_Transmit_Data(UART_Manage_Object, Tx_Buffer, data_length);
}

/**
 * @brief 数据处理过程
 *
 * @param Length 接收数据长度
 */
void Class_Serialplot_UART::Data_Process(const uint16_t &Length)
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
uint8_t Class_Serialplot_UART::_Judge_Variable_Name(const uint16_t &Length)
{
    Variable_Index = -1;
    const auto *data = UART_Manage_Object->Rx_Buffer_Ready;
    uint16_t end = 0;
    while (end < Length && end < SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH - 1 && data[end] != '=') { ++end; }
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
void Class_Serialplot_UART::_Judge_Variable_Value(const uint16_t &Length, int flag)
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
 * @brief 串口绘图数据输出到UART发送缓冲区
 *
 */
void Class_Serialplot_UART::Output()
{
    uint8_t *tmp_buffer = Tx_Buffer;

    memset(tmp_buffer, 0, UART_BUFFER_SIZE);

    // 放置帧头
    tmp_buffer[0] = Frame_Header;

    // 填充数据
    if (Tx_Data_Type == Serialplot_Data_Type_UINT8 || Tx_Data_Type == Serialplot_Data_Type_INT8)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint8_t) + 1, Data[i], sizeof(uint8_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint8_t)] = Basic_Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint8_t));
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_UINT16 || Tx_Data_Type == Serialplot_Data_Type_INT16)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint16_t) + 1, Data[i], sizeof(uint16_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint16_t)] = Basic_Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint16_t));
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_UINT32 || Tx_Data_Type == Serialplot_Data_Type_INT32 || Tx_Data_Type == Serialplot_Data_Type_FLOAT)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint32_t) + 1, Data[i], sizeof(uint32_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint32_t)] = Basic_Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint32_t));
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_DOUBLE)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint64_t) + 1, Data[i], sizeof(uint64_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint64_t)] = Basic_Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint64_t));
        }
    }
}

void Class_Serialplot_USB::Init(Struct_USB_Manage_Object *driver, const Enum_Serialplot_Checksum_8 &__Checksum_8, const uint8_t &__Rx_Variable_Assignment_Num, const char **__Rx_Variable_Assignment_List, const Enum_Serialplot_Data_Type &__Data_Type, const uint8_t &__Frame_Header)
{
    USB_Manage_Object = driver;

    Checksum_8 = __Checksum_8;
    Rx_Variable_Num = __Rx_Variable_Assignment_Num;
    Rx_Variable_List = const_cast<char **>(__Rx_Variable_Assignment_List);
    Tx_Data_Type = __Data_Type;
    Frame_Header = __Frame_Header;

    Tx_Buffer[0] = __Frame_Header;
}

/**
 * @brief USB通信接收回调函数
 *
 * @param Rx_Data 接收的数据
 * @param Length 接收数据长度
 */
void Class_Serialplot_USB::USB_RxCallback(const uint8_t *Rx_Data, const uint16_t &Length)
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
void Class_Serialplot_USB::TIM_1ms_Write_PeriodElapsedCallback()
{
    Output();

    size_t data_length = 1;
    if (Checksum_8 == Serialplot_Checksum_8_ENABLE)
    {
        data_length++;
    }

    switch (Tx_Data_Type)
    {
    case (Serialplot_Data_Type_UINT8):
    {
        data_length += Data_Number * sizeof(uint8_t);

        break;
    }
    case (Serialplot_Data_Type_UINT16):
    {
        data_length += Data_Number * sizeof(uint16_t);

        break;
    }
    case (Serialplot_Data_Type_UINT32):
    {
        data_length += Data_Number * sizeof(uint32_t);

        break;
    }
    case (Serialplot_Data_Type_INT8):
    {
        data_length += Data_Number * sizeof(int8_t);

        break;
    }
    case (Serialplot_Data_Type_INT16):
    {
        data_length += Data_Number * sizeof(int16_t);

        break;
    }
    case (Serialplot_Data_Type_INT32):
    {
        data_length += Data_Number * sizeof(int32_t);

        break;
    }
    case (Serialplot_Data_Type_FLOAT):
    {
        data_length += Data_Number * sizeof(float);

        break;
    }
    case (Serialplot_Data_Type_DOUBLE):
    {
        data_length += Data_Number * sizeof(double);

        break;
    }
    }

    USB_Transmit_Data(USB_Manage_Object, Tx_Buffer, data_length);
}

/**
 * @brief 数据处理过程
 *
 * @param Length 接收数据长度
 */
void Class_Serialplot_USB::Data_Process(const uint16_t &Length)
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
uint8_t Class_Serialplot_USB::_Judge_Variable_Name(const uint16_t &Length)
{
    Variable_Index = -1;
    const auto *data = USB_Manage_Object->Rx_Buffer_Ready;
    uint16_t end = 0;
    while (end < Length && end < SERIALPLOT_RX_VARIABLE_ASSIGNMENT_MAX_LENGTH - 1 && data[end] != '=') { ++end; }
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
void Class_Serialplot_USB::_Judge_Variable_Value(const uint16_t &Length, int flag)
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
 * @brief 串口绘图数据输出到USB发送缓冲区
 *
 */
void Class_Serialplot_USB::Output()
{
    uint8_t *tmp_buffer = Tx_Buffer;

    memset(tmp_buffer, 0, USB_BUFFER_SIZE);

    // 放置帧头
    tmp_buffer[0] = Frame_Header;

    // 填充数据
    if (Tx_Data_Type == Serialplot_Data_Type_UINT8 || Tx_Data_Type == Serialplot_Data_Type_INT8)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint8_t) + 1, Data[i], sizeof(uint8_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint8_t)] = Basic_Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint8_t));
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_UINT16 || Tx_Data_Type == Serialplot_Data_Type_INT16)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint16_t) + 1, Data[i], sizeof(uint16_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint16_t)] = Basic_Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint16_t));
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_UINT32 || Tx_Data_Type == Serialplot_Data_Type_INT32 || Tx_Data_Type == Serialplot_Data_Type_FLOAT)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint32_t) + 1, Data[i], sizeof(uint32_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint32_t)] = Basic_Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint32_t));
        }
    }
    else if (Tx_Data_Type == Serialplot_Data_Type_DOUBLE)
    {
        for (int i = 0; i < Data_Number; i++)
        {
            memcpy(tmp_buffer + i * sizeof(uint64_t) + 1, Data[i], sizeof(uint64_t));
        }

        if (Checksum_8 != Serialplot_Checksum_8_DISABLE)
        {
            tmp_buffer[1 + Data_Number * sizeof(uint64_t)] = Basic_Math_Sum_8(tmp_buffer + 1, Data_Number * sizeof(uint64_t));
        }
    }
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/