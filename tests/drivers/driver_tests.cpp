#include <cmath>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "1_Middleware/Driver/ADC/drv_adc.h"
#include "1_Middleware/Driver/CAN/drv_can.h"
#include "1_Middleware/Driver/SPI/drv_spi.h"
#include "1_Middleware/Driver/USB/drv_usb.h"
#include "1_Middleware/Driver/WDG/drv_wdg.h"
#include "2_Device/BSP/W25Q64JV/bsp_w25q64jv.h"
#include "2_Device/Motor/Motor_DJI/dvc_motor_dji.h"
#include "2_Device/Motor/Motor_DM/dvc_motor_dm.h"
#include "2_Device/Plotter/Serialplot/dvc_serialplot.h"
#include "2_Device/Plotter/Vofa/dvc_vofa.h"

// 检查驱动对外行为；输入条件和位置，失败时终止当前测试。
void Check(bool value, const char* message) {
  if (!value) {
    throw std::runtime_error(message);
  }
}

// 测试CAN复制、溢出和帧校验；无输入，失败时抛出诊断。
void Can() {
  LibXR::CAN backend;
  Struct_CAN_Manage_Object driver;
  Check(driver.Bind(backend) && !driver.Bind(backend), "single binding");
  LibXR::CAN::ClassicPack frame{
      0x206, LibXR::CAN::Type::STANDARD, 8, {1, 2, 3}};
  backend.receiver.Run(true, frame);
  frame.data[0] = 99;
  Check(driver.Process() == 1 && driver.Rx_Buffer[0] == 1,
        "receive owns copied frame");
  Check(driver.Rx_Header.Identifier == 0x206 && driver.Rx_Timestamp != 0,
        "header and timestamp");
  Check(driver.Send(0x800, frame.data, 8) == HAL_ERROR, "invalid identifier");
  Check(driver.Send(1, frame.data, 9) == HAL_ERROR, "classic size validation");
  Check(driver.Send(1, frame.data, 8) == HAL_OK && backend.last.data[0] == 99,
        "valid send");
  backend.result = LibXR::ErrorCode::FULL;
  Check(driver.Send(1, frame.data, 8) == HAL_BUSY, "backpressure mapping");
  for (int index = 0; index < 40; ++index) {
    backend.receiver.Run(true, frame);
  }
  Check(driver.dropped.load() == 8 && driver.Process(2) == 2,
        "bounded reception");
  Check(driver.Group(0x1fe) != nullptr && driver.Group(0x999) == nullptr,
        "owned group buffers");
}

// 测试字节流及CDC复用接口；无输入，检查提交与容量语义。
void Uart() {
  LibXR::ReadPort read;
  LibXR::WritePort write;
  LibXR::UART backend{&read, &write};
  Struct_UART_Manage_Object driver;
  Check(driver.Bind(backend), "bind UART");
  const uint8_t data[] = {1, 2, 3};
  Check(driver.Send(data, 3) == HAL_OK && write.bytes.size() == 3,
        "empty TX queue still accepts");
  Check(driver.Send(data, 129) == HAL_ERROR, "capacity bound");
  read.bytes.assign(600, 0x55);
  Check(driver.Process() == 512 && driver.Process() == 88 &&
            driver.Process() == 0,
        "bounded drain");
  Check(driver.Rx_Time_Stamp != 0, "readout timestamp");
  Struct_USB_Manage_Object usb;
  Check(usb.Bind(backend) && USB_Transmit_Data(&usb, data, 3) == HAL_OK,
        "same CDC transport");
}

// 测试任意拆分、粘包、超长恢复及固定长度接收；无输入。
void Framing() {
  StreamFramer parser;
  std::vector<std::string> frames;
  const auto receive = [&frames](const uint8_t* data, uint16_t size) {
    frames.emplace_back(reinterpret_cast<const char*>(data), size);
  };
  const char* text = "gain=1#offset=-2#";
  for (size_t split = 0; split <= std::strlen(text); ++split) {
    parser.Reset(StreamFramer::Mode::TEXT);
    frames.clear();
    parser.Feed(reinterpret_cast<const uint8_t*>(text), split, receive);
    parser.Feed(reinterpret_cast<const uint8_t*>(text) + split,
                std::strlen(text) - split, receive);
    Check(frames.size() == 2 && frames[0] == "gain=1#" &&
              frames[1] == "offset=-2#",
          "split/coalesced frames");
  }
  parser.Reset(StreamFramer::Mode::TEXT);
  frames.clear();
  std::vector<uint8_t> oversized(700, 'x');
  oversized.push_back('#');
  parser.Feed(oversized.data(), oversized.size(), receive);
  parser.Feed(reinterpret_cast<const uint8_t*>(text), 7, receive);
  Check(frames.size() == 1 && frames[0] == "gain=1#",
        "oversize resynchronization");
  parser.Reset(StreamFramer::Mode::POWERMETER);
  frames.clear();
  const uint8_t packet[14] = {0xab};
  parser.Feed(packet, 5, receive);
  parser.Feed(packet + 5, 9, receive);
  Check(frames.size() == 1 && frames[0].size() == 14, "power frame reassembly");
}

// 测试CDC单读者分流；输入任意拆分，验证终端及调参内容不丢失。
void UsbRouting() {
  const std::string input =
      "ref status\r\nq00=0.01#r11=2#\nref plot kalman 2\r";
  for (size_t split = 0; split <= input.size(); ++split) {
    frame_cdc::UsbInputRouter router;
    std::string console;
    std::vector<std::string> tune;
    const auto output = [&console](const uint8_t* data, size_t size) {
      console.append(reinterpret_cast<const char*>(data), size);
    };
    const auto parameter = [&tune](const uint8_t* data, size_t size) {
      tune.emplace_back(reinterpret_cast<const char*>(data), size);
    };
    router.Feed(reinterpret_cast<const uint8_t*>(input.data()), split, output,
                parameter);
    router.Feed(reinterpret_cast<const uint8_t*>(input.data()) + split,
                input.size() - split, output, parameter);
    Check(console == "ref status\r\n\nref plot kalman 2\r" &&
              tune.size() == 2 && tune[0] == "q00=0.01#" && tune[1] == "r11=2#",
          "CDC split routing");
    const std::string large = "q11=" + std::string(200, '1') + "#q00=3#";
    router.Feed(reinterpret_cast<const uint8_t*>(large.data()), large.size(),
                output, parameter);
    Check(tune.size() == 3 && tune.back() == "q00=3#", "CDC oversize recovery");
    const std::string cancelled = "q00=broken\nref status\r";
    router.Feed(reinterpret_cast<const uint8_t*>(cancelled.data()),
                cancelled.size(), output, parameter);
    Check(tune.size() == 3 && console.ends_with("\nref status\r"),
          "CDC incomplete tuning cancelled by newline");
  }
}

// 测试SPI完整时钟与片选生命周期；无输入。
void Spi() {
  GPIO_TypeDef cs;
  LibXR::SPI backend;
  backend.cs = &cs;
  Struct_SPI_Manage_Object driver;
  driver.Bind(backend);
  driver.Tx_Buffer[0] = 0x80;
  bool callback = false;
  driver.on_receive = LibXR::Callback<>::Create(
      [](bool in_isr, bool* value) {
        Check(!in_isr, "completion in worker");
        *value = true;
      },
      &callback);
  Check(driver.Transfer(&cs, 1, GPIO_PIN_RESET, 1, 6) == HAL_OK,
        "transaction result");
  Check(backend.sent.size() == 7 && backend.sent[0] == 0x80 &&
            backend.sent[1] == 0,
        "command plus dummy clocks");
  Check(backend.cs_was_active && cs.state == GPIO_PIN_SET && callback,
        "CS and completion");
  callback = false;
  backend.result = LibXR::ErrorCode::BUSY;
  Check(driver.Transfer(&cs, 1, GPIO_PIN_RESET, 1, 6) != HAL_OK && !callback &&
            cs.state == GPIO_PIN_SET,
        "failure deasserts CS");
  backend.result = LibXR::ErrorCode::OK;
  Check(driver.faulted() &&
            driver.Transfer(&cs, 1, GPIO_PIN_RESET, 1, 6) == HAL_ERROR,
        "failed backend quarantined against late DMA reuse");
  Check(driver.Transfer(&cs, 1, GPIO_PIN_RESET, 500, 13) == HAL_ERROR,
        "combined length bound");
}

// 测试电压接口和复用看门狗；无输入。
void AdcWatchdog() {
  LibXR::ADC adc;
  Struct_ADC_Manage_Object driver{&adc};
  Check(driver.ReadVoltage() == 1.5f, "ADC returns volts");
  LibXR::Watchdog watchdog;
  IWDG_Independent_Feed(watchdog);
  Check(watchdog.feeds == 1, "existing watchdog feed");
}

// 用状态化Flash替身验证实际驱动源码；无输入，不接触硬件。
void Flash() {
  OSPI_HandleTypeDef backend;
  Struct_OSPI_Manage_Object driver;
  driver.handle = &backend;
  Class_W25Q64JV flash;
  Check(flash.Init(&driver) && flash.jedec_id() == 0xef4017,
        "JEDEC validation");
  const uint8_t bytes[8] = {0, 1, 2, 3, 4, 5, 6, 7};
  Check(flash.Set_Buffer(bytes, 252, 8), "cross page write");
  Check(
      flash.Get_Buffer(252, 8) && std::memcmp(driver.Rx_Buffer, bytes, 8) == 0,
      "readback");
  size_t programs = 0;
  for (const auto& command : backend.commands) {
    if (command.Instruction == 0x02) {
      ++programs;
      Check(command.NbData == 4, "split at page boundary");
    }
  }
  Check(programs == 2 && !flash.Set_Sector_Erased(1),
        "page and alignment semantics");
  Check(flash.EnableQuad() && backend.sr2 == 2, "QE preserves status protocol");
  Check(flash.SetMemoryMapped(true) && !flash.Get_Buffer(0, 1) &&
            flash.SetMemoryMapped(false),
        "mapping ownership");
  Check(flash.Set_Sector_Erased(0) && backend.memory[252] == 0xff,
        "sector erase model");
  backend.result = HAL_TIMEOUT;
  const auto receives = backend.receives;
  Check(!flash.Get_Buffer(0, 1) && backend.receives == receives,
        "failed command prevents receive");
  Check(!flash.Get_Buffer(8 * 1024 * 1024, 1), "flash capacity bound");
}

// 测试参考绘图类的真实编码及有界命令解析；无输入。
void Plotters() {
  LibXR::ReadPort read;
  LibXR::WritePort write;
  LibXR::UART backend{&read, &write};
  Struct_UART_Manage_Object driver;
  driver.Bind(backend);
  const char* names[] = {"gain", "offset"};
  Class_Vofa_UART vofa{};
  vofa.Init(&driver, 2, names);
  const float value = 1.25f;
  vofa.Set_Data(1, &value);
  vofa.TIM_1ms_Write_PeriodElapsedCallback();
  Check(write.bytes.size() == 8 &&
            std::memcmp(write.bytes.data(), &value, 4) == 0,
        "VOFA pointer width and float encoding");
  const char* command = "offset=-2.5#";
  vofa.UART_RxCpltCallback(reinterpret_cast<const uint8_t*>(command),
                           std::strlen(command));
  Check(vofa.Get_Variable_Index() == 1 && vofa.Get_Variable_Value() == -2.5f,
        "real pointer array and supplied command frame");
  const char* invalid = "gain=nan#";
  vofa.UART_RxCpltCallback(reinterpret_cast<const uint8_t*>(invalid),
                           std::strlen(invalid));
  Check(vofa.Get_Variable_Index() == -1, "invalid physical parameter rejected");
  const std::string oversized(200, 'x');
  vofa.UART_RxCpltCallback(reinterpret_cast<const uint8_t*>(oversized.data()),
                           oversized.size());
  Check(vofa.Get_Variable_Index() == -1, "long variable name bounded");
  Class_Serialplot_UART serial{};
  serial.Init(&driver, Serialplot_Checksum_8_ENABLE, 2, names);
  serial.Set_Data(1, &value);
  serial.TIM_1ms_Write_PeriodElapsedCallback();
  Check(write.bytes.size() == 6 && write.bytes[0] == 0xab,
        "Serialplot framing");
}

// 核对真实DM/DJI类与总线适配；输入固定协议样本，输出帧/失联断言。
void Motors() {
  LibXR::CAN backend;
  Struct_CAN_Manage_Object bus;
  Check(bus.Bind(backend), "motor bus bind");
  Class_Motor_DM_Normal dm{};
  dm.Init(&bus, 0, 1, Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA);
  dm.CAN_Send_Enter();
  Check(backend.last.id == 0x101 && backend.last.data[7] == 0xfc,
        "DM mode offset/enable");
  dm.CAN_Send_Exit();
  Check(backend.last.data[7] == 0xfd, "DM disable protocol");
  bus.Rx_Header.DataLength = 8;
  bus.Rx_Buffer[0] = 0x11;  // ID1，反馈控制状态ENABLE。
  dm.CAN_RxCpltCallback();
  dm.Set_Control_Angle(1.25f);
  dm.Set_Control_Omega(-2.0f);
  dm.TIM_Send_PeriodElapsedCallback();
  float encoded[2]{};
  std::memcpy(encoded, backend.last.data, sizeof(encoded));
  Check(encoded[0] == 1.25f && encoded[1] == -2.0f,
        "DM float wire representation");
  Class_Motor_DJI_GM6020 gm{};
  gm.Init(&bus, Motor_DJI_ID_0x206, Motor_DJI_Control_Method_TORQUE);
  gm.Set_Target_Torque(0.1f);
  gm.TIM_Calculate_PeriodElapsedCallback();
  Check(bus.Group(0x1fe)[2] != 0 || bus.Group(0x1fe)[3] != 0,
        "DJI shared group slot");
  bus.Rx_Header.DataLength = 7;
  gm.CAN_RxCpltCallback();
  gm.TIM_100ms_Alive_PeriodElapsedCallback();
  Check(gm.Get_Status() == Motor_DJI_Status_DISABLE, "short feedback ignored");
  bus.Rx_Header.DataLength = 8;
  const uint8_t feedback[8] = {0x10, 0, 0, 60, 0, 0, 25, 0};
  std::memcpy(bus.Rx_Buffer, feedback, sizeof(feedback));
  gm.CAN_RxCpltCallback();
  gm.TIM_100ms_Alive_PeriodElapsedCallback();
  Check(gm.Get_Status() == Motor_DJI_Status_ENABLE &&
            std::isfinite(gm.Get_Now_Omega()),
        "full feedback decoded and observed");
  gm.TIM_100ms_Alive_PeriodElapsedCallback();
  Check(gm.Get_Status() == Motor_DJI_Status_DISABLE,
        "loss of feedback detected");
}

// 运行选中的行为测试；输入用例名，输出进程状态。
int main(int argc, char** argv) {
  try {
    const std::string_view name = argc == 2 ? argv[1] : "";
    if (name == "can") {
      Can();
    } else if (name == "uart") {
      Uart();
    } else if (name == "framing") {
      Framing();
    } else if (name == "spi") {
      Spi();
    } else if (name == "adc_watchdog") {
      AdcWatchdog();
    } else if (name == "flash") {
      Flash();
    } else if (name == "plotters") {
      Plotters();
    } else if (name == "usb_routing") {
      UsbRouting();
    } else if (name == "motors") {
      Motors();
    } else {
      return 2;
    }
    std::puts("PASS");
    return 0;
  } catch (const std::exception& error) {
    std::fprintf(stderr, "%s\n", error.what());
    return 1;
  }
}
