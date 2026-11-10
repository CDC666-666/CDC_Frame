/* BMI088 两颗芯片的寄存器协议、初始化和单位转换。 */
#include "bmi088.h"

#include <stddef.h>
#include <string.h>

#include "mc02_board.h"

enum {
    BMI088_REG_ID = 0x00,
    BMI088_ACC_ID = 0x1E,
    BMI088_GYRO_ID = 0x0F,
    BMI088_ACC_ERROR = 0x02,
    BMI088_ACC_STATUS = 0x03,
    BMI088_ACC_DATA = 0x12,
    BMI088_ACC_TEMP = 0x22,
    BMI088_ACC_CONF = 0x40,
    BMI088_ACC_RANGE = 0x41,
    BMI088_ACC_PWR_CONF = 0x7C,
    BMI088_ACC_PWR_CTRL = 0x7D,
    BMI088_ACC_RESET = 0x7E,
    BMI088_GYRO_DATA = 0x02,
    BMI088_GYRO_RANGE = 0x0F,
    BMI088_GYRO_BANDWIDTH = 0x10,
    BMI088_GYRO_LPM = 0x11,
    BMI088_GYRO_RESET = 0x14,
    BMI088_GYRO_INT_CTRL = 0x15,
    BMI088_GYRO_INT_IO = 0x16,
    BMI088_GYRO_INT_MAP = 0x18,
    BMI088_SOFT_RESET = 0xB6,
    BMI088_ACC_READY = 0x80,
    BMI088_READ_BIT = 0x80,
    BMI088_MAX_READ = BMI088_GYRO_DATA + 6,
};

typedef struct {
    uint8_t address;
    uint8_t value;
    uint8_t settle_ms;
} bmi088_register_setting_t;

static bool read_registers(mc02_board_spi_select_t select, uint8_t address, uint8_t *data,
                           size_t length)
{
    /* 加速度计地址后多返回一个 dummy，陀螺仪地址后直接返回数据。 */
    const size_t offset = select == MC02_BOARD_SPI_SELECT_0 ? 2U : 1U;
    const uint8_t tx[BMI088_MAX_READ + 2U] = {address | BMI088_READ_BIT};
    uint8_t rx[sizeof(tx)];
    if (!mc02_board_spi_exchange(select, tx, rx, offset + length)) {
        return false;
    }
    memcpy(data, rx + offset, length);
    return true;
}

static bool write_register(mc02_board_spi_select_t select, uint8_t address, uint8_t value)
{
    const uint8_t tx[] = {address, value};
    uint8_t rx[sizeof(tx)];
    return mc02_board_spi_exchange(select, tx, rx, sizeof(tx));
}

static bmi088_status_t check_id(mc02_board_spi_select_t select, uint8_t expected_id)
{
    uint8_t id;
    if (!read_registers(select, BMI088_REG_ID, &id, 1U)) {
        return BMI088_STATUS_BUS_ERROR;
    }
    return id == expected_id ? BMI088_STATUS_OK : BMI088_STATUS_BAD_ID;
}

static bmi088_status_t configure(mc02_board_spi_select_t select, uint8_t expected_id,
                                 uint8_t reset_register, const bmi088_register_setting_t *settings,
                                 size_t count)
{
    uint8_t discard;
    const bool is_accel = select == MC02_BOARD_SPI_SELECT_0;
    /* 加速度计需要一次 CS 上升沿切入 SPI，软复位后也重复该动作。 */
    if (is_accel && !read_registers(select, BMI088_REG_ID, &discard, 1U)) {
        return BMI088_STATUS_BUS_ERROR;
    }
    bmi088_status_t status = check_id(select, expected_id);
    if (status != BMI088_STATUS_OK) {
        return status;
    }
    if (!write_register(select, reset_register, BMI088_SOFT_RESET)) {
        return BMI088_STATUS_BUS_ERROR;
    }
    mc02_board_delay_ms(80U);
    if (is_accel && !read_registers(select, BMI088_REG_ID, &discard, 1U)) {
        return BMI088_STATUS_BUS_ERROR;
    }
    status = check_id(select, expected_id);
    if (status != BMI088_STATUS_OK) {
        return status;
    }

    for (size_t i = 0; i < count; ++i) {
        const bmi088_register_setting_t *setting = &settings[i];
        uint8_t value;
        if (!write_register(select, setting->address, setting->value)) {
            return BMI088_STATUS_BUS_ERROR;
        }
        mc02_board_delay_ms(setting->settle_ms);
        if (!read_registers(select, setting->address, &value, 1U)) {
            return BMI088_STATUS_BUS_ERROR;
        }
        if (value != setting->value) {
            return BMI088_STATUS_CONFIG_ERROR;
        }
    }
    return BMI088_STATUS_OK;
}

void bmi088_init(bmi088_t *device)
{
    const bmi088_register_setting_t accel_settings[] = {
        {BMI088_ACC_PWR_CONF, 0x00, 5},
        {BMI088_ACC_PWR_CTRL, 0x04, 50},
        {BMI088_ACC_CONF, 0xAB, 1},  /* Normal，800 Hz。 */
        {BMI088_ACC_RANGE, 0x00, 1}, /* ±3 g。 */
    };
    const bmi088_register_setting_t gyro_settings[] = {
        {BMI088_GYRO_LPM, 0x00, 30},      {BMI088_GYRO_RANGE, 0x00, 1}, /* ±2000 deg/s。 */
        {BMI088_GYRO_BANDWIDTH, 0x82, 1},                               /* 1000 Hz，116 Hz 带宽。 */
        {BMI088_GYRO_INT_IO, 0x00, 1},                                  /* INT3 推挽，低有效。 */
        {BMI088_GYRO_INT_MAP, 0x01, 1},                                 /* DRDY 映射到 INT3。 */
        {BMI088_GYRO_INT_CTRL, 0x80, 1},
    };

    *device = (bmi088_t){.sample = {.status = BMI088_STATUS_NOT_INITIALIZED}};
    mc02_board_delay_ms(80U);
    device->accel_init_status =
        configure(MC02_BOARD_SPI_SELECT_0, BMI088_ACC_ID, BMI088_ACC_RESET, accel_settings,
                  sizeof(accel_settings) / sizeof(accel_settings[0]));
    device->gyro_init_status =
        configure(MC02_BOARD_SPI_SELECT_1, BMI088_GYRO_ID, BMI088_GYRO_RESET, gyro_settings,
                  sizeof(gyro_settings) / sizeof(gyro_settings[0]));
}

static int32_t axis_counts(const uint8_t *data)
{
    const int32_t value = ((int32_t)data[1] << 8) | data[0];
    return value >= 32768 ? value - 65536 : value;
}

void bmi088_sample(bmi088_t *device, uint64_t time_us, uint32_t sequence)
{
    bmi088_sample_t *sample = &device->sample;
    sample->is_valid = false;
    if (device->accel_init_status != BMI088_STATUS_OK ||
        device->gyro_init_status != BMI088_STATUS_OK) {
        sample->status = BMI088_STATUS_NOT_INITIALIZED;
        return;
    }

    uint8_t accel_status[BMI088_ACC_STATUS + 1];
    uint8_t accel[6];
    uint8_t gyro[BMI088_GYRO_DATA + 6];
    uint8_t temperature[2];
    const bool is_gyro_ready = mc02_board_take_gyro_ready();
    if (!read_registers(MC02_BOARD_SPI_SELECT_0, BMI088_REG_ID, accel_status,
                        sizeof(accel_status)) ||
        !read_registers(MC02_BOARD_SPI_SELECT_0, BMI088_ACC_DATA, accel, sizeof(accel)) ||
        !read_registers(MC02_BOARD_SPI_SELECT_1, BMI088_REG_ID, gyro, sizeof(gyro)) ||
        !read_registers(MC02_BOARD_SPI_SELECT_0, BMI088_ACC_TEMP, temperature,
                        sizeof(temperature))) {
        sample->status = BMI088_STATUS_BUS_ERROR;
        return;
    }
    if (accel_status[BMI088_REG_ID] != BMI088_ACC_ID || gyro[BMI088_REG_ID] != BMI088_GYRO_ID) {
        sample->status = BMI088_STATUS_BAD_ID;
        return;
    }
    if ((accel_status[BMI088_ACC_ERROR] & 0x1DU) != 0U) {
        sample->status = BMI088_STATUS_CONFIG_ERROR;
        return;
    }
    if ((accel_status[BMI088_ACC_STATUS] & BMI088_ACC_READY) == 0U || !is_gyro_ready) {
        sample->status = BMI088_STATUS_NOT_READY;
        return;
    }

    const double accel_m_s2_per_count = 9.80665 / 10920.0;
    const double gyro_rad_s_per_count = (2000.0 * 0.017453292519943295) / 32768.0;
    for (size_t i = 0; i < 3; ++i) {
        sample->accel_sensor_m_s2[i] = axis_counts(&accel[2U * i]) * accel_m_s2_per_count;
        sample->gyro_sensor_rad_s[i] =
            axis_counts(&gyro[BMI088_GYRO_DATA + 2U * i]) * gyro_rad_s_per_count;
    }

    int32_t temp_counts = ((int32_t)temperature[0] << 3) | (temperature[1] >> 5);
    if (temp_counts >= 1024) {
        temp_counts -= 2048;
    }
    sample->temperature_deg_c = 23.0 + 0.125 * temp_counts;
    sample->time_us = time_us;
    sample->sequence = sequence;
    sample->status = BMI088_STATUS_OK;
    sample->is_valid = true;
}
