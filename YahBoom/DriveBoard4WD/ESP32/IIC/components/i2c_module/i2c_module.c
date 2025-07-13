#include "i2c_module.h"
#include "esp_log.h"  // 日志库，用于调试 logging library for debugging

// 定义日志标签，用于输出调试信息 Define log tag for debugging output
static const char *TAG = "i2c_module";

// 初始化I2C接口 Initialize I2C interface
esp_err_t i2c_module_init(void)
{
    // 定义I2C配置结构体 Define I2C configuration structure
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,         // 设置为主机模式 Set as master mode
        .sda_io_num = I2C_MASTER_SDA_IO, // SDA引脚 GPIO21 SDA pin GPIO21
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // 开启SDA上拉电阻 Enable SDA pull-up resistor
        .scl_io_num = I2C_MASTER_SCL_IO, // SCL引脚 GPIO22 SCL pin GPIO22
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // 开启SCL上拉电阻 Enable SCL pull-up resistor
        .master.clk_speed = I2C_MASTER_FREQ_HZ, // 设置I2C时钟频率 Set I2C clock frequency
    };

    // 应用I2C参数配置 Apply I2C parameter configuration
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }

    // 安装I2C驱动 Install I2C driver
    err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
    }

    return err;
}

// I2C写数据 Write data to I2C device
esp_err_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    // 创建I2C命令队列 Create I2C command queue
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // 开始信号 Start signal
    i2c_master_start(cmd);

    // 发送设备地址和写位 (addr << 1) | 0 表示写 Send device address + write bit
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);

    // 发送寄存器地址 Send register address
    i2c_master_write_byte(cmd, reg, true);

    // 发送数据 Send data
    i2c_master_write(cmd, data, len, true);

    // 停止信号 Stop signal
    i2c_master_stop(cmd);

    // 执行I2C命令 Execute I2C command
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    // 删除I2C命令队列 Delete I2C command queue
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(err));
    }

    return err;
}

// I2C读数据 Read data from I2C device
esp_err_t i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    // 创建I2C命令队列 Create I2C command queue
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    // 开始信号 Start signal
    i2c_master_start(cmd);

    // 发送设备地址和写位 Send device address + write bit
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);

    // 发送寄存器地址 Send register address
    i2c_master_write_byte(cmd, reg, true);

    // 重新开始信号 Repeated start signal
    i2c_master_start(cmd);

    // 发送设备地址和读位 Send device address + read bit
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);

    // 读取数据 Read data
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK); // 读前n-1字节，每个字节后应答ACK Read n-1 bytes with ACK
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK); // 最后1字节后发送NACK Send NACK after the last byte

    // 停止信号 Stop signal
    i2c_master_stop(cmd);

    // 执行I2C命令 Execute I2C command
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    // 删除I2C命令队列 Delete I2C command queue
    i2c_cmd_link_delete(cmd);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(err));
    }

    return err;
}
