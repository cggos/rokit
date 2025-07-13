#ifndef __I2C_MODULE_H__
#define __I2C_MODULE_H__

#include "driver/i2c.h"
#include "esp_err.h"
#include <stdint.h>

// ����I2C��SCL��SDA���� Define I2C SCL and SDA pins
#define I2C_MASTER_SCL_IO 37  // SCL
#define I2C_MASTER_SDA_IO 38  // SDA

// ʹ��I2C�ӿ�0 Use I2C port 0
#define I2C_MASTER_NUM I2C_NUM_0

// I2Cͨ��ʱ��Ƶ�ʣ�400kHz  I2C clock frequency, 400kHz
#define I2C_MASTER_FREQ_HZ 400000

// I2Cͨ�ų�ʱʱ�䣬��λ������ I2C communication timeout, in milliseconds
#define I2C_MASTER_TIMEOUT_MS 1000

esp_err_t i2c_module_init(void);
esp_err_t i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
esp_err_t i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif
