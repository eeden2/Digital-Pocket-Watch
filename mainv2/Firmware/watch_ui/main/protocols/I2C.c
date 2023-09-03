#include "driver/i2c.h"
#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

// I2C Touch Panel
#define I2CEXT0_SCL_in          GPIO_NUM_18
#define I2CEXT0_SDA_in          GPIO_NUM_19
#define TP_INT                  GPIO_NUM_22
#define TP_RST                  GPIO_NUM_23
#define I2C_MASTER_NUM          0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_TIMEOUT_MS   1000
#define CST816S_BASE_ADDRESS    0x15
#define CST816S_READ            0x2B
#define CST816S_WRITE           0x2A
#define portTICK_PERIOD_MS      1                       //This was defined manually because my IDE was not reading it from "sdkconfig.h"

/*
    Setting up the following method to make it more convenient to read from the CST816S registers.
*/
static esp_err_t cst816s_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM,CST816S_BASE_ADDRESS, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS);
}

/*
    This is the to make it convenient to write to the the CST816S registers.
*/

static esp_err_t cst816s_register_write_byte(uint8_t reg_addr,uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr,data};
    size_t w = sizeof(write_buf);
    ret = i2c_master_write_to_device(I2C_MASTER_NUM,CST816S_BASE_ADDRESS,write_buf,sizeof(write_buf),I2C_MASTER_TIMEOUT_MS/portTICK_PERIOD_MS);
    return ret;
}


/*
    This is to initialize and setup the configuration for the I2C Driver.
*/
static esp_err_t i2c_master_init(void)
{
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2CEXT0_SDA_in,
        .scl_io_num = I2CEXT0_SCL_in,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(I2C_MASTER_NUM, &i2c_config);

    return i2c_driver_install(I2C_MASTER_NUM,i2c_config.mode,0,0,0);
}
