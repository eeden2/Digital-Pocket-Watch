/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_rom_gpio.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "gpio_sig_map.h"
#include "driver/spi_master.h"
#include "hal/lcd_types.h"
#include "esp_lcd_panel_io.h"
#include "hal/lcd_hal.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "lvgl.h"


// 4MB Flash SPI
#define SPICLK 25
#define SPID 26
#define SPIQ 21
#define SPICS0 20

// LCD SPI Pins
#define FSPI_CLK GPIO_NUM_6
#define FSPID GPIO_NUM_7
#define FSPIQ GPIO_NUM_2
#define FSPICS0 GPIO_NUM_16
#define LCD_RST GPIO_NUM_15
#define LCD_BL GPIO_NUM_11
#define D_C GPIO_NUM_10

//LCD Properties
#define LCD_PIXEL_CLOCK_HZ     (20 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL  1
#define LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL

//Pixel Resolutions
#define LCD_H_RES 240
#define LCD_V_RES 240

// Bit number used to represent command and parameter
#define LCD_CMD_BITS           8
#define LCD_PARAM_BITS         8

#define LVGL_TICK_PERIOD_MS    2

// I2C Touch Panel
#define I2XEXT0_SCL_in GPIO_NUM_18
#define I2CEXT0_SDA_in GPIO_NUM_19
#define TP_INT GPIO_NUM_22
#define TP_RST GPIO_NUM_23

//Antenna
#define ANT 1

//USB Diffs
#define D_PLUS GPIO_NUM_13
#define D_NEG GPIO_NUM_12

//Flash Switch for when Chip is in download mode or regular SPI mode
#define FIRST_SWITCH GPIO_NUM_8
#define SECOND_SWITCH GPIO_NUM_9

#define LCD_HOST SPI

static const char *TAG = "example";

static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}



void app_main(void)
{
    
    esp_flash_t flash;
    esp_flash_spi_device_config_t properties;
    properties.io_mode = SPI_FLASH_FASTRD;
    properties.freq_mhz = ESP_FLASH_40MHZ;

    static lv_disp_draw_buf_t disp_buf;
    static lv_disp_drv_t disp_drv;

    ESP_LOGI(TAG, "Turn off LCD backlight.");
    gpio_config_t bk_gpio_config = 
    {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_BL
    };

    //Setting GPIO pins to alternate fucntions via the GPIO Matrix
    gpio_reset_pin(GPIO_NUM_6);
    gpio_iomux_in(GPIO_NUM_6, FSPICLK_OUT_IDX);
    gpio_reset_pin(GPIO_NUM_7);
    gpio_iomux_in(GPIO_NUM_7, FSPID_OUT_IDX);
    gpio_reset_pin(GPIO_NUM_2);
    gpio_iomux_in(GPIO_NUM_2, FSPIQ_IN_IDX);
    gpio_reset_pin(GPIO_NUM_21);
    gpio_iomux_in(GPIO_NUM_21, FSPICS0_OUT_IDX);
    
    //Now, the FSPI Pins will be configured to then be implemented in the LCD API From ESP-IDF
    spi_bus_config_t buscfg = 
    {
        .sclk_io_num = GPIO_NUM_6,
        .mosi_io_num = GPIO_NUM_7,
        .miso_io_num = GPIO_NUM_2,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));
    
    //After initializing the SPI Bus config, we now can config the SPI interface with the LCD API
    esp_lcd_panel_io_handle_t panel_IO = NULL;
    esp_lcd_panel_io_spi_config_t config_IO = 
    {
        .dc_gpio_num = D_C,
        .cs_gpio_num = FSPICS0,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
        .on_color_trans_done = notify_lvgl_flush_ready,
        .user_ctx = &disp_drv,
    };

    //Checking for Errors before adding LCD to SPI Bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,&config_IO,&panel_IO));

    esp_lcd_panel_handle_t handle_my_panel = NULL;
    esp_lcd_panel_dev_config_t config_my_panel = 
    {
        .reset_gpio_num = LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16,
    };

    //Install that which handles my panel :)
    ESP_LOGI(TAG, "Installing the Panel Handler.");
    ESP_ERROR_CHECK(esp_lcd_new_panel_gc9a01(panel_IO, &config_my_panel,&handle_my_panel));
    
    //Initialize the Panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(handle_my_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(handle_my_panel));

    //The colors must be inverted due to the config following a BGR Endian
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(handle_my_panel, true));

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(handle_my_panel, true));

    ESP_LCD_TOUCH

    printf("Hello world!\n");
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}