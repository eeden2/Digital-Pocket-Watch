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
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"



// 4MB Flash SPI
#define SPICLK 25
#define SPID 26
#define SPIQ 21
#define SPICS0 20

// LCD SPI Pins
#define FSPI_CLK 12
#define FSPID 13
#define FSPIQ 8
#define FSPICS5 34
#define LCD_RST 17
#define LCD_BL 27
#define D_C 16

// I2C Touch Panel
#define I2XEXT0_SCL_in 31
#define I2CEXT0_SDA_in 32
#define TP_INT 35
#define TP_RST 36

//Antenna
#define ANT 1

//USB Diffs
#define D_PLUS 19
#define D_NEG 18

//Flash Switch for when Chip is in download mode or regular SPI mode
#define FIRST_SWITCH 14
#define SECOND_SWITCH 15

void app_main(void)
{
    
    esp_flash_t flash;
    esp_flash_spi_device_config_t properties;
    properties.io_mode = SPI_FLASH_FASTRD;
    properties.freq_mhz = ESP_FLASH_80MHZ;


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
