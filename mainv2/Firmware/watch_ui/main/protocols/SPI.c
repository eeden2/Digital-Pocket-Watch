#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_err.h"


// 4MB Flash SPI
#define SPICLK                  25
#define SPID                    26
#define SPIQ                    21
#define SPICS0                  20

// LCD SPI Pins
#define FSPI_CLK                GPIO_NUM_6
#define FSPID                   GPIO_NUM_7
#define FSPIQ                   GPIO_NUM_2
#define FSPICS0                 GPIO_NUM_16
#define LCD_RST                 GPIO_NUM_15
#define LCD_BL                  GPIO_NUM_11
#define D_C                     GPIO_NUM_10

//LCD Properties
#define LCD_PIXEL_CLOCK_HZ      (20 * 1000 * 1000)
#define LCD_BK_LIGHT_ON_LEVEL   1
#define LCD_BK_LIGHT_OFF_LEVEL  !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL

//Pixel Resolutions
#define LCD_H_RES               240
#define LCD_V_RES               240

// Bit number used to represent command and parameter
#define LCD_CMD_BITS            8
#define LCD_PARAM_BITS          8

#define portTICK_PERIOD_MS      1                       //This was defined manually because my IDE was not reading it from "sdkconfig.h"

void printChipInfo(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s%s, ",
           CONFIG_IDF_TARGET_ESP32,
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
}

