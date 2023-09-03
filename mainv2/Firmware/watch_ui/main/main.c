#include <stdio.h>
#include <lvgl.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_timer.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "hal/lcd_types.h"
#include "gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "protocols/I2C.c"
#include "protocols/SPI.c"
#include "protocols/WIFI.c"
#include "signal.h"
#include "spi_master.h"
#include "ui/ui.c"
#include "protocols/timer_replacement.h" // This file was made due to the esp_timer.h not allowing use of its defined functions.

#define FSPIQ_IN_IDX                  64
#define FSPIQ_OUT_IDX                 64
#define FSPID_IN_IDX                  65
#define FSPID_OUT_IDX                 65
#define FSPICS0_IN_IDX                68
#define FSPICS0_OUT_IDX               68
#define LCD_HOST SPI2_HOST

#define portTICK_PERIOD_MS      1                       //This was defined manually because my IDE was not reading it from "sdkconfig.h"

#define LVGL_TICK_PERIOD_MS     2

//Antenna
#define ANT                     1

//USB Diffs
#define D_PLUS                  GPIO_NUM_13
#define D_NEG                   GPIO_NUM_12

//Flash Switch for when Chip is in download mode or regular SPI mode
#define FIRST_SWITCH            GPIO_NUM_8
#define SECOND_SWITCH           GPIO_NUM_9


extern void lvgl_demo_ui(lv_disp_t *disp);


static bool notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
    lv_disp_flush_ready(disp_driver);
    return false;
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
    // copy a buffer's content to a specific area of the display
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
}

/* Rotate display and touch, when rotated screen in LVGL. Called when driver parameters are updated. */
static void lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;

    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, true, false);

        break;
    case LV_DISP_ROT_90:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, true, true);

        break;
    case LV_DISP_ROT_180:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, false);
        esp_lcd_panel_mirror(panel_handle, false, true);

        break;
    case LV_DISP_ROT_270:
        // Rotate LCD display
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, false);

        break;
    }
}

static void increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}



void app_main(void)
{
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
    gpio_iomux_in(GPIO_NUM_6, FSPID_IN_IDX);
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
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(FSPIQ, &buscfg, SPI_DMA_CH_AUTO));
    
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

    /*need to initialize the i2c interface for the
    touch panel*/

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG,"Initialized I2C.");


    ESP_LOGI(TAG, "Initialize LVGL Library");
    lv_init();

    lv_color_t *buf1 = heap_caps_malloc(LCD_H_RES* 20 * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    assert(buf1);
    lv_color_t *buf2 = heap_caps_malloc(LCD_H_RES * 20 * sizeof(lv_color_t), MALLOC_CAP_INTERNAL);
    assert(buf2);

    //start LVGL Drawing buffers.
    lv_disp_set_draw_buffers(&disp_buf, buf1, buf2, LCD_H_RES*20);

    ESP_LOGI(TAG, "Register Display Driver to LVGL.");
    //lv_disp_set_driver_data(&disp_drv, NULL);
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.drv_update_cb = lvgl_port_update_callback;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = handle_my_panel;
    
    
    //This is the meat of what starts the squarline interface UI.
    lv_disp_t *disp = lv_disp_set_driver_data(&disp_drv,NULL);
    lv_theme_t * theme = lv_theme_basic_init(disp);
    lv_disp_set_theme(disp, theme);
    ui_watch_analog_screen_init();
    ui_camera_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_obj_add_event_cb(ui____initial_actions0,ui_event____initial_actions0,LV_EVENT_ALL, NULL);

    lv_disp_load_scr(ui____initial_actions0);
    lv_disp_load_scr(ui_watch_analog);
    //This is the end of the meat for starting the UI.


    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &increase_lvgl_tick,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Start example GUI");
    lvgl_demo_ui(disp);
    


    printf("Hello world!\n");
    
    

    while (1) {
        // raise the task priority of LVGL and/or reduce the handler period can improve the performance
        vTaskDelay(pdMS_TO_TICKS(10));
        // The task running lv_timer_handler should have lower priority than that running `lv_tick_inc`
        lv_timer_handler();
    }
}