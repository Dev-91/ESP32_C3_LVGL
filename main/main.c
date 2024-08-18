#include "driver/spi_master.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "esp_lvgl_port.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#define SCLK 4
#define MOSI 3
#define DC   1
#define CS   -1
#define RST  2
#define BK   0

#define LCD_HOST SPI2_HOST
#define LCD_SPI_MODE 3
#define LCD_PIXEL_CLOCK_HZ 40 * 1000 * 1000
#define LCD_CMD_BITS 8
#define LCD_PARAM_BITS 8

#define LCD_H_RES 240
#define LCD_V_RES 240

#define LVGL_TICK_PERIOD_MS    2
#define LVGL_TASK_MAX_DELAY_MS 500
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE   (4 * 1024)
#define LVGL_TASK_PRIORITY     2

#define LCD_DRAW_BUFF_DOUBLE (1)
#define LCD_DRAW_BUFF_HEIGHT (50)

#define TAG "MAIN"

static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;

/* LVGL display and touch */
static lv_disp_t *lvgl_disp = NULL;

// static SemaphoreHandle_t lvgl_mux = NULL;

void lcd_panel_init()
{
    spi_bus_config_t buscfg = {
        .sclk_io_num = SCLK,
        .mosi_io_num = MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1, // Quad SPI LCD driver is not yet supported
        .quadhd_io_num = -1, // Quad SPI LCD driver is not yet supported
        .max_transfer_sz = LCD_H_RES * 80 * sizeof(uint16_t), // transfer 80 lines of pixels (assume pixel is RGB565) at most in one SPI transaction
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO)); // Enable the DMA feature

    // esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = DC,
        .cs_gpio_num = CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = LCD_SPI_MODE,
        .trans_queue_depth = 10,
    };
    // Attach the LCD to the SPI bus
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &lcd_io));

    // esp_lcd_panel_handle_t lcd_panel = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };
    // Create LCD panel handle for ST7789, with the SPI IO device handle
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(lcd_io, &panel_config, &lcd_panel));

    esp_lcd_panel_reset(lcd_panel);
    esp_lcd_panel_init(lcd_panel);

    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(lcd_panel, true));
    
    // turn on the display
    esp_lcd_panel_disp_on_off(lcd_panel, true);
}

static esp_err_t app_lvgl_init(void)
{
    /* Initialize LVGL */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,         /* LVGL task priority */
        .task_stack = 4096*2,         /* LVGL task stack size */
        .task_affinity = PRO_CPU_NUM,        /* LVGL task pinned to core (-1 is no affinity) */
        .task_max_sleep_ms = 500,   /* Maximum sleep in LVGL task */
        .timer_period_ms = 5        /* LVGL timer tick period in ms */
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* Add LCD screen */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = lcd_io,
        .panel_handle = lcd_panel,
        .buffer_size = LCD_H_RES * LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
        .double_buffer = LCD_DRAW_BUFF_DOUBLE,
        .hres = LCD_H_RES,
        .vres = LCD_V_RES,
        .monochrome = false,
        /* Rotation values must be same as used in esp_lcd for initial settings of the screen */
        .rotation = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .flags = {
            .buff_dma = true,
        }
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    return ESP_OK;
}

static void app_main_display(void)
{
    lv_obj_t *scr = lv_scr_act();

    /* Task lock */
    lvgl_port_lock(0);

    /* Your LVGL objects code here .... */
    lv_obj_set_style_bg_color(scr, lv_color_make(0xFF, 0xFF, 0xFF), LV_PART_MAIN);

    /* Label */
    lv_obj_t *label = lv_label_create(scr);
    // lv_label_set_recolor(label, true);
    
    lv_obj_set_width(label, LCD_H_RES);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_color(scr, lv_color_black(), LV_PART_MAIN);
    lv_label_set_text(label, "WOW :D");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

    /* Task unlock */
    lvgl_port_unlock();
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize ST7789");
    lcd_panel_init();

    ESP_LOGI(TAG, "Initialize LVGL");
    app_lvgl_init();

    app_main_display();
}