#include "driver/spi_master.h"
#include "driver/i2c.h"

#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"

#include "esp_lvgl_port.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"

#include "ui/ui.h"
#include "ui/ui_events.h"

#include "wifi_d9.h"
#include "ntp_d9.h"

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
#define LCD_DRAW_BUFF_HEIGHT (30)

#define I2C_MASTER_SCL_IO           5
#define I2C_MASTER_SDA_IO           6
#define I2C_MASTER_NUM              0
#define I2C_MASTER_FREQ_HZ          400000
#define BME280_SENSOR_ADDR          0x76

#define WIFI_SSID "D9_2.4G"
#define WIFI_PASS "92842853"

static esp_lcd_panel_io_handle_t lcd_io = NULL;
static esp_lcd_panel_handle_t lcd_panel = NULL;

/* LVGL display and touch */
static lv_disp_t *lvgl_disp = NULL;

// BME280 보정 데이터
uint16_t dig_T1;
int16_t dig_T2, dig_T3;
uint8_t dig_H1, dig_H3;
int16_t dig_H2, dig_H4, dig_H5;
int8_t dig_H6;
int32_t t_fine;

#define TAG "MAIN"

static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) return err;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static esp_err_t bme280_read_registers(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(1000));
}

static esp_err_t bme280_write_register(uint8_t reg, uint8_t value)
{
    uint8_t write_buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM, BME280_SENSOR_ADDR, write_buf, sizeof(write_buf), pdMS_TO_TICKS(1000));
}

static void bme280_read_calibration_data(void)
{
    uint8_t data[26];
    bme280_read_registers(0x88, data, 6);
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];

    bme280_read_registers(0xA1, &dig_H1, 1);
    bme280_read_registers(0xE1, data, 7);
    dig_H2 = (data[1] << 8) | data[0];
    dig_H3 = data[2];
    dig_H4 = (data[3] << 4) | (data[4] & 0x0F);
    dig_H5 = (data[5] << 4) | (data[4] >> 4);
    dig_H6 = data[6];
}

static float bme280_read_temperature(void)
{
    uint8_t data[3];
    bme280_read_registers(0xFA, data, 3);
    int32_t adc_T = (data[0] << 16) | (data[1] << 8) | data[2];
    adc_T >>= 4;

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
    t_fine = var1 + var2;
    float T = (t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

static float bme280_read_humidity(void)
{
    uint8_t data[2];
    bme280_read_registers(0xFD, data, 2);
    int32_t adc_H = (data[0] << 8) | data[1];

    int32_t v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
                   ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                   (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                   ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    float h = (v_x1_u32r >> 12);
    return h / 1024.0f;
}

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

void app_main(void)
{
    ESP_LOGI(TAG, "Initialize WiFi");
    // wifi_net_init();
    // wifi_init_sta();
    // wifi_connect(WIFI_SSID, WIFI_PASS);

    // if (wifi_status_check(1000) == 0) {
    //     ntp_set_time();
    // }

    ESP_LOGI(TAG, "Initialize ST7789");
    lcd_panel_init();

    ESP_LOGI(TAG, "Initialize LVGL");
    app_lvgl_init();

    // app_main_display();
    ESP_LOGI(TAG, "UI init");
    ui_init();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    bme280_read_calibration_data();
    
    // Configure BME280
    bme280_write_register(0xF2, 0x01);  // humidity oversampling x1
    bme280_write_register(0xF4, 0x27);  // temperature and pressure oversampling x1, normal mode

    while (1) {
        float temperature = bme280_read_temperature();
        float humidity = bme280_read_humidity();

        ESP_LOGI(TAG, "Temperature: %.2f °C, Humidity: %.2f %%", temperature, humidity);

        char *temp_buf = calloc(20, sizeof(char));
        char *humi_buf = calloc(20, sizeof(char));
        
        sprintf(temp_buf, "%.2f °C", temperature);
        sprintf(humi_buf, "%.2f %%", humidity);
        
        lv_label_set_text(ui_TempLabel, temp_buf);
        lv_label_set_text(ui_HumiLabel, humi_buf);
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}