#include "display.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lvgl_port.h"

#define PIN_SCLK 14
#define PIN_MOSI 13
#define PIN_MISO 12
#define PIN_CS   15
#define PIN_DC    2
#define PIN_RST  -1
#define PIN_BL   21
#define LCD_H 240   // native portrait width
#define LCD_V 320   // native portrait height

lv_display_t* display_init(void) {
    gpio_config_t bk = {};
    bk.mode = GPIO_MODE_OUTPUT;
    bk.pin_bit_mask = 1ULL << PIN_BL;
    gpio_config(&bk);
    gpio_set_level((gpio_num_t)PIN_BL, 1);   // backlight on

    spi_bus_config_t bus = {};
    bus.sclk_io_num = PIN_SCLK;
    bus.mosi_io_num = PIN_MOSI;
    bus.miso_io_num = PIN_MISO;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = LCD_H * 80 * sizeof(uint16_t);
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t iocfg = {};
    iocfg.dc_gpio_num = (gpio_num_t)PIN_DC;
    iocfg.cs_gpio_num = (gpio_num_t)PIN_CS;
    iocfg.pclk_hz = 20 * 1000 * 1000;
    iocfg.lcd_cmd_bits = 8;
    iocfg.lcd_param_bits = 8;
    iocfg.spi_mode = 0;
    iocfg.trans_queue_depth = 10;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &iocfg, &io));

    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t devcfg = {};
    devcfg.reset_gpio_num = (gpio_num_t)PIN_RST;
    devcfg.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
    devcfg.bits_per_pixel = 16;
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &devcfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, false));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    const lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ESP_ERROR_CHECK(lvgl_port_init(&lvgl_cfg));

    lvgl_port_display_cfg_t disp_cfg = {};
    disp_cfg.io_handle = io;
    disp_cfg.panel_handle = panel;
    disp_cfg.buffer_size = LCD_H * 40;
    disp_cfg.double_buffer = true;
    disp_cfg.hres = LCD_V;   // landscape: 320 pixels wide
    disp_cfg.vres = LCD_H;   // landscape: 240 pixels tall
    disp_cfg.rotation.swap_xy = true;    // landscape 320x240
    disp_cfg.rotation.mirror_x = true;
    disp_cfg.rotation.mirror_y = false;
    disp_cfg.flags.buff_dma = true;
    disp_cfg.flags.swap_bytes = true;    // RGB565 byte order for this panel
    return lvgl_port_add_disp(&disp_cfg);
}
