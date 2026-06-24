#include "touch.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_touch_xpt2046.h"
#include "esp_lvgl_port.h"

#define T_SCLK 25
#define T_MOSI 32
#define T_MISO 39
#define T_CS   33
#define T_IRQ  36
#define LCD_H_LANDSCAPE 320
#define LCD_V_LANDSCAPE 240

void touch_init(lv_display_t* disp) {
    spi_bus_config_t bus = {};
    bus.sclk_io_num = T_SCLK;
    bus.mosi_io_num = T_MOSI;
    bus.miso_io_num = T_MISO;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 0;
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t tio = NULL;
    // The atanisoft macro doesn't initialise all fields on IDF >=6; suppress
    // the -Werror=missing-field-initializers false-positive around it.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
    esp_lcd_panel_io_spi_config_t iocfg = ESP_LCD_TOUCH_IO_SPI_XPT2046_CONFIG(T_CS);
#pragma GCC diagnostic pop
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &iocfg, &tio));

    esp_lcd_touch_config_t tcfg = {};
    tcfg.x_max = LCD_H_LANDSCAPE;
    tcfg.y_max = LCD_V_LANDSCAPE;
    tcfg.rst_gpio_num = GPIO_NUM_NC;
    tcfg.int_gpio_num = (gpio_num_t)T_IRQ;
    tcfg.flags.swap_xy = 1;     // match the landscape display rotation
    tcfg.flags.mirror_x = 1;
    tcfg.flags.mirror_y = 0;
    esp_lcd_touch_handle_t tp = NULL;
    ESP_ERROR_CHECK(esp_lcd_touch_new_spi_xpt2046(tio, &tcfg, &tp));

    lvgl_port_touch_cfg_t pt = {};
    pt.disp = disp;
    pt.handle = tp;
    lvgl_port_add_touch(&pt);
}
