// Copyright 2015-2018 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "soc/gpio_struct.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "esp_freertos_hooks.h"
#include "esp_system.h"
#include "lcd.h"

#ifdef CONFIG_GUI_FRAMEWORK_LVGL
#include "lvgl.h"
#endif

//resolution of lcd
#define LCD_X_RESOLUTION CONFIG_LCD_X_RESOLUTION
#define LCD_Y_RESOLUTION CONFIG_LCD_Y_RESOLUTION

//spi gpio pin number
#define PIN_NUM_MISO CONFIG_LCD_SPI_MISO
#define PIN_NUM_MOSI CONFIG_LCD_SPI_MOSI
#define PIN_NUM_CLK CONFIG_LCD_SPI_SCLK
#define PIN_NUM_CS CONFIG_LCD_SPI_CS

#define PIN_NUM_DC CONFIG_LCD_SPI_DC
#define PIN_NUM_RST CONFIG_LCD_SPI_RST
#define PIN_NUM_BCKL CONFIG_LCD_SPI_BCKL

//speed of SCLK
#define SPI_CLK_SPEED_MHZ CONFIG_LCD_SPI_CLK_SPEED_MHZ

//spi host device
#define LCD_USE_HOST_DEVICE HSPI_HOST

//number of transactions for a whole lcd operation
#define LCD_SPI_COMMANDS_QUANTITY 6

//depth of queue used for spi transaction
#define SPI_TRANS_QUEUE_SIZE 7

//backlight operation
#define LCD_BCKL_ON() gpio_set_level(PIN_NUM_BCKL, 0)
#define LCD_BCKL_OFF() gpio_set_level(PIN_NUM_BCKL, 1)

//timer period
#define LVGL_TIMER_PERIOD_MS (10)

//framebuffer used to descript what is displayed on lcd
static fb_t gFrameBuffer;
//store a line of image
static color_t line[LCD_X_RESOLUTION];

#ifdef CONFIG_GUI_FRAMEWORK_LVGL
//timer used to periodically call lv_task_handler
static TimerHandle_t lvgl_timer;
#endif

//mutex for lcd to make the api thread safe
static SemaphoreHandle_t lcd_mutex;

//lcd command type define
typedef struct
{
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //Number of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

//command table for different lcd driver ic
#ifdef CONFIG_LCD_DRIVER_ST7789V
DRAM_ATTR static const lcd_init_cmd_t lcd_init_cmds[] =
    {
        {0x36, {(1 << 5) | (1 << 6)}, 1},
        {0x3A, {0x55}, 1},
        {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
        {0xB7, {0x45}, 1},
        {0xBB, {0x2B}, 1},
        {0xC0, {0x2C}, 1},
        {0xC2, {0x01, 0xff}, 2},
        {0xC3, {0x11}, 1},
        {0xC4, {0x20}, 1},
        {0xC6, {0x0f}, 1},
        {0xD0, {0xA4, 0xA1}, 1},
        {0xE0, {0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19}, 14},
        {0xE1, {0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19}, 14},
        {0x11, {0}, 0x80},
        {0x29, {0}, 0x80},
        {0, {0}, 0xff}};
#elif CONFIG_LCD_DRIVER_ILI9341
DRAM_ATTR static const lcd_init_cmd_t lcd_init_cmds[] = {
    {0xCF, {0x00, 0x83, 0x30}, 3},
    {0xED, {0x64, 0x03, 0x12, 0x81}, 4},
    {0xE8, {0x85, 0x01, 0x79}, 3},
    {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
    {0xF7, {0x20}, 1},
    {0xEA, {0x00, 0x00}, 2},
    {0xC0, {0x26}, 1},
    {0xC1, {0x11}, 1},
    {0xC5, {0x35, 0x3E}, 2},
    {0xC7, {0xBE}, 1},
    {0x36, {0x28}, 1},
    {0x3A, {0x55}, 1},
    {0xB1, {0x00, 0x1B}, 2},
    {0xF2, {0x08}, 1},
    {0x26, {0x01}, 1},
    {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
    {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
    {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
    {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
    {0x2C, {0}, 0},
    {0xB7, {0x07}, 1},
    {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
    {0x11, {0}, 0x80},
    {0x29, {0}, 0x80},
    {0, {0}, 0xff},
};
#endif

#ifdef CONFIG_GUI_FRAMEWORK_LVGL
//Call the lv_tick_inc function periodically and tell the call period in millisecinds
static void lv_tick_hook(void)
{
    lv_tick_inc(portTICK_PERIOD_MS);
}
#endif

//callback funtion called before a spi transfer
static void lcd_spi_pre_transfer_callback(spi_transaction_t *t)
{
    int dc = (int)(t->user);
    gpio_set_level(PIN_NUM_DC, dc);
}

//gpio specification initialise
static void gpio_spec_init(void)
{
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
}

//spi specification initialise
static spi_device_handle_t spi;
static void spi_spec_init(void)
{
    esp_err_t ret;
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 80000};
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLK_SPEED_MHZ * 1000 * 1000,
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = SPI_TRANS_QUEUE_SIZE,
        .pre_cb = lcd_spi_pre_transfer_callback,
    };
    ret = spi_bus_initialize(LCD_USE_HOST_DEVICE, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(LCD_USE_HOST_DEVICE, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

//send a command to lcd
static void lcd_cmd(const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8; //Command is 8 bits
    t.tx_buffer = &cmd;
    t.user = (void *)0; //D/C needs to be set to 0
    ret = spi_device_transmit(spi, &t);
    ESP_ERROR_CHECK(ret);
}

//send data to lcd
static void lcd_data(const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len > 0)
    {
        memset(&t, 0, sizeof(t));
        t.length = len * 8; //Len is in bytes, transaction length is in bits.
        t.tx_buffer = data;
        t.user = (void *)1; //D/C needs to be set to 1
        ret = spi_device_transmit(spi, &t);
        ESP_ERROR_CHECK(ret);
    }
}

//get x resolution of lcd
static uint32_t lcd_get_x_resolution()
{
    return LCD_X_RESOLUTION;
}

//get y resolution of lcd
static uint32_t lcd_get_y_resolution()
{
    return LCD_Y_RESOLUTION;
}

//before draw pixel to lcd, boundary problem should be fix
static bool lcd_fixBoundary(int *xpos, int *ypos, int *xpos2, int *ypos2)
{
    if (*xpos > *xpos2)
    {
        *xpos = (*xpos) ^ (*xpos2);
        *xpos2 = (*xpos) ^ (*xpos2);
        *xpos = (*xpos) ^ (*xpos2);
    }
    if (*ypos > *ypos2)
    {
        *ypos = (*ypos) ^ (*ypos2);
        *ypos2 = (*ypos) ^ (*ypos2);
        *ypos = (*ypos) ^ (*ypos2);
    }
    if ((*xpos2 < 0) || (*xpos >= LCD_X_RESOLUTION) || (*ypos2 < 0) || (*ypos >= LCD_Y_RESOLUTION))
        return false;
    if (*xpos < 0)
    {
        *xpos = 0;
    }
    if (*xpos2 > LCD_X_RESOLUTION)
    {
        *xpos2 = LCD_X_RESOLUTION;
    }
    if (*ypos < 0)
    {
        *ypos = 0;
    }
    if (*ypos2 > LCD_Y_RESOLUTION)
    {
        *ypos2 = LCD_Y_RESOLUTION;
    }
    return true;
}

//wait for the result of send a line
static void lcd_send_line_finish()
{
    spi_transaction_t *rtrans;
    esp_err_t ret;
    for (int x = 0; x < LCD_SPI_COMMANDS_QUANTITY; x++)
    {
        ret = spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        ESP_ERROR_CHECK(ret);
    }
}

//fill operation type define
typedef enum FILL_OPERATION
{
    ONE_COLOR_TO_LINE,
    ONE_PIXEL,
    IMAGE
} fillop_t;
//deal with three types of flush operation
static void lcd_fill_line_directly_to_screen(int xpos, int ypos, int xpos2, const color_t *line_ptr, fillop_t fillOp, color_t color)
{
    esp_err_t ret;
    uint16_t byteLength = 0;
    uint16_t bitLength = 0;
    int dx = xpos2 - xpos;
    uint8_t hi = color.byte1;
    uint8_t lo = color.byte0;
    color.color = (lo << 8) | (hi);
    switch (fillOp)
    {
    case ONE_COLOR_TO_LINE:
        byteLength = dx * sizeof(color_t);
        for (int i = 0; i < dx; i++)
        {
            line[i] = color;
        }
        break;
    case ONE_PIXEL:
        byteLength = sizeof(color_t);
        line[0] = color;
        break;
    case IMAGE:
        byteLength = sizeof(color_t) * dx;
        memcpy(line, line_ptr, byteLength);
        for (int i = 0; i < dx; i++)
        {
            color.color = line[i].color;
            hi = color.byte1;
            lo = color.byte0;
            line[i].color = (lo << 8) | (hi);
        }
        break;
    default:
        return;
    }
    bitLength = byteLength << 3;                               //byte to bit
    static spi_transaction_t trans[LCD_SPI_COMMANDS_QUANTITY]; //the tansaction here must be static
    for (int i = 0; i < LCD_SPI_COMMANDS_QUANTITY; i++)
    {
        memset(&trans[i], 0, sizeof(spi_transaction_t));
        if ((i & 1) == 0)
        {
            trans[i].length = 8;
            trans[i].user = (void *)(0);
        }
        else
        {
            trans[i].length = 8 * 4;
            trans[i].user = (void *)(1);
        }
        trans[i].flags = SPI_TRANS_USE_TXDATA;
    }
    trans[0].tx_data[0] = 0x2A;              // Column Address Set
    trans[1].tx_data[0] = xpos >> 8;         // Start Col High
    trans[1].tx_data[1] = xpos & 0xff;       // Start Col Low
    trans[1].tx_data[2] = xpos2 >> 8;        // End Col High
    trans[1].tx_data[3] = xpos2 & 0xff;      // End Col Low
    trans[2].tx_data[0] = 0x2B;              // Page address set
    trans[3].tx_data[0] = ypos >> 8;         // Start page high
    trans[3].tx_data[1] = ypos & 0xff;       // start page low
    trans[3].tx_data[2] = (ypos + 1) >> 8;   // end page high
    trans[3].tx_data[3] = (ypos + 1) & 0xff; // end page low
    trans[4].tx_data[0] = 0x2C;              // memory write
    trans[5].tx_buffer = line;               // data array to send via SPI
    trans[5].length = bitLength;             // data length, in bits
    trans[5].flags = 0;                      // undo SPI_TRANS_USE_TXDATA flag

    for (int i = 0; i < LCD_SPI_COMMANDS_QUANTITY; i++)
    {
        ret = spi_device_queue_trans(spi, &trans[i], portMAX_DELAY);
        ESP_ERROR_CHECK(ret);
    }
    lcd_send_line_finish();
}

//draw a line of image
static void lcd_put_image_line(int x, int y, int x2, color_t *line)
{
    color_t color;
    int y2 = y + 1;
    if (x == x2)
    {
        x2++;
    }
    if (lcd_fixBoundary(&x, &y, &x2, &y2) != false)
    {
        lcd_fill_line_directly_to_screen(x, y, x2, line, IMAGE, color);
    }
}

//draw a pixel
static void lcd_set_pixel(int x, int y, color_t color)
{
    int x2 = x + 1;
    int y2 = y + 1;
    if (lcd_fixBoundary(&x, &y, &x2, &y2) != false)
    {
        lcd_fill_line_directly_to_screen(x, y, x2, NULL, ONE_PIXEL, color);
    }
}

//draw a line of same color
static void lcd_fill_horiz_line(int x, int y, int x2, color_t color)
{
    int y2 = y + 1;
    if (x == x2)
    {
        x2++;
    }
    if (lcd_fixBoundary(&x, &y, &x2, &y2) != false)
    {
        lcd_fill_line_directly_to_screen(x, y, x2, NULL, ONE_COLOR_TO_LINE, color);
    }
}

//draw a rectangle of specific color
static void lcd_fill_rect(int x, int y, int x2, int y2, color_t color)
{
    uint16_t dy = y2 - y;
    if (y == y2)
    {
        return lcd_fill_horiz_line(x, y, x2, color);
    }
    if (x == x2)
    {
        x2++;
    }
    if (lcd_fixBoundary(&x, &y, &x2, &y2) != false)
    {
        for (uint16_t yi = 0; yi < dy; yi++)
        {
            lcd_fill_line_directly_to_screen(x, y + yi, x2, NULL, ONE_COLOR_TO_LINE, color);
        }
    }
}

//draw an image
static void lcd_put_image(int x, int y, int x2, int y2, fb_t *src)
{
    if (src)
    {
        int dx = src->width;
        for (int iy = 0; iy < src->height; iy++)
        {
            lcd_put_image_line(x, y + iy, x2, (color_t *)(src->buffer));
            src->buffer += dx;
        }
    }
}

//reset lcd
static void lcd_reset(void)
{
    int cmd = 0;
    LCD_BCKL_OFF();
    //hardware reset lcd
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);

    while (lcd_init_cmds[cmd].databytes != 0xff)
    {
        lcd_cmd(lcd_init_cmds[cmd].cmd);
        lcd_data(lcd_init_cmds[cmd].data, lcd_init_cmds[cmd].databytes & 0x1F);
        if (lcd_init_cmds[cmd].databytes & 0x80)
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        cmd++;
    }
    LCD_BCKL_ON();
}

//deinit of lcd struct
static void lcd_deinit()
{
    frame_buffer_destroy(&gFrameBuffer);
#ifdef CONFIG_GUI_FRAMEWORK_LVGL
    xTimerStop(lvgl_timer, 0);
    esp_deregister_freertos_tick_hook(lv_tick_hook);
#endif
}

//get current framebuffer
static color_t *lcd_get_frame_buffer()
{
    return (color_t *)(gFrameBuffer.buffer);
}

#ifdef CONFIG_GUI_FRAMEWORK_LVGL
//lvgl lcd register function:flush(buffered mode)
static void lvgl_disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_map)
{
    esp_err_t ret;
    spi_transaction_t trans[6];
    for (int x = 0; x < 6; x++)
    {
        memset(&trans[x], 0, sizeof(spi_transaction_t));
        if ((x & 1) == 0)
        {
            //Even transfers are commands
            trans[x].length = 8;
            trans[x].user = (void *)0;
        }
        else
        {
            //Odd transfers are data
            trans[x].length = 8 * 4;
            trans[x].user = (void *)1;
        }
        trans[x].flags = SPI_TRANS_USE_TXDATA;
    }

    uint32_t size = (x2 - x1 + 1) * (y2 - y1 + 1);
    uint8_t *color_u8 = (uint8_t *)color_map;
    for (uint32_t i = 0; i < size * 2; i += 2)
    {
        color_u8[i] ^= color_u8[i + 1];
        color_u8[i + 1] ^= color_u8[i];
        color_u8[i] ^= color_u8[i + 1];
    }

    trans[0].tx_data[0] = 0x2A;             //Column Address Set
    trans[1].tx_data[0] = (x1 >> 8) & 0xFF; //Start Col High
    trans[1].tx_data[1] = x1 & 0xFF;        //Start Col Low
    trans[1].tx_data[2] = (x2 >> 8) & 0xFF; //End Col High
    trans[1].tx_data[3] = x2 & 0xFF;        //End Col Low
    trans[2].tx_data[0] = 0x2B;             //Page address set
    trans[3].tx_data[0] = (y1 >> 8) & 0xFF; //Start page high
    trans[3].tx_data[1] = y1 & 0xFF;        //start page low
    trans[3].tx_data[2] = (y2 >> 8) & 0xFF; //end page high
    trans[3].tx_data[3] = y2 & 0xFF;        //end page low
    trans[4].tx_data[0] = 0x2C;             //memory write
    trans[5].tx_buffer = color_map;
    trans[5].flags = 0;
    trans[5].length = size * 2 * 8;

    //Queue all transactions.
    for (int x = 0; x < 6; x++)
    {
        ret = spi_device_queue_trans(spi, &trans[x], portMAX_DELAY);
        assert(ret == ESP_OK);
    }
    spi_transaction_t *rtrans;
    for (int x = 0; x < 6; x++)
    {
        ret = spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
        assert(ret == ESP_OK);
    }

    lv_flush_ready();
}

//lv_task_handler should be called periodically around 10ms
static void lvgl_timer_callback(TimerHandle_t xTimer)
{
    lv_task_handler();
}
#endif

void lcd_init(lcd_t *lcd)
{
    lcd_mutex = xSemaphoreCreateMutex();
    memset((void *)lcd, 0, sizeof(lcd_t));
    memset(&gFrameBuffer, 0, sizeof(fb_t));
    gpio_spec_init();
    spi_spec_init();
    lcd_reset();
    lcd->deinit = lcd_deinit;
    lcd->get_x_resolution = lcd_get_x_resolution;
    lcd->get_y_resolution = lcd_get_y_resolution;
    lcd->reset = lcd_reset;
    lcd->get_frame_buffer = lcd_get_frame_buffer;
    lcd->set_pixel = lcd_set_pixel;
    lcd->fill_horiz_line = lcd_fill_horiz_line;
    lcd->fill_rect = lcd_fill_rect;
    lcd->put_image = lcd_put_image;
#ifdef CONFIG_GUI_FRAMEWORK_LVGL
    lv_init();
    lv_disp_drv_t littlevgl_disp_drv;
    lv_disp_drv_init(&littlevgl_disp_drv);
    littlevgl_disp_drv.disp_flush = lvgl_disp_flush;
    lv_disp_drv_register(&littlevgl_disp_drv);
    esp_register_freertos_tick_hook(lv_tick_hook);
    lvgl_timer = xTimerCreate(
        "lvgl_timer",
        LVGL_TIMER_PERIOD_MS / portTICK_PERIOD_MS,
        pdTRUE,
        (void *)0,
        lvgl_timer_callback);
    xTimerStart(lvgl_timer, 0);
#endif
}
