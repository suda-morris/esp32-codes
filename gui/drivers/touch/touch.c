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
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "touch.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#ifdef CONFIG_GUI_FRAMEWORK_LVGL
#include "lvgl.h"
#endif

static const char *TAG = "touch";                    //tag name of this file
static SemaphoreHandle_t semaphore_calibrate = NULL; //semaphore for calibration
static TimerHandle_t debounce_timer = NULL;          //timer handler for debounce
static TaskHandle_t sample_task = NULL;              //task handler for sample work
static EventGroupHandle_t sample_event_group;        //event group handler for sample work

#define BIT_PRESSED (1 << 0) //bit mask of "Pressed" used in event group
#define DEBOUNCE_TIME_MS 40  //delay time used in debounce,unit:ms

#ifndef CONFIG_SPI_TOUCH_PIN_SAME_LCD //check if touch panel's spi bus is same to lcd display
#define PIN_NUM_MISO CONFIG_LCD_SPI_MISO
#define PIN_NUM_MOSI CONFIG_LCD_SPI_MOSI
#define PIN_NUM_CLK CONFIG_LCD_SPI_SCLK
#define RTP_USE_HOST_DEVICE VSPI_HOST
#endif

#ifndef RTP_USE_HOST_DEVICE
#define RTP_USE_HOST_DEVICE HSPI_HOST
#endif
#define PIN_NUM_CS CONFIG_RTP_SPI_CS
#define PIN_NUM_IRQ CONFIG_RTP_SPI_IRQ
#define SPI_CLK_SPEED_MHZ CONFIG_RTP_SPI_CLK_SPEED_MHZ

#define SPI_TRANS_QUEUE_SIZE 7 //depth of queue used for spi transaction
#define TOUCH_CMD_X 0xD0       //command of getting x coordinate
#define TOUCH_CMD_Y 0x90       //command of getting y coordinate

//resolution of lcd
#define LCD_X_RESOLUTION CONFIG_LCD_X_RESOLUTION
#define LCD_Y_RESOLUTION CONFIG_LCD_Y_RESOLUTION

/**
 * @brief object to describe position
 * 
 */
typedef struct position
{
    uint32_t x;
    uint32_t y;
} position_t;

/**
 * @brief object to describe a runtime info for touch panel
 * 
 */
typedef struct touch_runtime_s
{
    bool calibrated;
    float xfactor;
    float yfactor;
    int offset_x;
    int offset_y;
    position_t position;
} touch_rt_t;

static touch_rt_t touch_rt = {
    .calibrated = true,
    .xfactor = -0.06645,
    .yfactor = -0.09052,
    .offset_x = 257,
    .offset_y = 355,
    .position = {0, 0}};

/**
 * @brief object of one sample
 * 
 */
typedef struct sample_s
{
    bool is_pressed;
    int last_x;
    int last_y;
} sample_t;

static sample_t sample = {
    .is_pressed = false,
    .last_x = 0,
    .last_y = 0};

/**
 * @brief state define for Debounce of every touch
 * 
 */
typedef enum debounce_state
{
    DEBOUNCE_WAITING_DOWN_EDGE,
    DEBOUNCE_DELAY1,
    DEBOUNCE_WAITING_UP_EDGE,
    DEBOUNCE_DELAY2
} debounce_state_t;

static debounce_state_t debounce_state = DEBOUNCE_WAITING_DOWN_EDGE;

/**
 * @brief GPIO external interrupt service routine
 * 
 * @param arg parameters passed to isr
 */
static void IRAM_ATTR touch_isr_handler(void *arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    gpio_set_intr_type(PIN_NUM_IRQ, GPIO_INTR_DISABLE); //disable the gpio interrupt for the moment
    /* state translation*/
    switch (debounce_state)
    {
    case DEBOUNCE_WAITING_DOWN_EDGE:          //waiting for negedge
        if (gpio_get_level(PIN_NUM_IRQ) == 0) //real negedge
        {
            debounce_state = DEBOUNCE_DELAY1; //delay some time to debounce
        }
        else //it's a noise
        {
            debounce_state = DEBOUNCE_WAITING_DOWN_EDGE;        //still waiting for negedge
            gpio_set_intr_type(PIN_NUM_IRQ, GPIO_INTR_NEGEDGE); //re-enable gpio interrupt
        }
        break;
    case DEBOUNCE_DELAY1:                                   //waiting stable low level,but come up with a edge interrupt
        debounce_state = DEBOUNCE_WAITING_DOWN_EDGE;        //re-waiting negedge
        gpio_set_intr_type(PIN_NUM_IRQ, GPIO_INTR_NEGEDGE); //re-enable gpio interrupt
        break;
    case DEBOUNCE_WAITING_UP_EDGE:            //waiting for posedge
        if (gpio_get_level(PIN_NUM_IRQ) == 1) //real posedge
        {
            debounce_state = DEBOUNCE_DELAY2; //delay some time to debounce
        }
        else //it's a noise
        {
            debounce_state = DEBOUNCE_WAITING_UP_EDGE;          //still waiting for posedge
            gpio_set_intr_type(PIN_NUM_IRQ, GPIO_INTR_POSEDGE); //re-enable gpio interrupt
        }
        break;
    case DEBOUNCE_DELAY2:                                   //waiting stable high level,but come up with a edge interrupt
        debounce_state = DEBOUNCE_WAITING_UP_EDGE;          //re-waiting negedge
        gpio_set_intr_type(PIN_NUM_IRQ, GPIO_INTR_POSEDGE); //re-enable gpio interrupt
        break;
    }
    if (debounce_state == DEBOUNCE_DELAY1 || debounce_state == DEBOUNCE_DELAY2) //only these two state will wake up timer
    {
        xTimerStartFromISR(debounce_timer, &xHigherPriorityTaskWoken); //wake up timer to delay specific time
        if (xHigherPriorityTaskWoken != pdFALSE)
        {
            portYIELD_FROM_ISR();
        }
    }
}

/**
 * @brief gpio specific initialise
 * 
 */
static void gpio_spec_init(void)
{
    gpio_pad_select_gpio(PIN_NUM_IRQ);                          //select gpio function
    gpio_set_direction(PIN_NUM_IRQ, GPIO_MODE_INPUT);           //direction:input
    gpio_pullup_en(PIN_NUM_IRQ);                                //pull-up
    gpio_pulldown_dis(PIN_NUM_IRQ);                             //no pull-down
    gpio_set_intr_type(PIN_NUM_IRQ, GPIO_INTR_NEGEDGE);         //enable interrupt
    gpio_install_isr_service(0);                                //install isr service
    gpio_isr_handler_add(PIN_NUM_IRQ, touch_isr_handler, NULL); //add gpio isr handler
}

/**
 * @brief spi specific initialise
 * 
 */
static spi_device_handle_t spi;
static void spi_spec_init(void)
{
    esp_err_t ret;
    //if touch panel's spi bus is same to lcd, no need to initialize the bus again
#ifndef CONFIG_SPI_TOUCH_PIN_SAME_LCD
    spi_bus_config_t buscfg = {
        .miso_io_num = PIN_NUM_MISO,
        .mosi_io_num = PIN_NUM_MOSI,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    ret = spi_bus_initialize(RTP_USE_HOST_DEVICE, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
#endif
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_CLK_SPEED_MHZ * 1000 * 1000,
        .mode = 0, //spi mode 0
        .spics_io_num = PIN_NUM_CS,
        .queue_size = SPI_TRANS_QUEUE_SIZE,
    };
    //add spi device to spi bus
    ret = spi_bus_add_device(RTP_USE_HOST_DEVICE, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
}

/**
 * @brief read raw date of touch panel
 * 
 * @param command 
 * @return uint16_t 
 */
static uint16_t touch_read_raw(const uint8_t command)
{
    esp_err_t ret;
    uint8_t data[3] = {0};
    data[0] = command;
    spi_transaction_t t = {
        .length = 8 * 3,
        .tx_buffer = &data,
        .flags = SPI_TRANS_USE_RXDATA,
    };
    ret = spi_device_transmit(spi, &t);
    assert(ret == ESP_OK);
    return (t.rx_data[1] << 8 | t.rx_data[2]) >> 3;
}

/**
 * @brief calibrate the touch panel
 * 
 */
static void touch_calibration()
{
    uint16_t px[2], py[2], xPot[4], yPot[4];
    touch_rt.calibrated = false;
    //create semaphore before use and delete it later
    if (!semaphore_calibrate)
    {
        semaphore_calibrate = xSemaphoreCreateBinary();
    }
    ESP_LOGW(TAG, "push [left-top] of your screen");
    xSemaphoreTake(semaphore_calibrate, portMAX_DELAY);
    ESP_LOGI(TAG, "%d,%d", touch_rt.position.x, touch_rt.position.y);
    xPot[0] = touch_rt.position.x;
    yPot[0] = touch_rt.position.y;

    ESP_LOGW(TAG, "push [right-top] of your screen");
    xSemaphoreTake(semaphore_calibrate, portMAX_DELAY);
    ESP_LOGI(TAG, "%d,%d", touch_rt.position.x, touch_rt.position.y);
    xPot[1] = touch_rt.position.x;
    yPot[1] = touch_rt.position.y;

    ESP_LOGW(TAG, "push [right-bottom] of your screen");
    xSemaphoreTake(semaphore_calibrate, portMAX_DELAY);
    ESP_LOGI(TAG, "%d,%d", touch_rt.position.x, touch_rt.position.y);
    xPot[2] = touch_rt.position.x;
    yPot[2] = touch_rt.position.y;

    ESP_LOGW(TAG, "push [left-bottom] of your screen");
    xSemaphoreTake(semaphore_calibrate, portMAX_DELAY);
    ESP_LOGI(TAG, "%d,%d", touch_rt.position.x, touch_rt.position.y);
    xPot[3] = touch_rt.position.x;
    yPot[3] = touch_rt.position.y;

    px[0] = (xPot[0] + xPot[1]) / 2;
    py[0] = (yPot[0] + yPot[3]) / 2;
    px[1] = (xPot[2] + xPot[3]) / 2;
    py[1] = (yPot[2] + yPot[1]) / 2;
    //calculate the calibration parameters
#ifdef CONFIG_TOUCH_SWAP_X_Y
    touch_rt.xfactor = (float)LCD_Y_RESOLUTION / (px[1] - px[0]);
    touch_rt.yfactor = (float)LCD_X_RESOLUTION / (py[1] - py[0]);
    touch_rt.offset_x = (int16_t)LCD_Y_RESOLUTION - ((float)px[1] * touch_rt.xfactor);
    touch_rt.offset_y = (int16_t)LCD_X_RESOLUTION - ((float)py[1] * touch_rt.yfactor);
#else
    touch_rt.xfactor = (float)LCD_X_RESOLUTION / (px[1] - px[0]);
    touch_rt.yfactor = (float)LCD_Y_RESOLUTION / (py[1] - py[0]);
    touch_rt.offset_x = (int16_t)LCD_X_RESOLUTION - ((float)px[1] * touch_rt.xfactor);
    touch_rt.offset_y = (int16_t)LCD_Y_RESOLUTION - ((float)py[1] * touch_rt.yfactor);
#endif
    ESP_LOGI(TAG, "xf=%f,yf=%f,xo=%d,yo=%d", touch_rt.xfactor, touch_rt.yfactor, touch_rt.offset_x, touch_rt.offset_y);
    touch_rt.calibrated = true;
    vSemaphoreDelete(semaphore_calibrate);
    semaphore_calibrate = NULL;
}

/**
 * @brief callback funtion for debounce timer
 * 
 * @param xTimer timer handler
 */
static void touch_debounce_callback(TimerHandle_t xTimer)
{
    /* state transform */
    switch (debounce_state)
    {
    case DEBOUNCE_WAITING_DOWN_EDGE:                        //waiting for negedge
        debounce_state = DEBOUNCE_WAITING_DOWN_EDGE;        //still waiting for negedge
        gpio_set_intr_type(PIN_NUM_IRQ, GPIO_INTR_NEGEDGE); //re-enable gpio interrupt
        break;
    case DEBOUNCE_DELAY1:                     //waiting stable low level
        if (gpio_get_level(PIN_NUM_IRQ) == 0) //low level has been stable
        {
            sample.is_pressed = true;
            ESP_LOGI(TAG, "Pressed");
            debounce_state = DEBOUNCE_WAITING_UP_EDGE;           //start to waiting posedge
            xEventGroupSetBits(sample_event_group, BIT_PRESSED); //wake up another task to get sample work
            gpio_set_intr_type(PIN_NUM_IRQ, GPIO_INTR_POSEDGE);  //re-enable gpio interrupt
        }
        else //low level is not stable
        {
            debounce_state = DEBOUNCE_WAITING_DOWN_EDGE;        //re-waiting negedge
            gpio_set_intr_type(PIN_NUM_IRQ, GPIO_INTR_NEGEDGE); //re-enable gpio interrupt
        }
        break;
    case DEBOUNCE_WAITING_UP_EDGE:                 //waiting for posedge
        debounce_state = DEBOUNCE_WAITING_UP_EDGE; //still waiting for posedge
        break;
    case DEBOUNCE_DELAY2:                     //waiting stable high level
        if (gpio_get_level(PIN_NUM_IRQ) == 1) //high level has been stable
        {
            sample.is_pressed = false;
            ESP_LOGI(TAG, "Released");
            debounce_state = DEBOUNCE_WAITING_DOWN_EDGE;           //start to waiting negedge
            xEventGroupClearBits(sample_event_group, BIT_PRESSED); //stop another task's sampling work
            if (!touch_rt.calibrated)                              //if it hasn't calibrated
            {
                xSemaphoreGive(semaphore_calibrate); //wake up the calibrate task
            }
            gpio_set_intr_type(PIN_NUM_IRQ, GPIO_INTR_NEGEDGE); //re-enable gpio interrupt
        }
        else //high level is not stable
        {
            debounce_state = DEBOUNCE_WAITING_UP_EDGE;          //re-waiting posedge
            gpio_set_intr_type(PIN_NUM_IRQ, GPIO_INTR_POSEDGE); //re-enable gpio interrupt
        }
        break;
    }
}

/**
 * @brief get a whole raw touch point, without any transform.
 * 
 */
static void touch_sample(void)
{
    position_t samples[16];
    uint32_t distances[16];
    uint32_t aveX = 0;
    uint32_t aveY = 0;
    /*  take 16 times sample, select half of them(based on minimum variance), and return the average*/
    for (int i = 0; i < 16; i++)
    {
        samples[i].x = touch_read_raw(TOUCH_CMD_X);
        samples[i].y = touch_read_raw(TOUCH_CMD_Y);
        if (samples[i].x == 0 || samples[i].x == 4095 || samples[i].y == 0 || samples[i].y == 4095)
        {
            return;
        }
        aveX += samples[i].x;
        aveY += samples[i].y;
    }
    aveX >>= 4;
    aveY >>= 4;
    for (int i = 0; i < 16; i++)
    {
        distances[i] = ((aveX - samples[i].x) * (aveX - samples[i].x)) + ((aveY - samples[i].y) * (aveY - samples[i].y));
    }

    for (int i = 0; i < 15; i++)
    {
        for (int j = 0; j < 15 - i; j++)
        {
            if (distances[j] > distances[j + 1])
            {
                position_t pos = samples[j];
                samples[j] = samples[j + 1];
                samples[j + 1] = pos;
            }
        }
    }

    aveX = 0;
    aveY = 0;
    for (int i = 0; i < 8; i++)
    {
        aveX += samples[i].x;
        aveY += samples[i].y;
    }

    touch_rt.position.x = aveX >> 3;
    touch_rt.position.y = aveY >> 3;
}

/**
 * @brief task function for calculate the real touch point
 * 
 * @param pvParameter 
 */
static void touch_sample_task(void *pvParameter)
{
    EventBits_t uxBits;
    while (1)
    {
        /* wait for event group to wake up */
        uxBits = xEventGroupWaitBits(
            sample_event_group,
            BIT_PRESSED,
            pdFALSE,
            pdFALSE,
            100 / portTICK_PERIOD_MS);
        if ((uxBits & BIT_PRESSED) != 0) //presse event happened
        {
            touch_sample(); //get raw data
            if (touch_rt.calibrated)
            {
                int x = touch_rt.offset_x + touch_rt.position.x * touch_rt.xfactor;
                int y = touch_rt.offset_y + touch_rt.position.y * touch_rt.yfactor;
                if (x != y)
                {
                    x ^= y;
                    y ^= x;
                    x ^= y;
                }
                if (x > LCD_X_RESOLUTION)
                {
                    x = LCD_X_RESOLUTION;
                }
                if (y > LCD_Y_RESOLUTION)
                {
                    y = LCD_Y_RESOLUTION;
                }
                if (x < 0)
                {
                    x = 0;
                }
                if (y < 0)
                {
                    y = 0;
                }
                sample.last_x = x;
                sample.last_y = y;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief touch driver read interface of littlevgl
 * 
 * @param data 
 * @return true 
 * @return false 
 */
static bool lvgl_touch_read(lv_indev_data_t *data)
{
    data->point.x = sample.last_x;
    data->point.y = sample.last_y;
    data->state = sample.is_pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    return false;
}

/**
 * @brief destruction of rtp object
 * 
 */
static void rtp_deinit(void)
{
    gpio_isr_handler_remove(PIN_NUM_IRQ);
    gpio_uninstall_isr_service();
    xTimerDelete(debounce_timer, 0);
    vEventGroupDelete(sample_event_group);
    if (semaphore_calibrate)
    {
        vSemaphoreDelete(semaphore_calibrate);
    }
    if (sample_task)
    {
        vTaskDelete(sample_task);
    }
}

/**
 * @brief resister touch panel initialization
 * 
 * @param rtp resister touch panel object
 */
void rtp_init(rtp_t *rtp)
{
    gpio_spec_init();
    spi_spec_init();
    memset(rtp, 0, sizeof(rtp_t));
    rtp->deinit = rtp_deinit;
    rtp->calibration = touch_calibration;
    sample_event_group = xEventGroupCreate();
    //create a timer for debounce
    debounce_timer = xTimerCreate(
        "debounce_timer",
        DEBOUNCE_TIME_MS / portTICK_PERIOD_MS, //period time
        pdFALSE,                               //no auto load
        (void *)0,                             //timer parameter
        touch_debounce_callback);              //timer callback
    //create a task for sample touch position
    xTaskCreate(
        touch_sample_task,   //task entry
        "touch_sample_task", //task name
        2048,                //task depth
        NULL,                //task parameter
        1,                   //task priority
        &sample_task);       //task handler
    // make sure touch panel has calibrated
    if (touch_rt.calibrated == false)
    {
        touch_calibration();
    }
    //install touch driver for littlevgl
#ifdef CONFIG_GUI_FRAMEWORK_LVGL
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read = lvgl_touch_read;
    lv_indev_drv_register(&indev_drv);
#endif
}