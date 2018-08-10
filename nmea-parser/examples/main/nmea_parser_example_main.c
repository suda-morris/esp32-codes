/* nmea parser example, that decode data stream from GPS receiver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nmea_parser.h"

static const char *TAG = "nmea_parser";

static void gps_update(void *arg)
{
    gps_t *gh = (gps_t *)arg;
    ESP_LOGI(TAG, "%d:%d:%d.%d %d-%d-%d=>latitude=%f,longtitude=%f,altitude=%f",
             gh->tim.hours, gh->tim.minutes, gh->tim.seconds, gh->tim.thousands,
             gh->date.year, gh->date.month, gh->date.day, gh->latitude, gh->longitude, gh->altitude);
}

void app_main()
{
    gps_t *gh = nmea_parser_start(gps_update);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    nmea_parser_stop(gh);

    vTaskDelay(5000 / portTICK_PERIOD_MS);

    gh = nmea_parser_start(gps_update);
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    nmea_parser_stop(gh);
}
