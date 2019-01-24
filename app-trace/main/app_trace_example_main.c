/* Application Trace to Host Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_trace.h"
#include "esp_log.h"

static const char *TAG = "example";

void app_main()
{
    int number = 0;
    while (1) {
        ESP_LOGI(TAG, "Hello");
        char *ptr = (char *)esp_apptrace_buffer_get(ESP_APPTRACE_DEST_TRAX, 32, 100);
        bzero(ptr, 32);
        snprintf(ptr, 32, "Here is the number %d\r\n", number++);
        esp_apptrace_buffer_put(ESP_APPTRACE_DEST_TRAX, (uint8_t *)ptr, 100);
        esp_apptrace_flush(ESP_APPTRACE_DEST_TRAX, ESP_APPTRACE_TMO_INFINITE);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

