/* esp_event (event loop library) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include "esp_types.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "event_demo.h"


ESP_EVENT_DEFINE_BASE(DEMO_EVENTS_BASE)

static const char *TAG = "event_demo";

esp_event_loop_handle_t event_loop = NULL;

static void application_task(void *args)
{
    while (1) {
        ESP_LOGI(TAG, "running application task");
        esp_event_loop_run(event_loop, 100);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void demo_event_handlerA(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    char *handler_str = (char *)handler_args;
    char *event_str = (char *)event_data;
    ESP_LOGI(TAG, "%s is handling and got data: %s", handler_str, event_str);
    switch (id) {
    case DEMO_EVENT_0:
        ESP_LOGI(TAG, "\tDEMO_EVENT0");
        break;
    case DEMO_EVENT_1:
        ESP_LOGI(TAG, "\tDEMO_EVENT1");
        break;
    case DEMO_EVENT_2:
        ESP_LOGI(TAG, "\tDEMO_EVENT2");
        break;
    default:
        break;
    }
}

static void demo_event_handlerB(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    char *handler_str = (char *)handler_args;
    char *event_str = (char *)event_data;
    ESP_LOGI(TAG, "%s is handling and got data: %s", handler_str, event_str);
    switch (id) {
    case DEMO_EVENT_0:
        ESP_LOGI(TAG, "\tDEMO_EVENT0");
        break;
    case DEMO_EVENT_1:
        ESP_LOGI(TAG, "\tDEMO_EVENT1");
        break;
    case DEMO_EVENT_2:
        ESP_LOGI(TAG, "\tDEMO_EVENT2");
        break;
    default:
        break;
    }
}

static void task_event_source(void *args)
{
    uint32_t i = 0;
    char event_data[] = "event data here";
    while (1) {
        ESP_ERROR_CHECK(esp_event_post_to(event_loop, DEMO_EVENTS_BASE, DEMO_EVENT_0 + i % DEMO_EVENT_MAX,
                                          event_data, sizeof(event_data), portMAX_DELAY));
        vTaskDelay(pdMS_TO_TICKS(1000));
        i++;
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_event_loop_args_t event_loop_args = {
        .queue_size = 5,
        .task_name = NULL
    };

    ESP_ERROR_CHECK(esp_event_loop_create(&event_loop_args, &event_loop));

    ESP_ERROR_CHECK(esp_event_handler_register_with(event_loop, DEMO_EVENTS_BASE,
                    ESP_EVENT_ANY_ID, demo_event_handlerA, "morris"));
    ESP_ERROR_CHECK(esp_event_handler_register_with(event_loop, DEMO_EVENTS_BASE,
                    ESP_EVENT_ANY_ID, demo_event_handlerB, "wendy"));

    xTaskCreate(task_event_source, "task_event_source", 2048, NULL, uxTaskPriorityGet(NULL), NULL);

    xTaskCreate(application_task, "application_task", 2048, NULL, uxTaskPriorityGet(NULL), NULL);
}