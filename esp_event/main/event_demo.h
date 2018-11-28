/* esp_event (event loop library) basic example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_timer.h"

/**
 * @brief Declaration of the demo event family
 *
 */
ESP_EVENT_DECLARE_BASE(DEMO_EVENTS_BASE);

/**
 * @brief Declaration of demo event id
 *
 */
enum {
    DEMO_EVENT_0,
    DEMO_EVENT_1,
    DEMO_EVENT_2,
    DEMO_EVENT_MAX
};

#ifdef __cplusplus
}
#endif
