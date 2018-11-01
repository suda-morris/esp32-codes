/* ethernet Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/socket.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "tcpip_adapter.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp32x_osl_eth.h"

#define SERVER_PORT (3629)
#define BUFFER_SIZE (1024)
static const char *TAG = "socket-example";
static EventGroupHandle_t net_event_group;
static const int GOTIP_BIT = BIT0;
static char buf[BUFFER_SIZE];

#define PIN_OSC_EN (17)

esp_err_t esp32x_eth_mac_lowlevel_init_extra(eth_mac_if_t *mac)
{
    ESP_LOGI(TAG, "esp32x_eth_mac_extra_lowlevel_init");

    /* ESP32_Ethernet_V3 board need GPIO17 to Enable the External Oscillator */
    gpio_pad_select_gpio(PIN_OSC_EN);
    gpio_set_direction(PIN_OSC_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_OSC_EN, 1);

    return ESP_OK;
}

static esp_err_t eth_event_handler(void *ctx, system_event_t *event)
{
    tcpip_adapter_ip_info_t ip;
    switch (event->event_id) {
    case SYSTEM_EVENT_ETH_CONNECTED:
        ESP_LOGI(TAG, "Ethernet Connected");
        break;
    case SYSTEM_EVENT_ETH_DISCONNECTED:
        ESP_LOGI(TAG, "Ethernet Disconnected");
        break;
    case SYSTEM_EVENT_ETH_START:
        ESP_LOGI(TAG, "Ethernet Started");
        break;
    case SYSTEM_EVENT_ETH_GOT_IP:
        memset(&ip, 0, sizeof(tcpip_adapter_ip_info_t));
        ESP_ERROR_CHECK(tcpip_adapter_get_ip_info(ESP_IF_ETH, &ip));
        ESP_LOGI(TAG, "Ethernet Got IP Addr");
        ESP_LOGI(TAG, "~~~~~~~~~~~");
        ESP_LOGI(TAG, "ETHIP:" IPSTR, IP2STR(&ip.ip));
        ESP_LOGI(TAG, "ETHMASK:" IPSTR, IP2STR(&ip.netmask));
        ESP_LOGI(TAG, "ETHGW:" IPSTR, IP2STR(&ip.gw));
        ESP_LOGI(TAG, "~~~~~~~~~~~");
        xEventGroupSetBits(net_event_group, GOTIP_BIT);
        break;
    case SYSTEM_EVENT_ETH_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        xEventGroupClearBits(net_event_group, GOTIP_BIT);
        break;
    default:
        break;
    }
    return ESP_OK;
}

static void thrd_accept(void *param)
{
    int client_fd = (int)param;
    int real_read, real_write;
    while (1) {
        memset(buf, 0, BUFFER_SIZE);
        real_read = recv(client_fd, buf, BUFFER_SIZE, 0);
        if (real_read < 0) {
            ESP_LOGE(TAG, "read error");
            break;
        } else if (real_read == 0) {
            ESP_LOGI(TAG, "Client %d has exited", client_fd);
            break;
        } else {
            printf("Receive from client %d:%s\r\n", client_fd, buf);
            if (strncmp(buf, "quit", 4) == 0) {
                ESP_LOGI(TAG, "client %d(socket) has exited\r\n", client_fd);
                break;
            }
            strcat(buf, "OK");
            real_write = send(client_fd, buf, strlen(buf), 0);
            if (real_write < 0) {
                ESP_LOGE(TAG, "write error");
                break;
            }
        }

    }
    close(client_fd);
    vTaskDelete(NULL);
}

void app_main()
{
    int server_fd, client_fd;
    struct sockaddr_in server_addr, client_addr;
    socklen_t cin_size;
    int ret;

    net_event_group = xEventGroupCreate();
    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_init(eth_event_handler, NULL));

    ESP_ERROR_CHECK(esp32x_osl_eth_init());

    /* Wait for ip */
    xEventGroupWaitBits(net_event_group, GOTIP_BIT, pdFALSE, pdTRUE, portMAX_DELAY);

    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == -1) {
        ESP_LOGE(TAG, "Create socket error");
    }
    ESP_LOGI(TAG, "Create TCP server socket success");

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    ret = bind(server_fd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (ret < 0) {
        ESP_LOGE(TAG, "bind socket error");
    }
    ESP_LOGI(TAG, "bind success");

    ret = listen(server_fd, 5);
    if (ret < 0) {
        ESP_LOGE(TAG, "socket listen error");
    }
    ESP_LOGI(TAG, "waiting for client");

    while (1) {
        client_fd = accept(server_fd, (struct sockaddr *) &client_addr, &cin_size);
        if (client_fd < 0) {
            ESP_LOGE(TAG, "accept error");
        }
        ret = xTaskCreate(thrd_accept, "accept_task", 4096, (void *)client_fd, 3, NULL);
        if (ret != pdPASS) {
            ESP_LOGE(TAG, "task create error");
        }
    }
    close(server_fd);
}