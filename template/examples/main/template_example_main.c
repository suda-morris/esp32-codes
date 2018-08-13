#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

#include "nvs_flash.h"
#include "sdkconfig.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "sdkconfig.h"

#include <stdio.h>
#include <string.h>

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static const char *TAG = "template";

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id)
	{
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		esp_wifi_disconnect();
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}

#define WEB_SERVER "baidu.com"
#define WEB_PORT 80
#define WEB_URL "http://www.baidu.com/"

static const char *REQUEST = "GET " WEB_URL " HTTP/1.1\r\n"
							 "Host: " WEB_SERVER "\r\n"
							 "User-Agent: esp-idf/2.0 esp32\r\n"
							 "\r\n";

static void http_get_task(void *pvParameters)
{
	const struct addrinfo hints = {.ai_family = AF_INET, .ai_socktype = SOCK_STREAM};
	struct addrinfo *res;
	struct in_addr *addr;
	int s, r;
	char recv_buf[64];

	while (1)
	{
		xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
							false, true, portMAX_DELAY);
		ESP_LOGI(TAG, "Connected to AP");

		int err = getaddrinfo(WEB_SERVER, "80", &hints, &res);

		if (err != 0 || res == NULL)
		{
			ESP_LOGE(TAG, "DNS lookup failed err=%d res=%p", err, res);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}

		addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
		ESP_LOGI(TAG, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

		s = socket(res->ai_family, res->ai_socktype, 0);
		if (s < 0)
		{
			ESP_LOGE(TAG, "... Failed to allocate socket.");
			freeaddrinfo(res);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(TAG, "... allocated socket");

		if (connect(s, res->ai_addr, res->ai_addrlen) != 0)
		{
			ESP_LOGE(TAG, "... socket connect failed errno=%d", errno);
			close(s);
			freeaddrinfo(res);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}

		ESP_LOGI(TAG, "... connected");
		freeaddrinfo(res);

		if (write(s, REQUEST, strlen(REQUEST)) < 0)
		{
			ESP_LOGE(TAG, "... socket send failed");
			close(s);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(TAG, "... socket send success");

		struct timeval receiving_timeout;
		receiving_timeout.tv_sec = 5;
		receiving_timeout.tv_usec = 0;
		if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
					   sizeof(receiving_timeout)) < 0)
		{
			ESP_LOGE(TAG, "... failed to set socket receiving timeout");
			close(s);
			vTaskDelay(4000 / portTICK_PERIOD_MS);
			continue;
		}
		ESP_LOGI(TAG, "... set socket receiving timeout success");

		do
		{
			bzero(recv_buf, sizeof(recv_buf));
			r = read(s, recv_buf, sizeof(recv_buf) - 1);
			for (int i = 0; i < r; i++)
			{
				putchar(recv_buf[i]);
			}
		} while (r > 0);

		ESP_LOGI(TAG,
				 "... done reading from socket. Last read return=%d errno=%d\r\n",
				 r, errno);
		close(s);
		for (int countdown = 10; countdown >= 0; countdown--)
		{
			ESP_LOGI(TAG, "%d... ", countdown);
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		ESP_LOGI(TAG, "Starting again!");
	}
}

static void initialise_wifi(void)
{
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	wifi_config_t wifi_config = {.sta = {.ssid = CONFIG_WIFI_SSID, .password = CONFIG_WIFI_PASSWORD, .bssid_set = false}};
	ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void)
{
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
		   (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
		   (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
	printf("silicon revision %d, ", chip_info.revision);
	printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
		   (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

	ESP_ERROR_CHECK(nvs_flash_init());

	initialise_wifi();

	xTaskCreate(&http_get_task, "http_get_task", 4096, NULL, 2, NULL);
}
