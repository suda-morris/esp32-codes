#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_spi_flash.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

#include "MQTTClient.h"

#include <stdio.h>
#include <string.h>

static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

static char* MQTT_BROKER_URL = "192.168.1.106";
static int MQTT_BROKER_PORT = 1883;
static char* MQTT_USERNAME = "pub_client";
static char* MQTT_PASSWORD = "321woaini";
static char* MQTT_CLIENT_ID = "ESP32";
static char* MQTT_TOPIC = "home/mytopic";

static const char *TAG = "mqtt";

static esp_err_t event_handler(void *ctx, system_event_t *event) {
	switch (event->event_id) {
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}

static void mqtt_task(void *pvParameters) {
	uint8_t sendbuf[64];
	uint8_t readbuf[64];
	MQTTClient client;
	Network net;
	MQTTMessage message;
	int count = 0;
	int rc;
	while (1) {
		xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
		false, true, portMAX_DELAY);
		ESP_LOGI(TAG, "Connected to AP");

		NetworkInit(&net);
		rc = NetworkConnect(&net, MQTT_BROKER_URL, MQTT_BROKER_PORT);
		if (rc != 0) {
			ESP_LOGE(TAG, "Connected to MQTT Broker's Network Failed,%d", rc);
		}
		ESP_LOGI(TAG, "Connected to MQTT Broker's Network");

		MQTTClientInit(&client, &net, 1000, sendbuf, sizeof(sendbuf), readbuf,
				sizeof(readbuf));
		MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
		data.willFlag = 0;
		data.MQTTVersion = 3;
		data.clientID.cstring = MQTT_CLIENT_ID;
		data.username.cstring = MQTT_USERNAME;
		data.password.cstring = MQTT_PASSWORD;
		data.keepAliveInterval = 5;
		data.cleansession = 1;

		rc = MQTTConnect(&client, &data);
		if (rc != 0) {
			ESP_LOGE(TAG, "MQTTConnect Failed,%d", rc);
		}
		ESP_LOGI(TAG, "MQTTConnect OK");

		char payload[30];
		message.qos = 1;
		message.retained = 1;
		sprintf(payload, "message number:%d", ++count);
		message.payload = payload;
		message.payloadlen = strlen(payload);
		rc = MQTTPublish(&client, MQTT_TOPIC, &message);
		if (rc != 0) {
			ESP_LOGE(TAG, "MQTT Publish Failed");
		}
		ESP_LOGI(TAG, "MQTT Publish OK");

		MQTTDisconnect(&client);
		NetworkDisconnect(&net);
		vTaskDelay(5000 / portTICK_PERIOD_MS);
	}
}

static void initialise_wifi(void) {
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT()
	;
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	wifi_config_t wifi_config = { .sta = { .ssid = "morris", .password =
			"321woaini", .bssid_set = false } };
	ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void) {
	esp_chip_info_t chip_info;
	esp_chip_info(&chip_info);
	printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ", chip_info.cores,
			(chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
			(chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
	printf("silicon revision %d, ", chip_info.revision);
	printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
			(chip_info.features & CHIP_FEATURE_EMB_FLASH) ?
					"embedded" : "external");

	ESP_ERROR_CHECK(nvs_flash_init());

	initialise_wifi();

	xTaskCreate(&mqtt_task, "mqtt_task", 4096, NULL, 5, NULL);
}
