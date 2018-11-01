#include <string.h>
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "tcpip_adapter.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp32x_osl_eth.h"
#include "errno.h"

#define TS_PACKET_SIZE (128)
#define RTP_PACKET_SIZE (1500)
#define RTP_STREAM_PORT (6666)
#define RTP_SEND_DELAY_MS (5000)

static const char *TAG = "rtp-sender";

extern const uint8_t server_cert_pem_start[] asm("_binary_espressif_ts_start");
extern const uint8_t server_cert_pem_end[] asm("_binary_espressif_ts_end");

static EventGroupHandle_t eth_event_group = NULL;
#define ETH_GOT_IP_BIT BIT(0)

/** RTP packets */
static uint8_t rtp_send_packet[RTP_PACKET_SIZE];

/**
 * @brief RTP(Realtime Transport Protocol) Header
 *
 */
typedef struct {
    uint8_t ccsrc_count : 4;  /* Contributing sources count*/
    uint8_t extension : 1;    /* Extension */
    uint8_t padding : 1;      /* Padding */
    uint8_t version : 2;      /* Version */
    uint8_t payload_type : 7; /* Payload type */
    uint8_t marker : 1;       /* Marker */
    uint16_t seq_num;         /* Sequence number */
    uint32_t timestamp;       /* Timestamp */
    uint32_t sync_src_id;     /* Synchronization source identifier */
} rtp_header_t;

/**
 * @brief RTP Payload Type
 *
 */
typedef enum {
    RTP_PT_PCMU = 0,
    RTP_PT_GSM = 3,
    RTP_PT_PCMA = 8,
    RTP_PT_JPEG = 26,
    RTP_PT_MP2T = 33,
    RTP_PT_H263 = 34,
    RTP_PT_DYNAMIC = 96
} rtp_payload_type_t;

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
        xEventGroupSetBits(eth_event_group, ETH_GOT_IP_BIT);
        break;
    case SYSTEM_EVENT_ETH_STOP:
        ESP_LOGI(TAG, "Ethernet Stopped");
        break;
    default:
        break;
    }
    return ESP_OK;
}

/**
 * RTP send packets
 */
static void rtp_send_packets(int sock, struct sockaddr_in *dest_addr)
{
    rtp_header_t *hdr;
    size_t count = 0;
    const uint8_t *ptr = server_cert_pem_start;

    /* prepare RTP packet */
    hdr = (rtp_header_t *)rtp_send_packet;
    hdr->version = 2;
    hdr->padding = 0;
    hdr->extension = 0;
    hdr->ccsrc_count = 0;
    hdr->marker = 0;
    hdr->payload_type = RTP_PT_MP2T;
    hdr->timestamp = 0;
    hdr->sync_src_id = 0;
    count = sizeof(rtp_header_t);
    /* send RTP stream packets */
    while (ptr < server_cert_pem_end) {
        /* copy data to payload */
        memcpy(rtp_send_packet + count, ptr, TS_PACKET_SIZE);
        count += TS_PACKET_SIZE;
        /* send RTP stream packet */
        if (count + TS_PACKET_SIZE > RTP_PACKET_SIZE) {
            sendto(sock, rtp_send_packet, count, 0, (const struct sockaddr *)dest_addr, sizeof(struct sockaddr));
            hdr->seq_num = htons((uint16_t) (ntohs(hdr->seq_num) + 1));
            count = sizeof(rtp_header_t);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
        ptr += TS_PACKET_SIZE;
    }
}

void app_main()
{
    int sock;
    struct sockaddr_in dest_addr;

    eth_event_group = xEventGroupCreate();
    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_init(eth_event_handler, NULL));

    ESP_ERROR_CHECK(esp32x_osl_eth_init());

    xEventGroupWaitBits(eth_event_group, ETH_GOT_IP_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

    /* create new socket */
    sock = socket(AF_INET, SOCK_DGRAM, 0);

    /* prepare RTP stream address */
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(RTP_STREAM_PORT);
    dest_addr.sin_addr.s_addr = inet_addr("192.168.2.160");

    /* send RTP packets */
    memset(rtp_send_packet, 0, sizeof(rtp_send_packet));
    while (1) {
        rtp_send_packets(sock, &dest_addr);
        vTaskDelay(RTP_SEND_DELAY_MS / portTICK_PERIOD_MS);
    }
    close(sock);
}
