#include "wifi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

#define TAG "WIFI"

#define AP_SSID      "Jutikliai"
#define AP_PASS      ""
#define AP_PORT      12345
#define MAX_CONN     2

volatile bool tcpConnected = false;
volatile bool wifiRunning = false;

int sock = -1;

void startWiFiSoftAP() {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    wifi_config_t ap_config = {};
    strcpy((char *)ap_config.ap.ssid, AP_SSID);
    ap_config.ap.ssid_len = strlen(AP_SSID);
    ap_config.ap.max_connection = MAX_CONN;

    if (strlen(AP_PASS) == 0) {
        ap_config.ap.authmode = WIFI_AUTH_OPEN;
    } else {
        strcpy((char *)ap_config.ap.password, AP_PASS);
        ap_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    }
    esp_wifi_set_mode(WIFI_MODE_AP);
    esp_wifi_set_config(WIFI_IF_AP, &ap_config);
    esp_wifi_start();
}

bool connectToTCPServer() {
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(AP_PORT);
    server_addr.sin_addr.s_addr = inet_addr("192.168.4.2");

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
    if (sock < 0) {
        return false;
    }

    int err = connect(sock, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket failed: %d", errno);
        close(sock);
        sock = -1;
        return false;
    }

    int flags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, flags | O_NONBLOCK);

    tcpConnected = true; 
    ESP_LOGI("TCP", "Connected to TCP server");

    return true;
}


void sendPacket(const void *data, size_t len) {
    if (!tcpConnected || sock < 0 || data == nullptr || len == 0) return;

    uint32_t packet_len = (uint32_t)len;
    if (send(sock, &packet_len, sizeof(packet_len), 0) != sizeof(packet_len)) {
        tcpConnected = false;
        close(sock);
        sock = -1;
        return;
    }

    if (send(sock, data, len, 0) != (int)len) {
        tcpConnected = false;
        close(sock);
        sock = -1;
    }
}

void enableWiFi() {
    startWiFiSoftAP();
    wifiRunning = true;
    ESP_LOGI(TAG, "Wi-Fi enabled");
}

void disableWiFi() {
    if (tcpConnected) {
        close(sock);
        sock = -1;
        tcpConnected = false;
        ESP_LOGI(TAG, "TCP connection closed");
    }

    if (wifiRunning) {
        esp_wifi_stop();
        esp_wifi_deinit();
        wifiRunning = false;
        ESP_LOGI(TAG, "Wi-Fi disabled");
    }
}




