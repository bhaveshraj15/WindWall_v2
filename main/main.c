#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "esp_netif.h"

static const char *TAG = "WW_Comm"; //logging tag

#define BLINK_GPIO 48

led_strip_handle_t led_strip;

void init_led_strip(){
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    ESP_ERROR_CHECK(led_strip_clear(led_strip)); // Clear the strip initially
} 

void set_led_color(int r, int g, int b, int timeout_ms, int blank_ms) {
    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, b));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    vTaskDelay(pdMS_TO_TICKS(timeout_ms));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    vTaskDelay(pdMS_TO_TICKS(blank_ms));
}

// PMK (Primary Master Key) and LMK (Local Master Key) for encryption
static const char *PMK = "0123456789abcdef"; // 16-byte PMK
static const char *LMK = "abcdef0123456789"; // 16-byte LMK

// Message types for dynamic peer discovery
typedef enum {
    MSG_PAIRING_REQUEST,
    MSG_PAIRING_RESPONSE,
    MSG_DATA
} message_type_t;

// Message structure
typedef struct __attribute__((packed)) {
    message_type_t msg_type;
    uint32_t sequence_num;
    uint8_t src_mac[6];
    uint8_t data[236]; // Flexible data field
} esp_now_message_t;

static uint32_t sequence_counter = 0;

// Peer device information
typedef struct {
    uint8_t mac[6];
    bool paired;
} peer_device_t;

// List of peers (max 10 for example)
static peer_device_t peers[10];
static int peer_count = 0;

// Function to log MAC address
void log_mac(const char *prefix, const uint8_t *mac) {
    ESP_LOGI(TAG, "%s %02x:%02x:%02x:%02x:%02x:%02x", prefix,
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// Add a peer device
esp_err_t add_peer(const uint8_t *mac, bool encrypt) {
    // Check if peer already exists
    for (int i = 0; i < peer_count; i++) {
        if (memcmp(peers[i].mac, mac, 6) == 0) {
            ESP_LOGW(TAG, "Peer already exists");
            return ESP_OK; // Peer already exists
        }
    }
    // Check if we have space for more peers
    if (peer_count >= 10) {
        ESP_LOGE(TAG, "Peer list full");
        return ESP_ERR_NO_MEM;
    }
    esp_now_peer_info_t peer_info = {
        .channel = 1,
        .encrypt = encrypt
    };
    memcpy(peer_info.peer_addr, mac, 6);
    if (encrypt) {
        memcpy(peer_info.lmk, LMK, 16);
    }
    esp_err_t ret = esp_now_add_peer(&peer_info);
    if (ret == ESP_OK) {
        // Add to our peer list
        memcpy(peers[peer_count].mac, mac, 6);
        peers[peer_count].paired = !encrypt; // If encrypted, not paired until response
        peer_count++;
        log_mac("Peer added:", mac);
        set_led_color(0, 32, 0, 200, 100); // Green flash for new peer
    } else {
        log_mac("Failed to add peer:", mac);
        set_led_color(32, 0, 0, 200, 100); // Red flash for error
    }
    return ret;
}

// Function to remove a peer
esp_err_t remove_peer(const uint8_t *mac) {
    esp_err_t ret = esp_now_del_peer(mac);
    if (ret == ESP_OK) {
        // Remove from our peer list
        for (int i = 0; i < peer_count; i++) {
            if (memcmp(peers[i].mac, mac, 6) == 0) {
                // Shift remaining peers
                for (int j = i; j < peer_count - 1; j++) {
                    memcpy(peers[j].mac, peers[j+1].mac, 6);
                    peers[j].paired = peers[j+1].paired;
                }
                peer_count--;
                break;
            }
        }
        log_mac("Peer removed:", mac);
    } else {
        log_mac("Failed to remove peer:", mac);
    }
    return ret;
}

// Function to list all peers
void list_peers() {
    ESP_LOGI(TAG, "Registered peers (%d):", peer_count);
    for (int i = 0; i < peer_count; i++) {
        log_mac(peers[i].paired ? "  Paired: " : "  Unpaired: ", peers[i].mac);
    }
}

// Send ESP-NOW message
void send_message(const uint8_t *mac, message_type_t type, const void *data, size_t len) {
    if (len > sizeof(esp_now_message_t)) {
        ESP_LOGE(TAG, "Message too long: %d > %d", len, sizeof(esp_now_message_t));
        return;
    }
    esp_now_message_t msg;
    msg.msg_type = type;
    msg.sequence_num = sequence_counter++;
    esp_read_mac(msg.src_mac, ESP_MAC_WIFI_STA);
    if (len > 0) {
        memcpy(msg.data, data, len);
    }
    esp_err_t ret = esp_now_send(mac, (uint8_t *)&msg, sizeof(msg));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Send error: %d", ret);
        set_led_color(32, 16, 0, 200, 100); // Orange flash for send error
    }
}

// Handle received ESP-NOW data
void handle_receive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < sizeof(esp_now_message_t)) {
        ESP_LOGE(TAG, "Invalid message length: %d", len);
        return;
    }
    esp_now_message_t *msg = (esp_now_message_t *)data;
    // Ignore messages from ourselves
    uint8_t my_mac[6];
    esp_read_mac(my_mac, ESP_MAC_WIFI_STA);
    if (memcmp(msg->src_mac, my_mac, 6) == 0) {
        return;
    }
    log_mac("Received from:", mac);
    ESP_LOGI(TAG, "Sequence: %lu, Type: %d", msg->sequence_num, msg->msg_type);

    switch (msg->msg_type) {
        case MSG_PAIRING_REQUEST:
            ESP_LOGI(TAG, "Pairing request received");
            // Add the sender as a peer
            add_peer(mac, true);
            // Send pairing response
            send_message(mac, MSG_PAIRING_RESPONSE, NULL, 0);
            set_led_color(0, 0, 32, 200, 100); // Blue flash for pairing
            break;
        case MSG_PAIRING_RESPONSE:
            ESP_LOGI(TAG, "Pairing response received");
            // Mark peer as paired
            for (int i = 0; i < peer_count; i++) {
                if (memcmp(peers[i].mac, mac, 6) == 0) {
                    peers[i].paired = true;
                    ESP_LOGI(TAG, "Peer successfully paired");
                    set_led_color(0, 32, 0, 500, 100); // Green flash for successful pairing
                    break;
                }
            }
            break;
        case MSG_DATA:
            ESP_LOGI(TAG, "Data message: %s", (char *)msg->data);
            set_led_color(32, 32, 32, 100, 50); // White flash for data received
            break;
        default:
            ESP_LOGE(TAG, "Unknown message type: %d", msg->msg_type);
            break;
    }
}

// ESP-NOW receive callback
void esp_now_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    handle_receive(info->src_addr, data, len);
}

// ESP-NOW send callback
void esp_now_send_cb(const uint8_t *mac, esp_now_send_status_t status) {
    log_mac("Send status:", mac);
    ESP_LOGI(TAG, "Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Initialize ESP-NOW
void init_esp_now() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb((esp_now_recv_cb_t)esp_now_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb((esp_now_send_cb_t)esp_now_send_cb));

    // Set PMK for encryption
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)PMK));

    // Add broadcast peer for discovery (unencrypted)
    uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    add_peer(broadcast_mac, false);

    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
}

void app_main(void){
    init_led_strip();
    init_esp_now();
    set_led_color(0, 32, 0, 1000, 0); // Solid green for ready state
    // Start dynamic discovery by sending a pairing request
    uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    send_message(broadcast_mac, MSG_PAIRING_REQUEST, NULL, 0);

    // Main loop: send data periodically
    int counter = 0;

    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        // Only send if we have paired peers
        bool has_paired_peers = false;
        for (int i = 0; i < peer_count; i++) {
            if (peers[i].paired) {
                has_paired_peers = true;
                break;
            }
        }
        if (has_paired_peers) {
            char data[50];
            snprintf(data, sizeof(data), "Hello from ESP32! Count: %d", counter++);
            
            for (int i = 0; i < peer_count; i++) {
                if (peers[i].paired) {
                    send_message(peers[i].mac, MSG_DATA, data, strlen(data) + 1);
                }
            }
            set_led_color(32, 32, 32, 500, 500); // White flash when sending data
        } else {
                // No paired peers, try to discover again
                ESP_LOGW(TAG, "No paired peers, sending discovery request");
                send_message(broadcast_mac, MSG_PAIRING_REQUEST, NULL, 0);
                set_led_color(32, 16, 0, 500, 500); // Orange flash for discovery
        }
    }
}