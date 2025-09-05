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
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_spiffs.h"

static const char *TAG = "WW_Comm"; //logging tag

// UART configuration
#define UART_PORT_NUM      UART_NUM_0
#define UART_BAUD_RATE     115200
#define UART_BUF_SIZE      1024
#define UART_QUEUE_LEN     20
// PWM LED configuration
#define NUM_LEDS 6
#define MAX_DUTY 8192 // 13-bit resolution for LEDC
// LED pin configuration
int ledPins[NUM_LEDS] = {2, 4, 5, 18, 19, 21};

static bool discovery_complete = false;
static uint32_t last_peer_activity[10] = {0};
#define PEER_TIMEOUT_MS 30000  // 30 seconds timeout

// UART command structure
typedef struct {
    char command[20];
    char params[50];
} uart_command_t;

static QueueHandle_t uart_queue; // Global variables for UART

#define BLINK_GPIO 48

led_strip_handle_t led_strip;

// PMK (Primary Master Key) and LMK (Local Master Key) for encryption
static const char *PMK = "0123456789abcdef"; // 16-byte PMK
static const char *LMK = "abcdef0123456789"; // 16-byte LMK

// Message types for dynamic peer discovery
typedef enum {
    MSG_PAIRING_REQUEST,
    MSG_PAIRING_RESPONSE,
    MSG_DATA,
    MSG_LED_CONTROL,
    MSG_PWM_LED_CONTROL
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

// Function declarations
void execute_command(char *command, bool uart_feedback);
void process_command_file(const char *file_path);
void process_uart_command(char* command);
bool parse_mac_address(const char *mac_str, uint8_t *mac);
void init_known_peers(void);
void set_led_color(int r, int g, int b, int timeout_ms, int blank_ms);
void set_brightness(int index, int percent);
void send_message(const uint8_t *mac, message_type_t type, const void *data, size_t len);
esp_err_t add_peer(const uint8_t *mac, bool encrypt);
esp_err_t remove_peer(const uint8_t *mac);
void list_peers(void);
void log_mac(const char *prefix, const uint8_t *mac);
void init_pwm_leds(void);
void init_led_strip(void);

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
    // Validate parameters
    r = (r < 0) ? 0 : (r > 255) ? 255 : r;
    g = (g < 0) ? 0 : (g > 255) ? 255 : g;
    b = (b < 0) ? 0 : (b > 255) ? 255 : b;
    timeout_ms = (timeout_ms < 0) ? 0 : timeout_ms;
    blank_ms = (blank_ms < 0) ? 0 : blank_ms;

    ESP_ERROR_CHECK(led_strip_set_pixel(led_strip, 0, r, g, b));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    vTaskDelay(pdMS_TO_TICKS(timeout_ms));
    ESP_ERROR_CHECK(led_strip_clear(led_strip));
    ESP_ERROR_CHECK(led_strip_refresh(led_strip));
    vTaskDelay(pdMS_TO_TICKS(blank_ms));
}

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

void init_pwm_leds() {
    // Configure 3 timers (for 3 columns)
    for (int timer = 0; timer < 3; timer++) {
        ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .timer_num        = (ledc_timer_t)timer,
            .duty_resolution  = LEDC_TIMER_13_BIT,
            .freq_hz          = 5000,
            .clk_cfg          = LEDC_AUTO_CLK
        };
        ledc_timer_config(&ledc_timer);
    }

    // Configure 6 unique channels, 2 LEDs per timer/column
    for (int i = 0; i < NUM_LEDS; i++) {
        int column = i % 3; // column: 0, 1, 2

        ledc_channel_config_t ledc_channel = {
            .channel    = (ledc_channel_t)i,
            .duty       = 0,
            .gpio_num   = ledPins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = (ledc_timer_t)column
        };

        ledc_channel_config(&ledc_channel);
    }
}

static int pwm_delays[NUM_LEDS] = {0};
static TickType_t pwm_off_times[NUM_LEDS] = {0};

void set_brightness(int index, int percent) {
    if (index < 0 || index >= NUM_LEDS) return;
    percent = (percent < 0) ? 0 : (percent > 100) ? 100 : percent; // Clamp percentage between 0 and 100

    uint32_t duty = (percent * MAX_DUTY) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)index, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)index);

    // If percent is 0, clear the delay
    if (percent == 0) {
        pwm_delays[index] = 0;
        pwm_off_times[index] = 0;
    }
    long now = esp_timer_get_time();
    ESP_LOGI(TAG, "LED[%d] -> %d%% (duty=%ld) at %ld us", index, percent, duty, now);
}
void pwm_delay_task(void *pvParameters) {
    // Check stack watermark at the beginning
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "pwm_delay_task stack high water mark: %d", uxHighWaterMark);
    int delay_ms = (int)(intptr_t)pvParameters;
    
    vTaskDelay(delay_ms / portTICK_PERIOD_MS);
    
    // Turn off all LEDs
    for (int j = 0; j < NUM_LEDS; j++) {
        set_brightness(j, 0);
    }
    
    ESP_LOGI(TAG, "PWM LEDs turned off after %d ms", delay_ms);
    // Check stack watermark at the end
    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "pwm_delay_task stack high water mark at end: %d", uxHighWaterMark);
    vTaskDelete(NULL);
}
void pwm_manager_task(void *pvParameters) {
    while (1) {
        TickType_t current_time = xTaskGetTickCount();
        
        for (int i = 0; i < NUM_LEDS; i++) {
            if (pwm_delays[i] > 0 && pwm_off_times[i] > 0 && 
                current_time >= pwm_off_times[i]) {
                set_brightness(i, 0);
                pwm_delays[i] = 0;
                pwm_off_times[i] = 0;
            }
        }
        
        vTaskDelay(100 / portTICK_PERIOD_MS); // Check every 100ms
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

// Add this to your peer management section
static const char *known_macs[] = {
    "F0:9E:9E:1E:4A:84",
    "F0:9E:9E:1E:4A:E4",
    "F0:9E:9E:21:E2:70",
    // Add more MAC addresses as needed
};
static int known_mac_count = sizeof(known_macs) / sizeof(known_macs[0]);

// Function to convert string MAC to uint8_t array
bool parse_mac_address(const char *mac_str, uint8_t *mac) {
    return sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
                  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6;
}

// Initialize known peers during setup
void init_known_peers() {
    for (int i = 0; i < known_mac_count; i++) {
        uint8_t mac[6];
        if (parse_mac_address(known_macs[i], mac)) {
            ESP_LOGI(TAG, "Adding known peer: %s", known_macs[i]);
            add_peer(mac, true);  // Add with encryption
        }
    }
}


// Enhanced add_peer function with retry logic
esp_err_t add_peer_with_retry(const uint8_t *mac, bool encrypt, int max_retries) {
    esp_err_t ret = ESP_FAIL;
    int retry_count = 0;
    
    while (ret != ESP_OK && retry_count < max_retries) {
        ret = add_peer(mac, encrypt);
        if (ret != ESP_OK) {
            retry_count++;
            ESP_LOGW(TAG, "Failed to add peer (attempt %d/%d), retrying...", 
                    retry_count, max_retries);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
    
    return ret;
}

// Handle received ESP-NOW data
void handle_receive(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < sizeof(esp_now_message_t)) {
        ESP_LOGE(TAG, "Invalid message length: %d", len);
        set_led_color(32, 0, 0, 200, 100); // Red flash for error
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
    bool peer_found = false;
    for (int i = 0; i < peer_count; i++) {
        if (memcmp(peers[i].mac, mac, 6) == 0) {
            last_peer_activity[i] = xTaskGetTickCount() * portTICK_PERIOD_MS;
            peer_found = true;
            break;
        }
    }
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
        case MSG_LED_CONTROL:
            ESP_LOGI(TAG, "LED control message received");
            // Parse the LED control parameters
            int r, g, b, timeout_ms, blank_ms;
            if (sscanf((char *)msg->data, "%d,%d,%d,%d,%d", &r, &g, &b, &timeout_ms, &blank_ms) == 5) {
                ESP_LOGI(TAG, "Setting LED: R=%d, G=%d, B=%d, Timeout=%dms, Blank=%dms", 
                         r, g, b, timeout_ms, blank_ms);
                set_led_color(r, g, b, timeout_ms, blank_ms);
            } else {
                ESP_LOGE(TAG, "Invalid LED control format");
            }
            break;
        case MSG_PWM_LED_CONTROL:
            ESP_LOGI(TAG, "PWM LED control message received");
            // Parse the PWM LED control parameters
            int brightness[NUM_LEDS];
            int delay_ms;
            
            if (sscanf((char *)msg->data, "%d,%d,%d,%d,%d,%d,%d", 
                    &brightness[0], &brightness[1], &brightness[2],
                    &brightness[3], &brightness[4], &brightness[5], &delay_ms) == NUM_LEDS + 1) {
                
                ESP_LOGI(TAG, "Setting PWM LEDs: %d,%d,%d,%d,%d,%d with delay %dms", 
                        brightness[0], brightness[1], brightness[2],
                        brightness[3], brightness[4], brightness[5], delay_ms);
                
                // Set all LEDs to specified brightness
                for (int j = 0; j < NUM_LEDS; j++) {
                    set_brightness(j, brightness[j]);
                
                    if (delay_ms > 0) {
                        pwm_delays[j] = delay_ms;
                        pwm_off_times[j] = xTaskGetTickCount() + (delay_ms / portTICK_PERIOD_MS);
                    }
                }
            } else {
                ESP_LOGE(TAG, "Failed to parse PWM LED control data: %s", (char *)msg->data);
                char error_msg[50];
                //snprintf(error_msg, sizeof(error_msg), "PWM parse error: %s", (char *)msg->data);
                send_message(mac, MSG_DATA, error_msg, strlen(error_msg) + 1);
            }
            break;
        default:
            ESP_LOGE(TAG, "Unknown message type: %d", msg->msg_type);
            break;
    }
    if (!peer_found && msg->msg_type != MSG_PAIRING_REQUEST) {
        ESP_LOGW(TAG, "Message from unknown peer, adding to list");
        add_peer(mac, true);
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
// Function to execute a command
void execute_command(char* command, bool uart_feedback) {
    // Remove newline characters
    char *newline = strchr(command, '\n');
    if (newline) *newline = '\0';
    newline = strchr(command, '\r');
    if (newline) *newline = '\0';
        
    if (strstr(command, "discover")) {
        // Manual discovery trigger
        discovery_complete = false;
        uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        send_message(broadcast_mac, MSG_PAIRING_REQUEST, NULL, 0);
        if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Discovery initiated\n", strlen("Discovery initiated\n"));
    } 
    else if (strstr(command, "list")) {
        if (uart_feedback){
            char response[100];
            snprintf(response, sizeof(response), "Registered peers: %d\n", peer_count);
            uart_write_bytes(UART_PORT_NUM, response, strlen(response));
            
            for (int i = 0; i < peer_count; i++) {
                snprintf(response, sizeof(response), "Peer %d: " MACSTR " %s\n", 
                        i, MAC2STR(peers[i].mac), peers[i].paired ? "Paired" : "Unpaired");
                uart_write_bytes(UART_PORT_NUM, response, strlen(response));
            }
        }
    }
    else if (strstr(command, "send ")) {
        // Send custom message: send <mac> <message>
        char* mac_str = strtok(command + 5, " ");
        char* message = strtok(NULL, "");
        
        if (mac_str && message) {
            uint8_t mac[6];
            if (sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
                      &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6) {
                send_message(mac, MSG_DATA, message, strlen(message) + 1);
                if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Message sent\n", strlen("Message sent\n"));
            } else {
                if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Invalid MAC format. Use AA:BB:CC:DD:EE:FF\n", 
                                strlen("Invalid MAC format. Use AA:BB:CC:DD:EE:FF\n"));
            }
        } else {
            if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Usage: send <MAC> <message>\n", 
                            strlen("Usage: send <MAC> <message>\n"));
        }
    }
    else if (strstr(command, "remove ")) {
        // Remove a peer: remove <mac>
        char* mac_str = command + 7;
        uint8_t mac[6];
        if (sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
                  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6) {
            remove_peer(mac);
            if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Peer removed\n", strlen("Peer removed\n"));
        } else {
            if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Invalid MAC format\n", strlen("Invalid MAC format\n"));
        }
    }
    else if (strstr(command, "led ")) {
        // LED control command: led <r> <g> <b> <timeout_ms> <blank_ms> [<mac>]
        // If MAC is provided, send to that device; otherwise, set locally and broadcast to all peers
        int r, g, b, timeout_ms, blank_ms;
        char mac_str[20] = {0};
        
        // Parse parameters
        int parsed = sscanf(command + 4, "%d %d %d %d %d %17s", 
                           &r, &g, &b, &timeout_ms, &blank_ms, mac_str);
        
        if (parsed >= 5) {
            // Format the LED control data
            char led_data[50];
            snprintf(led_data, sizeof(led_data), "%d,%d,%d,%d,%d", r, g, b, timeout_ms, blank_ms);
            
            if (parsed == 6) {
                // Send to specific MAC
                uint8_t mac[6];
                if (sscanf(mac_str, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx", 
                          &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5]) == 6) {
                    send_message(mac, MSG_LED_CONTROL, led_data, strlen(led_data) + 1);
                    if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "LED command sent to specific device\n", 
                                    strlen("LED command sent to specific device\n"));
                } else {
                    if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Invalid MAC format\n", strlen("Invalid MAC format\n"));
                }
            } else {
                // Set locally and broadcast to all peers
                set_led_color(r, g, b, timeout_ms, blank_ms);
                
                // Send to all paired peers
                for (int i = 0; i < peer_count; i++) {
                    if (peers[i].paired) {
                        send_message(peers[i].mac, MSG_LED_CONTROL, led_data, strlen(led_data) + 1);
                    }
                }
                if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "LED command set locally and sent to all peers\n", 
                                strlen("LED command set locally and sent to all peers\n"));
            }
        } else {
            if (uart_feedback) uart_write_bytes(UART_PORT_NUM, 
                            "Usage: led <r> <g> <b> <timeout_ms> <blank_ms> [<MAC>]\n"
                            "Example: led 255 0 0 1000 500\n"
                            "Example: led 0 255 0 500 200 AA:BB:CC:DD:EE:FF\n", 
                            strlen("Usage: led <r> <g> <b> <timeout_ms> <blank_ms> [<MAC>]\n"
                                   "Example: led 255 0 0 1000 500\n"
                                   "Example: led 0 255 0 500 200 AA:BB:CC:DD:EE:FF\n"));
        }
    }
    else if (strstr(command, "me")){
        // Get MAC address of the ESP32
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);
        if (uart_feedback) printf("MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
    else if (strstr(command, "addmac ")) {
        // Add a MAC to known list: addmac <mac>
        char* mac_str = command + 7;
        uint8_t mac[6];
        if (parse_mac_address(mac_str, mac)) {
            // Check if already in known MACs
            bool already_exists = false;
            for (int i = 0; i < known_mac_count; i++) {
                uint8_t known_mac[6];
                if (parse_mac_address(known_macs[i], known_mac) && 
                    memcmp(known_mac, mac, 6) == 0) {
                    already_exists = true;
                    break;
                }
            }
            
            if (!already_exists) {
                // Add to known MACs (in a real implementation, you might want to save to NVS)
                // For now, just add as a peer
                add_peer(mac, true);
                if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "MAC added and peer connection attempted\n", 
                                strlen("MAC added and peer connection attempted\n"));
            } else {
                if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "MAC already known\n", 
                                strlen("MAC already known\n"));
            }
        } else {
            if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Invalid MAC format\n", strlen("Invalid MAC format\n"));
        }
    }
    else if (strstr(command, "pwm ")) {
        ESP_LOGI(TAG, "Processing PWM LED command: %s", command);
        // PWM LED control: pwmled <b1> <b2> <b3> <b4> <b5> <b6> <delay_ms> [<MAC>]
        int brightness[NUM_LEDS] = {0};
        int delay_ms = 0;
        char mac_str[20] = {0};
        
        char *token = strtok(command + 4, " \t\n");
        int i = 0;
        int param_count = 0;
        
        // Parse all parameters
        while (token != NULL && param_count <= NUM_LEDS + 1) {
            if (param_count < NUM_LEDS) {
                brightness[param_count] = atoi(token);
                ESP_LOGI(TAG, "Brightness[%d] = %d", param_count, brightness[param_count]);
            } else if (param_count == NUM_LEDS) {
                delay_ms = atoi(token);
                ESP_LOGI(TAG, "Delay_ms = %d", delay_ms);
            } else {
                strncpy(mac_str, token, sizeof(mac_str) - 1);
                ESP_LOGI(TAG, "MAC = %s", mac_str);
            }
            param_count++;
            token = strtok(NULL, " \t\n");
        }
        
        // Validate input
        if (param_count < NUM_LEDS + 1) {
            if (uart_feedback) {
            uart_write_bytes(UART_PORT_NUM, "Invalid PWM LED command format\n", 
                            strlen("Invalid PWM LED command format\n"));
            return;
            }
        }
        // Format the PWM LED control data
        char pwm_data[50];
        snprintf(pwm_data, sizeof(pwm_data), "%d,%d,%d,%d,%d,%d,%d",
                brightness[0], brightness[1], brightness[2],
                brightness[3], brightness[4], brightness[5], delay_ms);
        
        if (strlen(mac_str) > 0) {
            // Send to specific MAC address
            uint8_t mac[6];
            if (parse_mac_address(mac_str, mac)) {
                send_message(mac, MSG_PWM_LED_CONTROL, pwm_data, strlen(pwm_data) + 1);
                if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "PWM LED command sent to remote device\n", 
                                strlen("PWM LED command sent to remote device\n"));
            } else {
                if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Invalid MAC format\n", 
                                strlen("Invalid MAC format\n"));
            }
        } else {
            // Execute locally
            for (int j = 0; j < NUM_LEDS; j++) {
                set_brightness(j, brightness[j]);
            }
            
            if (delay_ms > 0) {
                // Create task to handle delayed turn-off
                xTaskCreate(pwm_delay_task, "pwm_delay", 4096, 
                        (void*)(intptr_t)delay_ms, 5, NULL);
            }
            
            if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "PWM LEDs set locally\n", 
                            strlen("PWM LEDs set locally\n"));
        }
    }
    else if (strstr(command, "testpwm")) {
        // Simple test command
        for (int i = 0; i < NUM_LEDS; i++) {
            set_brightness(i, 50);
        }
        if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "All LEDs set to 50%\n", 
                        strlen("All LEDs set to 50%\n"));
    }
    else if (strstr(command, "execute ")) {
        char *file_path = command + 8;  // Skip "execute "
        process_command_file(file_path);  // Function to process the file
    }
    else if (strstr(command, "help")) {
        // Show help
        const char *help_text = 
            "Available commands:\n"
            "  discover - Initiate peer discovery\n"
            "  list - List all known peers\n"
            "  send <MAC> <message> - Send message to peer\n"
            "  remove <MAC> - Remove a peer\n"
            "  me - Gives MAC address of Current Device\n"
            "  led <r> <g> <b> <timeout_ms> <blank_ms> [<MAC>] - Control LED (local or remote)\n"
            "  pwm <b1> <b2> <b3> <b4> <b5> <b6> <delay_ms> [<MAC>] - Control PWM LEDs (local or remote)\n"
            "  testpwm - test the pwm-leds"
            "  addmac <MAC>- Add a MAC to known list\n"
            "  execute <file_path> - Execute a COMMANDs file </spiffs/commands.txt>\n"
            "  help - Show this help\n";
        uart_write_bytes(UART_PORT_NUM, help_text, strlen(help_text));
    }
    else {
        if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Unknown command. Type 'help' for available commands.\n", 
                        strlen("Unknown command. Type 'help' for available commands.\n"));
    }
}
// Process UART commands
void process_uart_command(char* command) {
   execute_command(command, true); 
}
// opens the file, reads each line, and executes the commands silently
void process_command_file(const char *file_path) {
    FILE *file = fopen(file_path, "r");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file: %s", file_path);
        uart_write_bytes(UART_PORT_NUM, "Error: File not found\n", strlen("Error: File not found\n"));
        return;
    }

    char line[100];
    while (fgets(line, sizeof(line), file) != NULL) {
        // Remove newline characters
        char *newline = strchr(line, '\n');
        if (newline) *newline = '\0';
        newline = strchr(line, '\r');
        if (newline) *newline = '\0';

        // Skip empty lines or comments (lines starting with #)
        if (strlen(line) == 0 || line[0] == '#') {
            continue;
        }

        ESP_LOGI(TAG, "Executing command: %s", line);
        execute_command(line, false);  // Execute without UART feedback
    }

    fclose(file);
    uart_write_bytes(UART_PORT_NUM, "File execution completed\n", strlen("File execution completed\n"));
}

// UART task function
void uart_task(void *pvParameters) {
    uart_event_t event;
    uint8_t* data = (uint8_t*) malloc(UART_BUF_SIZE);
    
    while (1) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            switch (event.type) {
                case UART_DATA:
                    int len = uart_read_bytes(UART_PORT_NUM, data, event.size, portMAX_DELAY);
                    data[len] = '\0'; // Null terminate the string
                    
                    // Process UART command
                    process_uart_command((char*)data);
                    break;
                    
                case UART_FIFO_OVF:
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                case UART_BUFFER_FULL:
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart_queue);
                    break;
                    
                default:
                    break;
            }
        }
    }
    free(data);
    vTaskDelete(NULL);
}

// Initialize UART
void init_uart() {
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, UART_BUF_SIZE, UART_BUF_SIZE, UART_QUEUE_LEN, &uart_queue, 0));
    esp_vfs_dev_uart_use_driver(UART_PORT_NUM);
    
    // Create UART task
    xTaskCreate(uart_task, "uart_task", 8192, NULL, 10, NULL);
    
    // Print welcome message
    const char *welcome = "\nESP-NOW Communication System\nType 'help' for available commands\n\n";
    uart_write_bytes(UART_PORT_NUM, welcome, strlen(welcome));
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

void test_leds() {
    ESP_LOGI(TAG, "Testing all LEDs");
    
    // Test each LED individually
    for (int i = 0; i < NUM_LEDS; i++) {
        ESP_LOGI(TAG, "Testing LED %d on GPIO %d", i, ledPins[i]);
        set_brightness(i, 100);
        vTaskDelay(500 / portTICK_PERIOD_MS);
        set_brightness(i, 0);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
    
    // Test all LEDs together
    ESP_LOGI(TAG, "Testing all LEDs together");
    for (int i = 0; i < NUM_LEDS; i++) {
        set_brightness(i, 50);
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    for (int i = 0; i < NUM_LEDS; i++) {
        set_brightness(i, 0);
    }
    
    ESP_LOGI(TAG, "LED test complete");
}

// Initialize SPIFFS
void init_sniffs(){
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }
}

void app_main(void){
    init_sniffs();
    init_led_strip();
    init_pwm_leds();
    init_uart();
    test_leds();
    init_esp_now();
    init_known_peers();
    set_led_color(0, 32, 0, 1000, 0); // Solid green for ready state
    // Start dynamic discovery by sending a pairing request
    uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    send_message(broadcast_mac, MSG_PAIRING_REQUEST, NULL, 0);
    xTaskCreate(pwm_manager_task, "pwm_manager", 4096, NULL, 5, NULL);
    // Main loop: send data periodically
    int counter = 0;

    while (1) {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        // Only send if we have paired peers
        for (int i = 0; i < peer_count; i++) {
            if (peers[i].paired && (xTaskGetTickCount() * portTICK_PERIOD_MS - last_peer_activity[i] > PEER_TIMEOUT_MS)) {
                ESP_LOGW(TAG, "Peer timeout: " MACSTR, MAC2STR(peers[i].mac));
                
                // Instead of removing, try to re-establish connection
                uint8_t mac[6];
                memcpy(mac, peers[i].mac, 6);
                remove_peer(peers[i].mac);
                add_peer(mac, true);  // Try to reconnect
                last_peer_activity[i] = xTaskGetTickCount() * portTICK_PERIOD_MS;
            }
        }
        if (peer_count > 0) {
            char data[50];
            snprintf(data, sizeof(data), "Hello from ESP32! Count: %d", counter++);
            
            for (int i = 0; i < peer_count; i++) {
                if (peers[i].paired) {
                    send_message(peers[i].mac, MSG_DATA, data, strlen(data) + 1);
                }
            }
        } else {
            // If no peers, try to reinitialize known peers
            ESP_LOGW(TAG, "No peers connected, reinitializing...");
            init_known_peers();
            set_led_color(32, 16, 0, 500, 500); // Orange flash for discovery
        }
    }
}