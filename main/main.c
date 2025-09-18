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

// Motor PWM configuration
#define MOTOR_PWM_FREQ_HZ   50     // Standard for ESCs
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_12_BIT
#define MOTOR_MAX_DUTY      4095   // 12-bit resolution

// PWM MOTOR configuration
#define NUM_MOTORS 6

// Motor pin configuration
int motorPins[NUM_MOTORS] = {5,6,7,15,16,17};

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
    MSG_MOTOR_CONTROL
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
void init_led_strip(void);
void init_motor_pwm();
void set_motor_pulse(int index, uint16_t pulse_us);
void motor_delay_task(void *pvParameters);

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

void init_motor_pwm() {
    // Configure timer for motors (50Hz for ESCs)
    ledc_timer_config_t motor_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = MOTOR_PWM_RESOLUTION,
        .timer_num        = LEDC_TIMER_1,        // Use a different timer
        .freq_hz          = MOTOR_PWM_FREQ_HZ,  // Standard ESC frequency
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer));

    // Configure channels for each motor
    for (int i = 0; i < NUM_MOTORS; i++) {
        ledc_channel_config_t motor_channel = {
            .channel    = (ledc_channel_t)(i), // Start from channel 6
            .duty       = 0,
            .gpio_num   = motorPins[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_1
        };
        ledc_channel_config(&motor_channel);
        
        // Start with motors disarmed
        set_motor_pulse(i, 1000);
    }
}

// Function to set motor pulse width
void set_motor_pulse(int index, uint16_t pulse_us) {
    if (index < 0 || index >= NUM_MOTORS) return;
    
    // Validate pulse width (1000-2000 µs standard for ESCs)
    pulse_us = (pulse_us < 1000) ? 1000 : (pulse_us > 2000) ? 2000 : pulse_us;
    
    // Convert µs to duty cycle (12-bit resolution, 50Hz = 20000µs period)
    uint32_t duty = (pulse_us * MOTOR_MAX_DUTY) / 20000;
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(index), duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)(index));
    
    ESP_LOGI(TAG, "Motor[%d] -> %d µs (duty=%lu)", index, pulse_us, duty);
}
void motor_delay_task(void *pvParameters) {
    int delay_time = (int)(intptr_t)pvParameters;
    vTaskDelay(delay_time / portTICK_PERIOD_MS);
    
    // Return all motors to idle (1000 µs)
    for (int i = 0; i < NUM_MOTORS; i++) {
        set_motor_pulse(i, 1000);
    }
    
    ESP_LOGI(TAG, "All motors returned to idle after %d ms", delay_time);
    vTaskDelete(NULL);
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
    "A0:85:E3:E8:00:EC",
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
        case MSG_MOTOR_CONTROL:
            ESP_LOGI(TAG, "Multi-motor control message received");
            // Parse the motor control parameters using strtok for robustness
            int motor_values[NUM_MOTORS];
            int delay_time;
            char *data_str = (char *)msg->data;
            
            // Use strtok to parse comma-separated values
            char *token = strtok(data_str, ",");
            int i = 0;
            for (i = 0; i < NUM_MOTORS && token != NULL; i++) {
                motor_values[i] = atoi(token);
                token = strtok(NULL, ",");
            }
            
            if (token != NULL && i == NUM_MOTORS) {
                delay_time = atoi(token);
                
                ESP_LOGI(TAG, "Setting motors: %d,%d,%d,%d,%d,%d for %d ms",
                        motor_values[0], motor_values[1], motor_values[2],
                        motor_values[3], motor_values[4], motor_values[5], delay_time);
                
                // Set all motors
                for (int j = 0; j < NUM_MOTORS; j++) {
                    set_motor_pulse(j, motor_values[j]);
                }
                
                // If delay_time > 0, create a task to return to idle
                if (delay_time > 0) {
                    xTaskCreate(motor_delay_task, "motor_delay", 4096, 
                            (void*)(intptr_t)delay_time, 3, NULL);
                }
            } else {
                ESP_LOGE(TAG, "Invalid motor control format: %s", (char *)msg->data);
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
        // LED control command: led <r> <g> <b> <timeout_ms> <blank_ms> [mac1] [mac2] [mac3] ...
        int r, g, b, timeout_ms, blank_ms;
        
        // Parse the first 5 parameters
        char *token = strtok(command + 4, " \t\n");
        int param_count = 0;
        
        // Parse the required parameters
        while (token != NULL && param_count < 5) {
            switch(param_count) {
                case 0: r = atoi(token); break;
                case 1: g = atoi(token); break;
                case 2: b = atoi(token); break;
                case 3: timeout_ms = atoi(token); break;
                case 4: blank_ms = atoi(token); break;
            }
            param_count++;
            token = strtok(NULL, " \t\n");
        }
        
        // Validate we have all required parameters
        if (param_count < 5) {
            if (uart_feedback) {
                uart_write_bytes(UART_PORT_NUM, 
                    "Invalid LED command format\n"
                    "Usage: led <r> <g> <b> <timeout_ms> <blank_ms> [mac1] [mac2] [mac3] ...\n"
                    "Example: led 255 0 0 1000 500\n"
                    "Example: led 0 255 0 500 200 AA:BB:CC:DD:EE:FF\n"
                    "Example: led 0 0 255 300 100 AA:BB:CC:DD:EE:FF 11:22:33:44:55:66\n",
                    strlen("Invalid LED command format\n"
                        "Usage: led <r> <g> <b> <timeout_ms> <blank_ms> [mac1] [mac2] [mac3] ...\n"
                        "Example: led 255 0 0 1000 500\n"
                        "Example: led 0 255 0 500 200 AA:BB:CC:DD:EE:FF\n"
                        "Example: led 0 0 255 300 100 AA:BB:CC:DD:EE:FF 11:22:33:44:55:66\n"));
            }
            return;
        }
        
        // Format the LED control data
        char led_data[50];
        snprintf(led_data, sizeof(led_data), "%d,%d,%d,%d,%d", r, g, b, timeout_ms, blank_ms);
        
        // Check if any MAC addresses were provided
        bool has_mac_addresses = (token != NULL);
        bool executed_locally = false;
        
        // Process MAC addresses if provided
        while (token != NULL) {
            uint8_t mac[6];
            if (parse_mac_address(token, mac)) {
                send_message(mac, MSG_LED_CONTROL, led_data, strlen(led_data) + 1);
                if (uart_feedback) {
                    char response[100];
                    snprintf(response, sizeof(response), 
                            "LED command sent to: %s\n", token);
                    uart_write_bytes(UART_PORT_NUM, response, strlen(response));
                }
            } else {
                if (uart_feedback) {
                    char response[100];
                    snprintf(response, sizeof(response), 
                            "Invalid MAC format: %s\n", token);
                    uart_write_bytes(UART_PORT_NUM, response, strlen(response));
                }
            }
            token = strtok(NULL, " \t\n");
        }
        
        // If no MAC addresses were provided, execute locally and broadcast to all peers
        if (!has_mac_addresses) {
            // Set LED locally
            set_led_color(r, g, b, timeout_ms, blank_ms);
            
            // Send to all paired peers
            for (int i = 0; i < peer_count; i++) {
                if (peers[i].paired) {
                    send_message(peers[i].mac, MSG_LED_CONTROL, led_data, strlen(led_data) + 1);
                }
            }
            
            if (uart_feedback) {
                char response[100];
                snprintf(response, sizeof(response), 
                        "LED command set locally and sent to all peers: R=%d, G=%d, B=%d, Timeout=%dms, Blank=%dms\n",
                        r, g, b, timeout_ms, blank_ms);
                uart_write_bytes(UART_PORT_NUM, response, strlen(response));
            }
            executed_locally = true;
        }
        
        // Update the help text to reflect the new format
        if (uart_feedback && !executed_locally && !has_mac_addresses) {
            uart_write_bytes(UART_PORT_NUM, 
                "LED command executed remotely\n", 
                strlen("LED command executed remotely\n"));
        }
    }
    else if (strstr(command, "motor ")) {
        // Motor control: motor <m1> <m2> <m3> <m4> <m5> <m6> <delay time> [mac1] [mac2] [mac3] ...
        int motor_values[NUM_MOTORS];
        int delay_time;
        
        // Parse parameters
        char *token = strtok(command + 6, " \t\n");
        int param_count = 0;
        
        // Parse motor values and delay time
        while (token != NULL && param_count <= NUM_MOTORS) {
            if (param_count < NUM_MOTORS) {
                motor_values[param_count] = atoi(token);
                // Validate motor values (1000-2000 µs)
                if (motor_values[param_count] < 1000) motor_values[param_count] = 1000;
                if (motor_values[param_count] > 2000) motor_values[param_count] = 2000;
            } else if (param_count == NUM_MOTORS) {
                delay_time = atoi(token);
            } 
            param_count++;
            token = strtok(NULL, " \t\n");
        }
        
        // Validate input
        if (param_count < NUM_MOTORS + 1) {
            if (uart_feedback) {
                uart_write_bytes(UART_PORT_NUM, 
                    "Invalid motor command format\n"
                    "Usage: motor <m1> <m2> <m3> <m4> <m5> <m6> <delay time> [mac1] [mac2] [mac3] ...\n"
                    "Motor values: 1000-2000 µs\n",
                    strlen("Invalid motor command format\n"
                        "Usage: motor <m1> <m2> <m3> <m4> <m5> <m6> <delay time> [mac1] [mac2] [mac3] ...\n"
                        "Motor values: 1000-2000 µs\n"));
            }
            return;
        }
        
        // Format the motor control data
        char motor_data[100];
        snprintf(motor_data, sizeof(motor_data), "%d,%d,%d,%d,%d,%d,%d",
                motor_values[0], motor_values[1], motor_values[2],
                motor_values[3], motor_values[4], motor_values[5], delay_time);
        
        // Check if any MAC addresses were provided
        bool has_mac_addresses = (token != NULL);
        bool executed_locally = false;

        // Process MAC addresses if provided
        while (token != NULL) {
            // Send to specific MAC address
            uint8_t mac[6];
            if (parse_mac_address(token, mac)) {
                send_message(mac, MSG_MOTOR_CONTROL, motor_data, strlen(motor_data) + 1);
                if (uart_feedback) {
                    char response[100];
                    snprintf(response, sizeof(response), 
                            "Motor command sent to remote device: %s\n", token);
                    uart_write_bytes(UART_PORT_NUM, response, strlen(response));
                }
            } 
            token = strtok(NULL, " \t\n");
        } 
        if (!has_mac_addresses) {
            // Execute locally
            for (int j = 0; j < NUM_MOTORS; j++) {
                set_motor_pulse(j, motor_values[j]);
            }
            
            if (delay_time > 0) {
                // Create task to handle delayed return to idle
                xTaskCreate(motor_delay_task, "motor_delay", 4096, 
                        (void*)(intptr_t)delay_time, 5, NULL);
            }
            
            if (uart_feedback) {
                char response[100];
                snprintf(response, sizeof(response), 
                        "Motors set to: %d,%d,%d,%d,%d,%d for %d ms\n",
                        motor_values[0], motor_values[1], motor_values[2],
                        motor_values[3], motor_values[4], motor_values[5], delay_time);
                uart_write_bytes(UART_PORT_NUM, response, strlen(response));
            }
            executed_locally = true;
        }
        // Update the help text to reflect the new format
        if (uart_feedback && !executed_locally && !has_mac_addresses) {
            uart_write_bytes(UART_PORT_NUM, 
                "Motor command executed remotely\n", 
                strlen("Motor command executed remotely\n"));
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
    else if (strstr(command, "execute ")) {
        char *file_path = command + 8;  // Skip "execute "
        process_command_file(file_path);  // Function to process the file
    }
    else if (strstr(command, "hold ")) {
        // Hold command: hold <time_ms>
        int hold_time = atoi(command + 5); // Skip "hold "
        if (hold_time > 0) {
            if (uart_feedback) {
                char response[30];
                snprintf(response, sizeof(response), "Holding for %d ms\n", hold_time);
                uart_write_bytes(UART_PORT_NUM, response, strlen(response));
            }
            vTaskDelay(hold_time / portTICK_PERIOD_MS);
        } else {
            if (uart_feedback) {
                uart_write_bytes(UART_PORT_NUM, "Invalid hold time\n", strlen("Invalid hold time\n"));
            }
        }
    }
    else if (strstr(command, "loop ")) {
        if (uart_feedback) {
            uart_write_bytes(UART_PORT_NUM, 
                            "Loop command only supported in command files\n", 
                            strlen("Loop command only supported in command files\n"));
        }
    }
    else if (strcmp(command, "end") == 0) {
        if (uart_feedback) {
            uart_write_bytes(UART_PORT_NUM, 
                            "End command only supported in command files\n", 
                            strlen("End command only supported in command files\n"));
        }
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
            "  motor <m1> <m2> <m3> <m4> <m5> <m6> <delay time> [mac] - Control multiple motors (1000-2000 µs)\n"
            "  addmac <MAC>- Add a MAC to known list\n"
            "  execute <file_path> - Execute a COMMANDs file </spiffs/commands.txt>\n"
            "  hold <time_ms> - Pause execution for specified milliseconds\n"
            "  loop <count> - Start a loop block (must be followed by 'end')\n"
            "  end - End a loop block\n"
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

    // Stack for loop management (support up to 5 nested loops)
    int loop_counts[5] = {0};
    long loop_positions[5] = {0};
    int loop_depth = -1;

    char line[100];
    long current_pos;
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

        // Check for loop command
        if (strstr(line, "loop ")) {
            if (loop_depth >= 4) {
                ESP_LOGE(TAG, "Maximum loop nesting exceeded");
                uart_write_bytes(UART_PORT_NUM, "Error: Too many nested loops\n", 
                                strlen("Error: Too many nested loops\n"));
                break;
            }
            
            loop_depth++;
            loop_counts[loop_depth] = atoi(line + 5); // Get loop count
            current_pos = ftell(file); // Remember current position
            loop_positions[loop_depth] = current_pos; // Next line is start of loop
            
            if (loop_counts[loop_depth] <= 0) {
                ESP_LOGE(TAG, "Invalid loop count: %s", line + 5);
                uart_write_bytes(UART_PORT_NUM, "Error: Invalid loop count\n", 
                                strlen("Error: Invalid loop count\n"));
                break;
            }
            
            ESP_LOGI(TAG, "Starting loop %d, count: %d", loop_depth, loop_counts[loop_depth]);
            continue;
        }
        
        // Check for end command
        if (strcmp(line, "end") == 0) {
            if (loop_depth < 0) {
                ESP_LOGE(TAG, "End without loop");
                uart_write_bytes(UART_PORT_NUM, "Error: End without loop\n", 
                                strlen("Error: End without loop\n"));
                break;
            }
            
            loop_counts[loop_depth]--;
            
            if (loop_counts[loop_depth] > 0) {
                // Repeat the loop by seeking back to the start position
                fseek(file, loop_positions[loop_depth], SEEK_SET);
                ESP_LOGI(TAG, "Loop %d iteration %d", loop_depth, loop_counts[loop_depth]);
            } else {
                // Exit the loop
                ESP_LOGI(TAG, "Exiting loop %d", loop_depth);
                loop_depth--;
            }
            continue;
        }
        
        // Check for hold command
        if (strstr(line, "hold ")) {
            int hold_time = atoi(line + 5); // Skip "hold "
            if (hold_time > 0) {
                ESP_LOGI(TAG, "Holding for %d ms", hold_time);
                vTaskDelay(hold_time / portTICK_PERIOD_MS);
            }
            continue;
        }
        
        // Execute regular command
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
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase()); // Erase the entire NVS partition
        ret = nvs_flash_init(); // Retry initialization
    }
    ESP_ERROR_CHECK(ret);
    //ESP_ERROR_CHECK(nvs_flash_init());
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
    //init_pwm_leds();
    init_uart();
    //test_leds();
    init_esp_now();
    init_known_peers();
    init_motor_pwm();
    set_led_color(0, 32, 0, 1000, 0); // Solid green for ready state
    // Start dynamic discovery by sending a pairing request
    uint8_t broadcast_mac[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    send_message(broadcast_mac, MSG_PAIRING_REQUEST, NULL, 0);

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