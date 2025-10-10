#include <stdio.h>
#include <string.h>
#include <math.h>
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
#include "esp_heap_caps.h"

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
    MSG_MOTOR_CONTROL,
    MSG_WAVEFORM_CONTROL
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

// Waveform types
typedef enum {
    WAVE_SINE,
    WAVE_TRIANGULAR,
    WAVE_SAWTOOTH
} waveform_type_t;

// Waveform control structure for ESP-NOW messages
typedef struct __attribute__((packed)) {
    waveform_type_t type;
    uint16_t low;
    uint16_t high;
    uint32_t period;
    uint32_t total_time;
} waveform_control_t;

// Structure for passing parameters to waveform tasks
typedef struct {
    uint16_t low;
    uint16_t high;
    uint32_t period_ms;
    uint32_t total_time_ms;
} waveform_args_t;

typedef struct {
    waveform_control_t params;
    bool active;
    TaskHandle_t task_handle;
} waveform_state_t;

static waveform_state_t current_waveform = {0};

// Function declarations
void execute_command(char *command, bool uart_feedback);
void process_command_file(const char *file_path);
void process_uart_command(char* command);
bool parse_mac_address(const char *mac_str, uint8_t *mac);
void generate_sine_wave(void *pvParameters);
void generate_triangular_wave(void *pvParameters);
void generate_sawtooth_wave(void *pvParameters);
void stop_waveform_generation(void);
static bool parse_wave_params(char* buf, uint16_t *low, uint16_t *high, uint32_t *period, uint32_t *total_time);
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
    "A0:85:E3:E8:00:EC", // Master
    "ff:ff:ff:ff:ff:ff", // for broadcasting
    "F0:9E:9E:1E:4A:84",
    "F0:9E:9E:1E:4A:E4",
    "F0:9E:9E:21:E2:70",
    "B4:3A:45:A3:54:04",
    "FC:01:2C:C5:4A:48", 
    "94:A9:90:2E:92:08",
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
        case MSG_WAVEFORM_CONTROL:
            ESP_LOGI(TAG, "Waveform control message received");
            waveform_control_t *wave_cmd = (waveform_control_t *)msg->data;
            
            // Allocate memory for waveform parameters
            waveform_args_t *wave_args = heap_caps_malloc(sizeof(waveform_args_t), MALLOC_CAP_8BIT);
            if (wave_args == NULL) {
                ESP_LOGE(TAG, "Failed to allocate memory for waveform parameters");
                break;
            }
            
            wave_args->low = wave_cmd->low;
            wave_args->high = wave_cmd->high;
            wave_args->period_ms = wave_cmd->period;
            wave_args->total_time_ms = wave_cmd->total_time;
            
            BaseType_t task_result;
            
            switch (wave_cmd->type) {
                case WAVE_SINE:
                    stop_waveform_generation();
                    task_result = xTaskCreate(generate_sine_wave, "remote_sine", 4096, wave_args, 5, &current_waveform.task_handle);
                    break;
                case WAVE_TRIANGULAR:
                    stop_waveform_generation();
                    task_result = xTaskCreate(generate_triangular_wave, "remote_tri", 4096, wave_args, 5, &current_waveform.task_handle);
                    break;
                case WAVE_SAWTOOTH:
                    stop_waveform_generation();
                    task_result = xTaskCreate(generate_sawtooth_wave, "remote_saw", 4096, wave_args, 5, &current_waveform.task_handle);
                    break;
                default:
                    heap_caps_free(wave_args);
                    task_result = pdFALSE;
                    break;
            }
            
            if (task_result == pdPASS) {
                current_waveform.active = true;
                current_waveform.params = *wave_cmd;
            } else {
                heap_caps_free(wave_args);
                ESP_LOGE(TAG, "Failed to create waveform task");
            }
            
            set_led_color(0, 32, 32, 300, 100);
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

static bool parse_wave_params(char* buf, uint16_t *low, uint16_t *high, uint32_t *period, uint32_t *total_time) {
    char* low_str = strtok(buf, " ");
    char* high_str = strtok(NULL, " ");
    char* period_str = strtok(NULL, " ");
    char* time_str = strtok(NULL, " ");

    if (!low_str || !high_str || !period_str || !time_str) {
        ESP_LOGW(TAG, "Missing parameters. Format: CMD low high period total_time [mac(s)]");
        return false;
    }

    *low = atoi(low_str);
    *high = atoi(high_str);
    *period = atoi(period_str);
    *total_time = atoi(time_str);

    // Validate ESC-safe parameters
    if (*low < 1000 || *high > 2000 || *low >= *high || *period < 40 || *total_time < 0) {
        ESP_LOGW(TAG, "Invalid params (low:%d high:%d period:%d time:%d)", *low, *high, *period, *total_time);
        ESP_LOGW(TAG, "Requires: 1000≤low<high≤2000, period≥40ms, time≥0ms");
        return false;
    }

    return true;
}

void generate_sine_wave(void *pvParameters) {
    // Extract parameters from the structure
    waveform_args_t *args = (waveform_args_t *)pvParameters;
    uint16_t low = args->low;
    uint16_t high = args->high;
    uint32_t period_ms = args->period_ms;
    uint32_t total_time_ms = args->total_time_ms;
    
    const uint16_t center = (low + high) / 2;
    const uint16_t amplitude = (high - low) / 2;
    const uint32_t step_duration = 20; // 50Hz update rate
    
    ESP_LOGI(TAG, "SINE: %d-%dµs, period:%dms, duration:%dms", low, high, period_ms, total_time_ms);

    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t end_time = (total_time_ms > 0) ? (start_time + total_time_ms) : UINT32_MAX;
    
    while (current_waveform.active) {
        // Check if we've reached the time limit (if total_time_ms > 0)
        if (total_time_ms > 0) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (current_time >= end_time) {
                break;
            }
        }
        
        uint32_t elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time;
        double phase = 2 * M_PI * fmod((double)elapsed, period_ms) / period_ms;
        uint16_t pulse = center + amplitude * sin(phase);
        pulse = (pulse < low) ? low : (pulse > high) ? high : pulse;
        
        // Apply to all motors
        for (int i = 0; i < NUM_MOTORS; i++) {
            set_motor_pulse(i, pulse);
        }
        
        vTaskDelay(pdMS_TO_TICKS(step_duration));
    }
    
    // Return to idle when stopped or time elapsed
    for (int i = 0; i < NUM_MOTORS; i++) {
        set_motor_pulse(i, 1000);
    }
    
    // Free the allocated memory
    heap_caps_free(args);
    current_waveform.active = false;
    current_waveform.task_handle = NULL;
    ESP_LOGI(TAG, "Sine wave completed");
    vTaskDelete(NULL);
}

void generate_triangular_wave(void *pvParameters) {
    // Extract parameters from the structure
    waveform_args_t *args = (waveform_args_t *)pvParameters;
    uint16_t low = args->low;
    uint16_t high = args->high;
    uint32_t period_ms = args->period_ms;
    uint32_t total_time_ms = args->total_time_ms;
    
    const uint32_t half_period = period_ms / 2;
    const uint32_t step_duration = 20;
    
    ESP_LOGI(TAG, "TRI: %d-%dµs, period:%dms, duration:%dms", low, high, period_ms, total_time_ms);

    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t end_time = (total_time_ms > 0) ? (start_time + total_time_ms) : UINT32_MAX;
    
    while (current_waveform.active) {
        // Check if we've reached the time limit (if total_time_ms > 0)
        if (total_time_ms > 0) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (current_time >= end_time) {
                break;
            }
        }
        uint32_t elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time;
        uint32_t mod_time = elapsed % period_ms;
        uint16_t pulse;
        
        if (mod_time < half_period) {
            pulse = low + (high - low) * mod_time / half_period;
        } else {
            uint32_t fall_time = mod_time - half_period;
            pulse = high - (high - low) * fall_time / half_period;
        }
        
        for (int i = 0; i < NUM_MOTORS; i++) {
            set_motor_pulse(i, pulse);
        }
        
        vTaskDelay(pdMS_TO_TICKS(step_duration));
    }
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        set_motor_pulse(i, 1000);
    }
    
    // Free the allocated memory
    heap_caps_free(args);
    current_waveform.active = false;
    current_waveform.task_handle = NULL;
    ESP_LOGI(TAG, "Triangular wave completed");
    vTaskDelete(NULL);
}

void generate_sawtooth_wave(void *pvParameters) {
    // Extract parameters from the structure
    waveform_args_t *args = (waveform_args_t *)pvParameters;
    uint16_t low = args->low;
    uint16_t high = args->high;
    uint32_t period_ms = args->period_ms;
    uint32_t total_time_ms = args->total_time_ms;
    
    const uint32_t step_duration = 20;
    
    ESP_LOGI(TAG, "SAW: %d-%dµs, period:%dms, duration:%dms", low, high, period_ms, total_time_ms);

    uint32_t start_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t end_time = (total_time_ms > 0) ? (start_time + total_time_ms) : UINT32_MAX;
    
    while (current_waveform.active) {
        // Check if we've reached the time limit (if total_time_ms > 0)
        if (total_time_ms > 0) {
            uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (current_time >= end_time) {
                break;
            }
        }
        uint32_t elapsed = (xTaskGetTickCount() * portTICK_PERIOD_MS) - start_time;
        uint32_t mod_time = elapsed % period_ms;
        uint16_t pulse = low + (high - low) * mod_time / period_ms;
        
        for (int i = 0; i < NUM_MOTORS; i++) {
            set_motor_pulse(i, pulse);
        }
        
        vTaskDelay(pdMS_TO_TICKS(step_duration));
    }
    
    for (int i = 0; i < NUM_MOTORS; i++) {
        set_motor_pulse(i, 1000);
    }
    
    // Free the allocated memory
    heap_caps_free(args);
    current_waveform.active = false;
    current_waveform.task_handle = NULL;
    ESP_LOGI(TAG, "Sawtooth wave completed");
    vTaskDelete(NULL);
}

void stop_waveform_generation(void) {
    if (current_waveform.active) {
        current_waveform.active = false;
        if (current_waveform.task_handle != NULL) {
            // Wait for task to clean up
            vTaskDelay(pdMS_TO_TICKS(100));
            // If task is still running, force delete it
            if (current_waveform.task_handle != NULL) {
                vTaskDelete(current_waveform.task_handle);
                current_waveform.task_handle = NULL;
            }
        }
        for (int i = 0; i < NUM_MOTORS; i++) {
            set_motor_pulse(i, 1000);
        }
        ESP_LOGI(TAG, "Waveform generation stopped");
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

    // Create a copy for parsing to avoid destroying original string
    char command_copy[100];
    strncpy(command_copy, command, sizeof(command_copy) - 1);
    command_copy[sizeof(command_copy) - 1] = '\0';
        
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
    else if (strstr(command, "sin ")) {
        uint16_t low, high;
        uint32_t period, total_time;
        
        // Use sscanf for non-destructive parsing of the first 4 parameters
        int parsed = sscanf(command + 4, "%hu %hu %lu %lu", &low, &high, &period, &total_time);
        
        if (parsed == 4) {
            // Validate parameters
            if (low < 1000 || high > 2000 || low >= high || period < 40) {
                if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Invalid waveform parameters\n", strlen("Invalid waveform parameters\n"));
                return;
            }
            
            stop_waveform_generation();
            
            // Look for MAC addresses after the 4th parameter
            const char *mac_start = command + 4;
            // Advance past the first 4 parameters
            for (int i = 0; i < 4; i++) {
                mac_start = strchr(mac_start, ' ');
                if (!mac_start) break;
                mac_start++; // Move past the space
            }
            
            bool has_mac_addresses = (mac_start != NULL && strlen(mac_start) > 0);
            
            if (has_mac_addresses) {
                // Send to remote devices
                waveform_control_t wave_cmd = {
                    .type = WAVE_SINE,
                    .low = low,
                    .high = high,
                    .period = period,
                    .total_time = total_time
                };
                
                // Parse MAC addresses using the original string
                char mac_copy[100];
                strncpy(mac_copy, mac_start, sizeof(mac_copy) - 1);
                mac_copy[sizeof(mac_copy) - 1] = '\0';
                
                char *token = strtok(mac_copy, " ");
                while (token != NULL) {
                    uint8_t mac[6];
                    if (parse_mac_address(token, mac)) {
                        send_message(mac, MSG_WAVEFORM_CONTROL, &wave_cmd, sizeof(wave_cmd));
                        if (uart_feedback) {
                            char response[80];
                            snprintf(response, sizeof(response), 
                                    "Sine wave command sent to: %s\n", token);
                            uart_write_bytes(UART_PORT_NUM, response, strlen(response));
                        }
                        // Add debug output to verify sending
                        ESP_LOGI(TAG, "Sending sine wave to MAC: %s", token);
                    } else {
                        if (uart_feedback) {
                            char response[80];
                            snprintf(response, sizeof(response), 
                                    "Invalid MAC format: %s\n", token);
                            uart_write_bytes(UART_PORT_NUM, response, strlen(response));
                        }
                        ESP_LOGE(TAG, "Failed to parse MAC: %s", token);
                    }
                    token = strtok(NULL, " ");
                }
            } else {
                // Execute locally (your existing local execution code)
                // [Keep your existing local execution code here]
            }
        } else {
            if (uart_feedback) uart_write_bytes(UART_PORT_NUM, 
                "Invalid sine command format. Use: sin <low> <high> <period> <total_time> [mac(s)]\n",
                strlen("Invalid sine command format. Use: sin <low> <high> <period> <total_time> [mac(s)]\n"));
        }
    }
    else if (strstr(command, "tri ")) {
        uint16_t low, high;
        uint32_t period, total_time;
        
        // Use sscanf for non-destructive parsing of the first 4 parameters
        int parsed = sscanf(command + 4, "%hu %hu %lu %lu", &low, &high, &period, &total_time);
        
        if (parsed == 4) {
            // Validate parameters
            if (low < 1000 || high > 2000 || low >= high || period < 40) {
                if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Invalid waveform parameters\n", strlen("Invalid waveform parameters\n"));
                return;
            }
            
            stop_waveform_generation();
            
            // Look for MAC addresses after the 4th parameter
            const char *mac_start = command + 4;
            // Advance past the first 4 parameters
            for (int i = 0; i < 4; i++) {
                mac_start = strchr(mac_start, ' ');
                if (!mac_start) break;
                mac_start++; // Move past the space
            }
            
            bool has_mac_addresses = (mac_start != NULL && strlen(mac_start) > 0);
            
            if (has_mac_addresses) {
                // Send to remote devices
                waveform_control_t wave_cmd = {
                    .type = WAVE_TRIANGULAR,
                    .low = low,
                    .high = high,
                    .period = period,
                    .total_time = total_time
                };
                
                // Parse MAC addresses using a copy of the string
                char mac_copy[100];
                strncpy(mac_copy, mac_start, sizeof(mac_copy) - 1);
                mac_copy[sizeof(mac_copy) - 1] = '\0';
                
                char *token = strtok(mac_copy, " ");
                while (token != NULL) {
                    uint8_t mac[6];
                    if (parse_mac_address(token, mac)) {
                        // Check if peer exists before sending
                        bool peer_exists = false;
                        for (int i = 0; i < peer_count; i++) {
                            if (memcmp(peers[i].mac, mac, 6) == 0) {
                                peer_exists = true;
                                break;
                            }
                        }
                        
                        if (!peer_exists) {
                            ESP_LOGW(TAG, "MAC %s not in peer list, attempting to add", token);
                            add_peer(mac, true);
                            vTaskDelay(100 / portTICK_PERIOD_MS); // Brief delay for peer registration
                        }
                        
                        send_message(mac, MSG_WAVEFORM_CONTROL, &wave_cmd, sizeof(wave_cmd));
                        if (uart_feedback) {
                            char response[80];
                            snprintf(response, sizeof(response), 
                                    "Triangular wave command sent to: %s\n", token);
                            uart_write_bytes(UART_PORT_NUM, response, strlen(response));
                        }
                        ESP_LOGI(TAG, "Sending triangular wave to MAC: %s", token);
                    } else {
                        if (uart_feedback) {
                            char response[80];
                            snprintf(response, sizeof(response), 
                                    "Invalid MAC format: %s\n", token);
                            uart_write_bytes(UART_PORT_NUM, response, strlen(response));
                        }
                        ESP_LOGE(TAG, "Failed to parse MAC: %s", token);
                    }
                    token = strtok(NULL, " ");
                }
            } else {
                // Execute locally
                waveform_args_t *tri_args = heap_caps_malloc(sizeof(waveform_args_t), MALLOC_CAP_8BIT);
                if (tri_args != NULL) {
                    tri_args->low = low;
                    tri_args->high = high;
                    tri_args->period_ms = period;
                    tri_args->total_time_ms = total_time;
                    
                    if (xTaskCreate(generate_triangular_wave, "tri_wave", 4096, tri_args, 5, &current_waveform.task_handle) == pdPASS) {
                        current_waveform.active = true;
                        current_waveform.params.low = low;
                        current_waveform.params.high = high;
                        current_waveform.params.period = period;
                        current_waveform.params.total_time = total_time;
                        
                        if (uart_feedback) {
                            char response[120];
                            snprintf(response, sizeof(response), 
                                    "Triangular wave started: %d-%dµs, period:%ldms, duration:%ldms\n", 
                                    low, high, period, total_time);
                            uart_write_bytes(UART_PORT_NUM, response, strlen(response));
                        }
                    } else {
                        heap_caps_free(tri_args);
                        if (uart_feedback) {
                            uart_write_bytes(UART_PORT_NUM, "Failed to create triangular wave task\n", 
                                            strlen("Failed to create triangular wave task\n"));
                        }
                    }
                } else {
                    if (uart_feedback) {
                        uart_write_bytes(UART_PORT_NUM, "Failed to allocate memory for triangular wave\n", 
                                        strlen("Failed to allocate memory for triangular wave\n"));
                    }
                }
            }
        } else {
            if (uart_feedback) uart_write_bytes(UART_PORT_NUM, 
                "Invalid triangular command format. Use: tri <low> <high> <period> <total_time> [mac(s)]\n",
                strlen("Invalid triangular command format. Use: tri <low> <high> <period> <total_time> [mac(s)]\n"));
        }
    }
    else if (strstr(command, "saw ")) {
        uint16_t low, high;
        uint32_t period, total_time;
        
        // Use sscanf for non-destructive parsing of the first 4 parameters
        int parsed = sscanf(command + 4, "%hu %hu %lu %lu", &low, &high, &period, &total_time);
        
        if (parsed == 4) {
            // Validate parameters
            if (low < 1000 || high > 2000 || low >= high || period < 40) {
                if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Invalid waveform parameters\n", strlen("Invalid waveform parameters\n"));
                return;
            }
            
            stop_waveform_generation();
            
            // Look for MAC addresses after the 4th parameter
            const char *mac_start = command + 4;
            // Advance past the first 4 parameters
            for (int i = 0; i < 4; i++) {
                mac_start = strchr(mac_start, ' ');
                if (!mac_start) break;
                mac_start++; // Move past the space
            }
            
            bool has_mac_addresses = (mac_start != NULL && strlen(mac_start) > 0);
            
            if (has_mac_addresses) {
                // Send to remote devices
                waveform_control_t wave_cmd = {
                    .type = WAVE_SAWTOOTH,
                    .low = low,
                    .high = high,
                    .period = period,
                    .total_time = total_time
                };
                
                // Parse MAC addresses using a copy of the string
                char mac_copy[100];
                strncpy(mac_copy, mac_start, sizeof(mac_copy) - 1);
                mac_copy[sizeof(mac_copy) - 1] = '\0';
                
                char *token = strtok(mac_copy, " ");
                while (token != NULL) {
                    uint8_t mac[6];
                    if (parse_mac_address(token, mac)) {
                        // Check if peer exists before sending
                        bool peer_exists = false;
                        for (int i = 0; i < peer_count; i++) {
                            if (memcmp(peers[i].mac, mac, 6) == 0) {
                                peer_exists = true;
                                break;
                            }
                        }
                        
                        if (!peer_exists) {
                            ESP_LOGW(TAG, "MAC %s not in peer list, attempting to add", token);
                            add_peer(mac, true);
                            vTaskDelay(100 / portTICK_PERIOD_MS); // Brief delay for peer registration
                        }
                        
                        send_message(mac, MSG_WAVEFORM_CONTROL, &wave_cmd, sizeof(wave_cmd));
                        if (uart_feedback) {
                            char response[80];
                            snprintf(response, sizeof(response), 
                                    "Sawtooth wave command sent to: %s\n", token);
                            uart_write_bytes(UART_PORT_NUM, response, strlen(response));
                        }
                        ESP_LOGI(TAG, "Sending sawtooth wave to MAC: %s", token);
                    } else {
                        if (uart_feedback) {
                            char response[80];
                            snprintf(response, sizeof(response), 
                                    "Invalid MAC format: %s\n", token);
                            uart_write_bytes(UART_PORT_NUM, response, strlen(response));
                        }
                        ESP_LOGE(TAG, "Failed to parse MAC: %s", token);
                    }
                    token = strtok(NULL, " ");
                }
            } else {
                // Execute locally
                waveform_args_t *saw_args = heap_caps_malloc(sizeof(waveform_args_t), MALLOC_CAP_8BIT);
                if (saw_args != NULL) {
                    saw_args->low = low;
                    saw_args->high = high;
                    saw_args->period_ms = period;
                    saw_args->total_time_ms = total_time;
                    
                    if (xTaskCreate(generate_sawtooth_wave, "saw_wave", 4096, saw_args, 5, &current_waveform.task_handle) == pdPASS) {
                        current_waveform.active = true;
                        current_waveform.params.low = low;
                        current_waveform.params.high = high;
                        current_waveform.params.period = period;
                        current_waveform.params.total_time = total_time;
                        
                        if (uart_feedback) {
                            char response[120];
                            snprintf(response, sizeof(response), 
                                    "Sawtooth wave started: %d-%dµs, period:%ldms, duration:%ldms\n", 
                                    low, high, period, total_time);
                            uart_write_bytes(UART_PORT_NUM, response, strlen(response));
                        }
                    } else {
                        heap_caps_free(saw_args);
                        if (uart_feedback) {
                            uart_write_bytes(UART_PORT_NUM, "Failed to create sawtooth wave task\n", 
                                            strlen("Failed to create sawtooth wave task\n"));
                        }
                    }
                } else {
                    if (uart_feedback) {
                        uart_write_bytes(UART_PORT_NUM, "Failed to allocate memory for sawtooth wave\n", 
                                        strlen("Failed to allocate memory for sawtooth wave\n"));
                    }
                }
            }
        } else {
            if (uart_feedback) uart_write_bytes(UART_PORT_NUM, 
                "Invalid sawtooth command format. Use: saw <low> <high> <period> <total_time> [mac(s)]\n",
                strlen("Invalid sawtooth command format. Use: saw <low> <high> <period> <total_time> [mac(s)]\n"));
        }
    }
    else if (strcmp(command, "wave stop") == 0) {
        stop_waveform_generation();
        if (uart_feedback) uart_write_bytes(UART_PORT_NUM, "Waveform stopped\n", strlen("Waveform stopped\n"));
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
            "  sin <low> <high> <period> <total_time> [mac(s)] - Sine wave\n"
            "  tri <low> <high> <period> <total_time> [mac(s)] - Triangular wave\n"
            "  saw <low> <high> <period> <total_time> [mac(s)] - Sawtooth wave\n"
            "  wave stop - Stop waveform generation\n"
            "  \n"
            "  Parameters:\n"
            "    low/high: 1000-2000µs, low < high\n"
            "    period: ≥40ms\n"
            "    total_time: duration in ms (0 = infinite)\n"
            "    mac: optional target devices\n"
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