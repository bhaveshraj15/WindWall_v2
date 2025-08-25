#include <stdio.h>
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_mac.h"

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

void app_main(void){
    init_led_strip();

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);  // Get MAC for Wi-Fi station

    printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n",mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    while (1) {
        set_led_color(0, 32, 0, 500, 500); // green
        set_led_color(32, 0, 0, 500, 500); // red
        set_led_color(0, 0, 32, 500, 500); // blue
        set_led_color(32, 32, 32, 500, 500); // white
    }
}