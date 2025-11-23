#include <can.h>

void setupCAN() {
    // Configure CAN controller
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install CAN driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("CAN driver installed");
    } else {void setupCAN();
        Serial.println("Failed to install CAN driver");
        return;
    }

    // Start CAN driver
    if (twai_start() == ESP_OK) {
        Serial.println("CAN driver started");
    } else {
        Serial.println("Failed to start CAN driver");
    }
}