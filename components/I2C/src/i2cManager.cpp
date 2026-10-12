#include "i2cManager.hpp"

#include "esp_log.h"

namespace {
    constexpr char TAG[] = "I2C";
}

i2c_master_bus_handle_t I2CManager::busHandle = nullptr;

esp_err_t I2CManager::initialize(gpio_num_t sdaPin, gpio_num_t sclPin) {
    if(busHandle != nullptr)
        return ESP_OK;

    if(sdaPin == sclPin || sdaPin < 0 || sdaPin >= GPIO_NUM_MAX || sclPin < 0 || sclPin >= GPIO_NUM_MAX)
        return ESP_ERR_INVALID_ARG;

    i2c_master_bus_config_t busConfig{};
    busConfig.i2c_port = I2C_NUM_0;
    busConfig.sda_io_num = sdaPin;
    busConfig.scl_io_num = sclPin;
    busConfig.clk_source = I2C_CLK_SRC_DEFAULT;
    busConfig.glitch_ignore_cnt = 7;
    busConfig.intr_priority = 0;
    busConfig.trans_queue_depth = 0;
    busConfig.flags.enable_internal_pullup = true;

    const esp_err_t error = i2c_new_master_bus(&busConfig, &busHandle);
    if(error != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(error));
        busHandle = nullptr;
        return error;
    }

    ESP_LOGI(TAG, "I2C master bus initialized: SDA=%d SCL=%d", static_cast<int>(sdaPin), static_cast<int>(sclPin));
    return ESP_OK;
}

esp_err_t I2CManager::scan(uint8_t* addresses, size_t capacity, size_t* foundCnt, int timeoutMs) {
    if(busHandle == nullptr || foundCnt == nullptr || timeoutMs < 0 || (capacity > 0 && addresses == nullptr))
        return ESP_ERR_INVALID_ARG;

    *foundCnt = 0;
    size_t totalFound = 0;
    for(uint16_t address = 0x08; address <= 0x77; ++address) {
        const esp_err_t error = i2c_master_probe(busHandle, address, timeoutMs);
        if(error == ESP_OK) {
            if(totalFound < capacity)
                addresses[totalFound] = static_cast<uint8_t>(address);
            ++totalFound;
        } else if(error != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "I2C scan probe failed at 0x%02X: %s", static_cast<unsigned int>(address), esp_err_to_name(error));
        }
    }

    *foundCnt = totalFound < capacity ? totalFound : capacity;
    return (totalFound > capacity) ? ESP_ERR_NO_MEM : ESP_OK;
}

esp_err_t I2CManager::probe(uint8_t address, int timeoutMs) {
    if(busHandle == nullptr || address > 0x7F || timeoutMs < 0)
        return ESP_ERR_INVALID_ARG;

    const esp_err_t error = i2c_master_probe(busHandle, address, timeoutMs);

    if(error != ESP_OK)
        ESP_LOGW(TAG, "I2C device not found at 0x%02X: %s", address, esp_err_to_name(error));

    return error;
}

esp_err_t I2CManager::addDevice(uint8_t address, uint32_t frequencyHz, i2c_master_dev_handle_t* deviceHandle) {
    if(busHandle == nullptr || deviceHandle == nullptr || frequencyHz == 0 || address > 0x7F)
        return ESP_ERR_INVALID_ARG;

    i2c_device_config_t deviceConfig{};
    deviceConfig.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    deviceConfig.device_address = address;
    deviceConfig.scl_speed_hz = frequencyHz;
    deviceConfig.scl_wait_us = 0;
    deviceConfig.flags.disable_ack_check = false;

    const esp_err_t error = i2c_master_bus_add_device(busHandle, &deviceConfig, deviceHandle);
    if(error != ESP_OK)
        ESP_LOGE(TAG, "Failed to add I2C device 0x%02X: %s", address, esp_err_to_name(error));

    return error;
}

esp_err_t I2CManager::removeDevice(i2c_master_dev_handle_t deviceHandle) {
    if(deviceHandle == nullptr)
        return ESP_ERR_INVALID_ARG;

    const esp_err_t error = i2c_master_bus_rm_device(deviceHandle);
    if(error != ESP_OK)
        ESP_LOGE(TAG, "Failed to remove I2C device: %s", esp_err_to_name(error));

    return error;
}

esp_err_t I2CManager::write(i2c_master_dev_handle_t deviceHandle, const uint8_t* data, size_t length, int timeoutMs) {
    if(deviceHandle == nullptr || data == nullptr || length == 0 || timeoutMs < 0)
        return ESP_ERR_INVALID_ARG;

    const esp_err_t error = i2c_master_transmit(deviceHandle, data, length, timeoutMs);
    if(error != ESP_OK)
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(error));

    return error;
}
