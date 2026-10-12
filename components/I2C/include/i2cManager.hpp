#ifndef I2C_MANAGER_HPP
#define I2C_MANAGER_HPP

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

class I2CManager {
    private:
        static i2c_master_bus_handle_t busHandle;

        I2CManager() = default;

    public:
        static esp_err_t initialize(gpio_num_t sdaPin, gpio_num_t sclPin);
        static esp_err_t scan(uint8_t* addresses, size_t capacity, size_t* foundCnt, int timeoutMs = 20);
        static esp_err_t probe(uint8_t address, int timeoutMs = 1000);
        static esp_err_t addDevice(uint8_t address, uint32_t frequencyHz, i2c_master_dev_handle_t* deviceHandle);
        static esp_err_t removeDevice(i2c_master_dev_handle_t deviceHandle);
        static esp_err_t write(i2c_master_dev_handle_t deviceHandle, const uint8_t* data, size_t length, int timeoutMs = 1000);
};

#endif // !I2C_MANAGER_HPP
