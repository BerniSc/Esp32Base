#include "gpioManager.hpp"
#include "gpio.hpp"
#include "driver/gpio.h"
#include "soc/gpio_num.h"

#include "esp_log.h"

#define TAG "GPIO"

bool GPIOManager::isInitialized = false;
QueueHandle_t GPIOManager::interruptQueue = nullptr;
std::map<gpio_num_t, std::function<void()>> GPIOManager::callbackRegistry;

void GPIOManager::initialize() {
    if(!isInitialized) {
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        interruptQueue = xQueueCreate(10, sizeof(int)); // Queue for GPIO events
        // Task function | Task Name | Stacksize | Parameters | Priority (Still manually changable for own Interrupts) | Task Handle
        xTaskCreate(interruptTask, "GPIO Interrupt Task", 2048, nullptr, 10, nullptr);
        isInitialized = true;
        ESP_LOGI(TAG, "GPIO Manager initialized");
    }
}

std::shared_ptr<GPIO> GPIOManager::createGPIO(int pin, GPIOMode mode) {
    // Ensure manager is initialized
    // Check here to avoid Functioncall Overhead
    if(!isInitialized)
        initialize();
    ESP_LOGI(TAG, "Creating GPIO %d with mode %d", pin, static_cast<int>(mode));
    // No make shared. Need to be friend class ^^
    return std::shared_ptr<GPIO>(new GPIO(static_cast<gpio_num_t>(pin), mode));
}

void GPIOManager::registerInterrupt(gpio_num_t pin, gpio_int_type_t type, std::function<void()> callback) {
    // Put into registry, then set interrupttype and register it at handler with its dictkey as param
    callbackRegistry[pin] = callback;
    gpio_set_intr_type(pin, type);
    gpio_isr_handler_add(pin, isrHandler, (void*) pin);
    ESP_LOGI(TAG, "Attached interrupt to GPIO %d with type %d", pin, type);
}

void GPIOManager::deregisterInterrupt(gpio_num_t pin) {
    gpio_isr_handler_remove(pin);
    callbackRegistry.erase(pin);
    ESP_LOGI(TAG, "Removed interrupt from GPIO %d", pin);
}

// ISR Handler. Pass to seperate Queue in Task to prevent blocking ISR  
// Do this via activation Queue from ISR
void IRAM_ATTR GPIOManager::isrHandler(void* arg) {
    static uint32_t lastInterruptTime = 0;                  // Store last interrupttime
    uint32_t currentTime = xTaskGetTickCountFromISR();      // current time in ticks

    // Debounce threshold in ticks
    const uint32_t debounceThreshold = pdMS_TO_TICKS(50);

    // Handle overflow naturally with unsigned subtraction; Ignore interrupt if within debounce threshold
    if((currentTime - lastInterruptTime) < debounceThreshold)
        return;

    lastInterruptTime = currentTime; // Update last interrupt time and continue with Interrupt

    uint32_t pin = (uint32_t) arg;
    xQueueSendFromISR(interruptQueue, &pin, nullptr);
}

void GPIOManager::interruptTask(void* arg) {
    gpio_num_t pin;
    while(true)
        if(xQueueReceive(interruptQueue, &pin, portMAX_DELAY))
            if(callbackRegistry.find(pin) != callbackRegistry.end())
                callbackRegistry[pin]();
}

void GPIOManager::dumpPin(int pin) {
    gpio_dump_io_configuration(stdout, (1ULL << pin));
}

void GPIOManager::dumpPins() {
    gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);
}

