#ifndef SSD1306_DISPLAY_HPP
#define SSD1306_DISPLAY_HPP

#include <cstddef>
#include <cstdint>
#include <vector>

#include "driver/i2c_master.h"
#include "esp_err.h"

class SSD1306Display {
    private:
        static constexpr uint8_t CONTROL_COMMAND = 0x00;
        static constexpr uint8_t CONTROL_DATA = 0x40;
        static constexpr size_t TRANSFER_CHUNK_SIZE = 16;

        uint8_t address;
        uint8_t displayHeight;
        uint32_t frequencyHz;
        i2c_master_dev_handle_t deviceHandle = nullptr;
        std::vector<uint8_t> framebuffer;
        bool initialized = false;

        esp_err_t sendCommand(uint8_t command);
        esp_err_t sendCommands(const uint8_t* commands, size_t length);
        esp_err_t sendInitCommands();

    public:
        enum class Geometry : uint8_t {
            Width128Height32,
            Width128Height64
        };

        SSD1306Display(uint8_t address, Geometry geometry = Geometry::Width128Height64, uint32_t frequencyHz = 400000);
        ~SSD1306Display();

        SSD1306Display(const SSD1306Display&) = delete;
        SSD1306Display& operator=(const SSD1306Display&) = delete;

        esp_err_t initialize();
        esp_err_t display();

        void clear();
        void setPixel(int16_t x, int16_t y, bool on = true);
        void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool on = true);
        void drawRect(int16_t x, int16_t y, int16_t width, int16_t height, bool on = true);
        void fillRect(int16_t x, int16_t y, int16_t width, int16_t height, bool on = true);
        void drawString(int16_t x, int16_t y, const char* text);

        inline uint16_t width() const { return 128; }
        inline uint16_t height() const { return displayHeight; }
};

#endif // !SSD1306_DISPLAY_HPP
