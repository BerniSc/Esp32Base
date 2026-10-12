#include "ssd1306Display.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>

#include "esp_log.h"
#include "i2cManager.hpp"
#include "oledFonts.hpp"

namespace {
    constexpr uint8_t DISPLAY_OFF = 0xAE;
    constexpr uint8_t DISPLAY_ON = 0xAF;
    constexpr uint8_t SET_DISPLAY_CLOCK_DIV = 0xD5;
    constexpr uint8_t SET_MULTIPLEX = 0xA8;
    constexpr uint8_t SET_DISPLAY_OFFSET = 0xD3;
    constexpr uint8_t SET_START_LINE = 0x40;
    constexpr uint8_t CHARGE_PUMP = 0x8D;
    constexpr uint8_t MEMORY_MODE = 0x20;
    constexpr uint8_t SEGMENT_REMAP = 0xA1;
    constexpr uint8_t COM_SCAN_DEC = 0xC8;
    constexpr uint8_t SET_COM_PINS = 0xDA;
    constexpr uint8_t SET_CONTRAST = 0x81;
    constexpr uint8_t SET_PRECHARGE = 0xD9;
    constexpr uint8_t SET_VCOM_DETECT = 0xDB;
    constexpr uint8_t DISPLAY_ALL_ON_RESUME = 0xA4;
    constexpr uint8_t NORMAL_DISPLAY = 0xA6;
    constexpr uint8_t COLUMN_ADDRESS = 0x21;
    constexpr uint8_t PAGE_ADDRESS = 0x22;

    // Font header stores width at byte 0, height at 1, first character at 2, and character count at 3. Jump-table entries begin after this 4-byte header.
    constexpr size_t FONT_JUMP_TABLE_START = 4;
    // Byte offsets of height, first supported character, and character count within the font's 4-byte header.
    constexpr size_t FONT_HEIGHT = 1;
    constexpr size_t FONT_FIRST_CHAR = 2;
    constexpr size_t FONT_CHAR_COUNT = 3;
    // Each jump-table entry stores a 16-bit bitmap-data offset (high byte, then low byte), followed by glyph bitmap size and glyph width.
    constexpr size_t FONT_JUMP_TABLE_BYTES = 4;
    constexpr size_t FONT_JUMP_LSB = 1;
    constexpr size_t FONT_JUMP_SIZE = 2;
    constexpr size_t FONT_JUMP_WIDTH = 3;

    uint8_t fontByte(const uint8_t* font, size_t index) {
        return font[index];
    }
}

SSD1306Display::SSD1306Display(uint8_t address, Geometry geometry, uint32_t frequencyHz) : address(address),
      displayHeight(geometry == Geometry::Width128Height32 ? 32 : 64), frequencyHz(frequencyHz),
      framebuffer(static_cast<size_t>(128) * displayHeight / 8, 0) {}

SSD1306Display::~SSD1306Display() {
    if(deviceHandle != nullptr)
        I2CManager::removeDevice(deviceHandle);
}

esp_err_t SSD1306Display::initialize() {
    if(initialized)
        return ESP_OK;

    const esp_err_t probeError = I2CManager::probe(address);
    if(probeError != ESP_OK)
        return probeError;

    esp_err_t error = I2CManager::addDevice(address, frequencyHz, &deviceHandle);
    if(error != ESP_OK)
        return error;

    error = sendInitCommands();
    if(error != ESP_OK) {
        I2CManager::removeDevice(deviceHandle);
        deviceHandle = nullptr;
        return error;
    }

    initialized = true;
    clear();
    return display();
}

esp_err_t SSD1306Display::display() {
    if(!initialized)
        return ESP_ERR_INVALID_STATE;

    // Select full controller column range (0–127) for framebuffer write.
    const uint8_t columnCommands[] = {COLUMN_ADDRESS, 0, 127};
    esp_err_t error = sendCommands(columnCommands, sizeof(columnCommands));
    if(error != ESP_OK)
        return error;

    // Select all 8-pixel-high display pages for this panel's height.
    const uint8_t pageCommands[] = {PAGE_ADDRESS, 0, static_cast<uint8_t>((displayHeight / 8) - 1)};
    error = sendCommands(pageCommands, sizeof(pageCommands));
    if(error != ESP_OK)
        return error;

    // Prefix each data transfer with the SSD1306 data-control byte.
    uint8_t transfer[TRANSFER_CHUNK_SIZE + 1];
    transfer[0] = CONTROL_DATA;
    for(size_t offset = 0; offset < framebuffer.size(); offset += TRANSFER_CHUNK_SIZE) {
        const size_t chunkLength = std::min(TRANSFER_CHUNK_SIZE, framebuffer.size() - offset);
        // Copy next framebuffer slice after control byte and transmit it.
        std::memcpy(transfer + 1, framebuffer.data() + offset, chunkLength);
        error = I2CManager::write(deviceHandle, transfer, chunkLength + 1);
        if(error != ESP_OK)
            return error;
    }
    return ESP_OK;
}

void SSD1306Display::clear() {
    std::fill(framebuffer.begin(), framebuffer.end(), 0);
}

void SSD1306Display::setPixel(int16_t x, int16_t y, bool on) {
    if(x < 0 || x >= 128 || y < 0 || y >= displayHeight)
        return;

    const size_t index = static_cast<size_t>(x) + static_cast<size_t>(y / 8) * 128;
    const uint8_t mask = static_cast<uint8_t>(1U << (y & 7));
    if(on)
        framebuffer[index] |= mask;
    else
        framebuffer[index] &= static_cast<uint8_t>(~mask);
}

void SSD1306Display::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, bool on) {
    // Bresenham's algorithm tracks line error using integer steps.
    const int16_t dx = static_cast<int16_t>(std::abs(x1 - x0));
    const int16_t sx = x0 < x1 ? 1 : -1;
    const int16_t dy = static_cast<int16_t>(-std::abs(y1 - y0));
    const int16_t sy = y0 < y1 ? 1 : -1;
    int16_t error = static_cast<int16_t>(dx + dy);

    while(true) {
        setPixel(x0, y0, on);
        if(x0 == x1 && y0 == y1)
            break;
        // Use one error snapshot for both axis decisions before updating it.
        const int16_t twiceError = static_cast<int16_t>(2 * error);
        // Step horizontally when accumulated error crosses the vertical delta.
        if(twiceError >= dy) {
            error = static_cast<int16_t>(error + dy);
            x0 = static_cast<int16_t>(x0 + sx);
        }
        // Step vertically when accumulated error crosses the horizontal delta.
        if(twiceError <= dx) {
            error = static_cast<int16_t>(error + dx);
            y0 = static_cast<int16_t>(y0 + sy);
        }
    }
}

void SSD1306Display::drawRect(int16_t x, int16_t y, int16_t width, int16_t height, bool on) {
    if(width <= 0 || height <= 0)
        return;
    // 00 -> 10; 00 -> 01; 10 -> 11; 01 -> 11
    drawLine(x, y, static_cast<int16_t>(x + width - 1), y, on);
    drawLine(x, y, x, static_cast<int16_t>(y + height - 1), on);
    drawLine(static_cast<int16_t>(x + width - 1), y, static_cast<int16_t>(x + width - 1), static_cast<int16_t>(y + height - 1), on);
    drawLine(x, static_cast<int16_t>(y + height - 1), static_cast<int16_t>(x + width - 1), static_cast<int16_t>(y + height - 1), on);
}

void SSD1306Display::fillRect(int16_t x, int16_t y, int16_t width, int16_t height, bool on) {
    if(width <= 0 || height <= 0)
        return;
    for(int16_t column = x; column < x + width; ++column)
        drawLine(column, y, column, static_cast<int16_t>(y + height - 1), on);
}

void SSD1306Display::drawString(int16_t x, int16_t y, const char* text) {
    if(text == nullptr)
        return;

    // Font header provides glyph height and supported character range.
    const uint8_t* font = fonts::ArialMT_Plain_10;
    const uint8_t fontHeight = fontByte(font, FONT_HEIGHT);
    int16_t cursorX = x;
    int16_t cursorY = y;
    const uint8_t firstCharacter = fontByte(font, FONT_FIRST_CHAR);
    const uint8_t characterCount = fontByte(font, FONT_CHAR_COUNT);
    const size_t jumpTableSize = static_cast<size_t>(characterCount) * FONT_JUMP_TABLE_BYTES;

    // Walk as unsigned bytes so characters above ASCII 127 keep their font codes.
    for(const unsigned char* character = reinterpret_cast<const unsigned char*>(text); *character != '\0'; ++character) {
        // Newline returns to the initial x position and starts the next text row.
        if(*character == '\n') {
            cursorX = x;
            cursorY = static_cast<int16_t>(cursorY + fontHeight);
            continue;
        }
        // Skip codes outside this font's character range.
        if(*character < firstCharacter || *character >= firstCharacter + characterCount)
            continue;

        // Character code selects one fixed-size jump-table entry.
        const uint8_t characterIndex = static_cast<uint8_t>(*character - firstCharacter);
        const size_t jumpTableIndex = FONT_JUMP_TABLE_START + static_cast<size_t>(characterIndex) * FONT_JUMP_TABLE_BYTES;

        // Entry stores bitmap offset, bitmap byte count, and advance width.
        const uint8_t highOffset = fontByte(font, jumpTableIndex);
        const uint8_t lowOffset = fontByte(font, jumpTableIndex + FONT_JUMP_LSB);

        const uint8_t characterBytes = fontByte(font, jumpTableIndex + FONT_JUMP_SIZE);
        const uint8_t characterWidth = fontByte(font, jumpTableIndex + FONT_JUMP_WIDTH);

        // 0xFFFF offset marks an unsupported or empty glyph.
        if(!(highOffset == 0xFF && lowOffset == 0xFF)) {
            // Bitmap offsets are relative to bitmap data after header and jump table.
            const size_t dataOffset = FONT_JUMP_TABLE_START + jumpTableSize + (static_cast<size_t>(highOffset) << 8) + lowOffset;
            // Glyph bytes are stored column-first, with one byte per vertical 8-pixel band. This holds how many bands are in one Col.
            const uint8_t glyphBandCount = static_cast<uint8_t>(1 + ((fontHeight - 1) >> 3));
            for(uint16_t byteIndex = 0; byteIndex < characterBytes; ++byteIndex) {
                // Decode byte position into glyph column and vertical 8-pixel band.
                const uint8_t rasterByte = fontByte(font, dataOffset + byteIndex);
                const uint8_t column = static_cast<uint8_t>(byteIndex / glyphBandCount);
                const uint8_t glyphBand = static_cast<uint8_t>(byteIndex % glyphBandCount);
                for(uint8_t bit = 0; bit < 8; ++bit) {
                    // Set corresponding display pixel for each set bit in the glyph byte.
                    if((rasterByte & (1U << bit)) != 0)
                        setPixel(static_cast<int16_t>(cursorX + column), static_cast<int16_t>(cursorY + glyphBand * 8 + bit));
                }
            }
        }
        // Advance by glyph width, including spacing encoded by the font.
        cursorX = static_cast<int16_t>(cursorX + characterWidth);
    }
}

esp_err_t SSD1306Display::sendCommand(uint8_t command) {
    return sendCommands(&command, 1);
}

esp_err_t SSD1306Display::sendCommands(const uint8_t* commands, size_t length) {
    if(deviceHandle == nullptr || commands == nullptr || length == 0)
        return ESP_ERR_INVALID_STATE;

    uint8_t transfer[TRANSFER_CHUNK_SIZE + 1];
    transfer[0] = CONTROL_COMMAND;
    for(size_t offset = 0; offset < length; offset += TRANSFER_CHUNK_SIZE) {
        const size_t chunkLength = std::min(TRANSFER_CHUNK_SIZE, length - offset);
        std::memcpy(transfer + 1, commands + offset, chunkLength);
        const esp_err_t error = I2CManager::write(deviceHandle, transfer, chunkLength + 1);
        if(error != ESP_OK)
            return error;
    }
    return ESP_OK;
}

esp_err_t SSD1306Display::sendInitCommands() {
    const uint8_t commands[] = {
        DISPLAY_OFF,
        SET_DISPLAY_CLOCK_DIV, 0xF0,
        SET_MULTIPLEX, static_cast<uint8_t>(displayHeight - 1),
        SET_DISPLAY_OFFSET, 0x00,
        SET_START_LINE,
        CHARGE_PUMP, 0x14,
        MEMORY_MODE, 0x00,
        SEGMENT_REMAP,
        COM_SCAN_DEC,
        SET_COM_PINS, static_cast<uint8_t>(displayHeight == 64 ? 0x12 : 0x02),
        SET_CONTRAST, static_cast<uint8_t>(displayHeight == 64 ? 0xCF : 0x8F),
        SET_PRECHARGE, 0xF1,
        SET_VCOM_DETECT, 0x40,
        DISPLAY_ALL_ON_RESUME,
        NORMAL_DISPLAY,
        0x2E,
        DISPLAY_ON
    };
    return sendCommands(commands, sizeof(commands));
}
