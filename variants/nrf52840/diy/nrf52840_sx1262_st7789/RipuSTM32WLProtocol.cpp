#include "RipuSTM32WLProtocol.h"

namespace ripu_stm32wl
{

uint16_t crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8U;
        for (uint8_t bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000U) ? static_cast<uint16_t>((crc << 1U) ^ 0x1021U) : static_cast<uint16_t>(crc << 1U);
        }
    }
    return crc;
}

} // namespace ripu_stm32wl
