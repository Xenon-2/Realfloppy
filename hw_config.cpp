#include "hw_config.h"
#include "bridge.h"
#include "hardware/gpio.h"

static spi_t spis[] = {
    {
        .hw_inst = spi0,
        .miso_gpio = SD_PIN_MISO, // GP16
        .mosi_gpio = SD_PIN_MOSI, // GP19
        .sck_gpio = SD_PIN_SCK,   // GP18
        .baud_rate = 1000 * 1000, // 2 MHz
        .set_drive_strength = true,
        .mosi_gpio_drive_strength = GPIO_DRIVE_STRENGTH_4MA,
        .sck_gpio_drive_strength = GPIO_DRIVE_STRENGTH_4MA
    }
};

static sd_spi_if_t spi_ifs[] = {
    {
        .spi = &spis[0],
        .ss_gpio = SD_PIN_CS, // GP17
        .set_drive_strength = true,
        .ss_gpio_drive_strength = GPIO_DRIVE_STRENGTH_4MA
    }
};

static sd_card_t sd_cards[] = {
    {
        .type = SD_IF_SPI,
        .spi_if_p = &spi_ifs[0]
    }
};

extern "C" size_t sd_get_num() {
    return count_of(sd_cards);
}

extern "C" sd_card_t *sd_get_by_num(size_t num) {
    if (num < sd_get_num()) {
        return &sd_cards[num];
    }
    return NULL;
}
