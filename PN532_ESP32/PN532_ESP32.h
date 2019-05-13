
#ifndef __PN532_ESP32_H__
#define __PN532_ESP32_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "PN532Interface.h"

class PN532_ESP32 : public PN532Interface {
public:
    PN532_ESP32();

    void begin();
    void wakeup();
    int8_t writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);

    int16_t readResponse(uint8_t buf[], uint8_t len, uint16_t timeout);

private:
    spi_device_handle_t _spi;
    gpio_num_t _ss;
    uint8_t command;

    bool isReady();
    void writeFrame(const uint8_t *header, uint8_t hlen, const uint8_t *body = 0, uint8_t blen = 0);
    int8_t readAckFrame();

    void write(const uint8_t data);
    uint8_t read();
};

#endif
