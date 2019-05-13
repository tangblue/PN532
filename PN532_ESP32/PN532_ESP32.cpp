
#include "PN532_ESP32.h"
#include "PN532_debug.h"

#define PIN_NUM_CS   GPIO_NUM_5
#define PIN_NUM_CLK  GPIO_NUM_18
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_MISO GPIO_NUM_19

#define STATUS_READ     2
#define DATA_WRITE      1
#define DATA_READ       3

static inline void delay(int ms)
{
    vTaskDelay((ms + portTICK_RATE_MS - 1) / portTICK_RATE_MS);
}

PN532_ESP32::PN532_ESP32()
{
    command = 0;
    _ss = PIN_NUM_CS;
}

void PN532_ESP32::begin()
{
    spi_bus_config_t buscfg {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 8,
        .flags = 0,
        .intr_flags = 0,
    };
    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits= 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = 2*1000*1000,
        .input_delay_ns = 0,
        .spics_io_num = -1,
        .flags = SPI_DEVICE_BIT_LSBFIRST,
        .queue_size = 7,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    esp_err_t ret = spi_bus_initialize(VSPI_HOST, &buscfg, 2);
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &_spi);

    gpio_set_direction(_ss, GPIO_MODE_OUTPUT);
    gpio_set_level(_ss, 1);
}

void PN532_ESP32::wakeup()
{
    gpio_set_level(_ss, 0);
    delay(2);
    gpio_set_level(_ss, 1);
}



int8_t PN532_ESP32::writeCommand(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    command = header[0];
    writeFrame(header, hlen, body, blen);

    uint8_t timeout = PN532_ACK_WAIT_TIME;
    while (!isReady()) {
        delay(1);
        timeout--;
        if (0 == timeout) {
            DMSG("Time out when waiting for ACK\n");
            return -2;
        }
    }
    if (readAckFrame()) {
        DMSG("Invalid ACK\n");
        return PN532_INVALID_ACK;
    }
    return 0;
}

int16_t PN532_ESP32::readResponse(uint8_t buf[], uint8_t len, uint16_t timeout)
{
    uint16_t time = 0;
    while (!isReady()) {
        delay(1);
        time++;
        if (timeout > 0 && time > timeout) {
            return PN532_TIMEOUT;
        }
    }

    gpio_set_level(_ss, 0);
    delay(1);

    int16_t result;
    do {
        write(DATA_READ);

        if (0x00 != read()      ||       // PREAMBLE
                0x00 != read()  ||       // STARTCODE1
                0xFF != read()           // STARTCODE2
           ) {

            result = PN532_INVALID_FRAME;
            break;
        }

        uint8_t length = read();
        if (0 != (uint8_t)(length + read())) {   // checksum of length
            result = PN532_INVALID_FRAME;
            break;
        }

        uint8_t cmd = command + 1;               // response command
        if (PN532_PN532TOHOST != read() || (cmd) != read()) {
            result = PN532_INVALID_FRAME;
            break;
        }

        DMSG("read:  ");
        DMSG_HEX(cmd);

        length -= 2;
        if (length > len) {
            for (uint8_t i = 0; i < length; i++) {
                DMSG_HEX(read());                 // dump message
            }
            DMSG("\nNot enough space\n");
            read();
            read();
            result = PN532_NO_SPACE;  // not enough space
            break;
        }

        uint8_t sum = PN532_PN532TOHOST + cmd;
        for (uint8_t i = 0; i < length; i++) {
            buf[i] = read();
            sum += buf[i];

            DMSG_HEX(buf[i]);
        }
        DMSG("\n");

        uint8_t checksum = read();
        if (0 != (uint8_t)(sum + checksum)) {
            DMSG("checksum is not ok\n");
            result = PN532_INVALID_FRAME;
            break;
        }
        read();         // POSTAMBLE

        result = length;
    } while (0);

    gpio_set_level(_ss, 1);

    return result;
}

bool PN532_ESP32::isReady()
{
    gpio_set_level(_ss, 0);

    write(STATUS_READ);
    uint8_t status = read() & 1;
    gpio_set_level(_ss, 1);
    return status;
}

void PN532_ESP32::writeFrame(const uint8_t *header, uint8_t hlen, const uint8_t *body, uint8_t blen)
{
    gpio_set_level(_ss, 0);
    delay(2);

    write(DATA_WRITE);
    write(PN532_PREAMBLE);
    write(PN532_STARTCODE1);
    write(PN532_STARTCODE2);

    uint8_t length = hlen + blen + 1;   // length of data field: TFI + DATA
    write(length);
    write(~length + 1);         // checksum of length

    write(PN532_HOSTTOPN532);
    uint8_t sum = PN532_HOSTTOPN532;    // sum of TFI + DATA

    DMSG("write:");

    for (uint8_t i = 0; i < hlen; i++) {
        write(header[i]);
        sum += header[i];

        DMSG_HEX(header[i]);
    }
    for (uint8_t i = 0; i < blen; i++) {
        write(body[i]);
        sum += body[i];

        DMSG_HEX(body[i]);
    }

    uint8_t checksum = ~sum + 1;        // checksum of TFI + DATA
    write(checksum);
    write(PN532_POSTAMBLE);

    gpio_set_level(_ss, 1);

    DMSG("\n");
}

int8_t PN532_ESP32::readAckFrame()
{
    const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};

    uint8_t ackBuf[sizeof(PN532_ACK)];

    gpio_set_level(_ss, 0);
    delay(1);
    write(DATA_READ);

    for (uint8_t i = 0; i < sizeof(PN532_ACK); i++) {
        ackBuf[i] = read();
    }

    gpio_set_level(_ss, 1);

    return memcmp(ackBuf, PN532_ACK, sizeof(PN532_ACK));
}

void PN532_ESP32::write(const uint8_t data)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_USE_TXDATA;
    t.length = 8;
    t.tx_data[0] = data;
    esp_err_t ret = spi_device_polling_transmit(_spi, &t);
    assert( ret == ESP_OK );
}

uint8_t PN532_ESP32::read()
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.length = 8;
    t.tx_data[0] = 0;

    esp_err_t ret = spi_device_polling_transmit(_spi, &t);
    assert( ret == ESP_OK );

    return t.rx_data[0];
}
