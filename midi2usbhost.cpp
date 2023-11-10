/**
 * @file Pico-USB-Host-MIDI-Adapter.c
 * @brief A USB Host to Serial Port MIDI adapter that runs on a Raspberry Pi
 * Pico board
 *
 * MIT License

 * Copyright (c) 2022 rppicomidi

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 */
#include <cstdio>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "midi_uart_lib.h"
#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_midi_host.h"
#include "encoder.hpp"
#include "pico-ssd1306/textRenderer/TextRenderer.h"

// On-board LED mapping. If no LED, set to NO_LED_GPIO
constexpr uint NO_LED_GPIO = 255;
constexpr uint LED_GPIO = 25;

// UART selection Pin mapping. You can move these for your design if you want to
// Make sure all these values are consistent with your choice of midi_uart
// The default is to use UART 1, but you are free to use UART 0 if you make
// the changes in the CMakeLists.txt file or in your environment. Note
// that if you use UART0, then serial port debug will not be enabled
#ifndef MIDI_UART_NUM
#define MIDI_UART_NUM 1
#endif
#ifndef MIDI_UART_TX_GPIO
#define MIDI_UART_TX_GPIO 4
#endif
#ifndef MIDI_UART_RX_GPIO
#define MIDI_UART_RX_GPIO 5
#endif

static void *midi_uart_instance;
static uint8_t midi_dev_addr = 0;

using namespace encoder;

// Create an encoder on the 3 ADC pins, using PIO 0 and State Machine 0
constexpr uint PIN_A = 26;  // The A channel pin
constexpr uint PIN_B = 28;  // The B channel pin
constexpr uint PIN_C = 27;  // The common pin
Encoder enc(pio0, 0, {PIN_A, PIN_B}, PIN_C);

constexpr uint8_t GM_PROGRAM_NUMBER_SIZE = 128;
uint8_t sound = 0;

static void blink_led(void)
{
    static absolute_time_t previous_timestamp = {0};

    static bool led_state = false;

    // This design has no on-board LED
    if (NO_LED_GPIO == LED_GPIO)
        return;
    absolute_time_t now = get_absolute_time();

    int64_t diff = absolute_time_diff_us(previous_timestamp, now);
    if (diff > 1000000) {
        board_led_write(led_state);
        led_state = !led_state;
        previous_timestamp = now;
    }
}

static void init_display(void)
{
    constexpr uint I2C_PORT = 0;
    constexpr uint I2C_PIN_SDA = 4;
    constexpr uint I2C_PIN_SCL = 5;

    i2c_init(I2C_PORT, 1000000);
    gpio_set_function(I2C_PIN_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_PIN_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_PIN_SDA);
    gpio_pull_up(I2C_PIN_SCL);
}

static void sound_name(char *buf, const uint8_t sound)
{
    // TODO
    sprintf(buf, sizeof(buf), "Sound = %d", sound + 1);
}

static void poll_midi_uart_rx(bool connected)
{
    uint8_t rx[48];
    // Pull any bytes received on the MIDI UART out of the receive buffer and
    // send them out via USB MIDI on virtual cable 0
    uint8_t nread = midi_uart_poll_rx_buffer(midi_uart_instance, rx, sizeof(rx));
    if (nread > 0 && connected && tuh_midih_get_num_tx_cables(midi_dev_addr) >= 1)
    {
        uint32_t nwritten = tuh_midi_stream_write(midi_dev_addr, 0, rx, nread);
        if (nwritten != nread) {
            TU_LOG1("Warning: Dropped %lu bytes receiving from UART MIDI In\r\n", nread - nwritten);
        }
    }
}

static void write_midi_uart_program_change(const uint8_t sound)
{
    uint8_t tx[2];
    tx[0] = 0xc0; // Program Change
    tx[1] = sound;
    const uint8_t buflen = sizeof(tx);
    const uint8_t nwritten = midi_uart_write_tx_buffer(midi_uart_instance, tx, buflen);
    if (nwritten != buflen) {
        TU_LOG1("Warning: Dropped %lu bytes sending to UART MIDI Out\r\n", buflen - nwritten);
    }
}

int main() {
    bi_decl(bi_program_description("Provide a USB host interface for Serial Port MIDI."));
    bi_decl(bi_1pin_with_name(LED_GPIO, "On-board LED"));
    bi_decl(bi_2pins_with_names(MIDI_UART_TX_GPIO, "MIDI UART TX", MIDI_UART_RX_GPIO, "MIDI UART RX"));

    stdio_init_all();

    // Sleep 8 seconds to give enough time to connect up a terminal
    sleep_ms(8000);

    board_init();
    printf("Pico MIDI Host to MIDI UART Adapter\r\n");
    tusb_init();

    // Map the pins to functions
    midi_uart_instance = midi_uart_configure(MIDI_UART_NUM, MIDI_UART_TX_GPIO, MIDI_UART_RX_GPIO);
    printf("Configured MIDI UART %u for 31250 baud\r\n", MIDI_UART_NUM);

    // Uncomment the below line to reverse the counting direction
    // enc.direction(REVERSED_DIR);

    enc.init();
    printf("Sound = %d\n", sound + 1);

    // Create a new display object
    init_display();
    pico_ssd1306::SSD1306 display = pico_ssd1306::SSD1306(I2C_PORT, 0x3D, pico_ssd1306::Size::W128xH64);

    while (1) {
        tuh_task();
        blink_led();

        if (enc.delta() != 0)
        {
            sound = enc.count() % GM_PROGRAM_NUMBER_SIZE;
            write_midi_uart_program_change(sound);
            printf("Sound = %d\n", sound + 1);

            char *str[12]; // 128/12 = 10.6, so 12 chars is enough
            sound_name(str, sound);
            drawText(&display, font_12x16, str, 0, 0);
        }

        const bool connected = midi_dev_addr != 0 && tuh_midi_configured(midi_dev_addr);
        poll_midi_uart_rx(connected);
        if (connected)
            tuh_midi_stream_flush(midi_dev_addr);
        midi_uart_drain_tx_buffer(midi_uart_instance);
    }
}

//--------------------------------------------------------------------+
// TinyUSB Callbacks
//--------------------------------------------------------------------+

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx)
{
  printf("MIDI device address = %u, IN endpoint %u has %u cables, OUT endpoint %u has %u cables\r\n",
      dev_addr, in_ep & 0xf, num_cables_rx, out_ep & 0xf, num_cables_tx);

  if (midi_dev_addr == 0) {
    // then no MIDI device is currently connected
    midi_dev_addr = dev_addr;
  }
  else {
    printf("A different USB MIDI Device is already connected.\r\nOnly one device at a time is supported in this program\r\nDevice is disabled\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  if (dev_addr == midi_dev_addr) {
    midi_dev_addr = 0;
    printf("MIDI device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  }
  else {
    printf("Unused MIDI device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  }
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets)
{
    if (midi_dev_addr == dev_addr)
    {
        if (num_packets != 0)
        {
            uint8_t cable_num;
            uint8_t buffer[48];
            while (1) {
                uint32_t bytes_read = tuh_midi_stream_read(dev_addr, &cable_num, buffer, sizeof(buffer));
                if (bytes_read == 0)
                    return;
                if (cable_num == 0) {
                    uint8_t npushed = midi_uart_write_tx_buffer(midi_uart_instance,buffer,bytes_read);
                    if (npushed != bytes_read) {
                        TU_LOG1("Warning: Dropped %lu bytes sending to UART MIDI Out\r\n", bytes_read - npushed);
                    }
                }
            }
        }
    }
}

void tuh_midi_tx_cb(uint8_t dev_addr)
{
    (void)dev_addr;
}
