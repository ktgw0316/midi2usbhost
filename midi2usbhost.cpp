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
#include "textRenderer/TextRenderer.h"

// On-board LED mapping. If no LED, set to NO_LED_GPIO
constexpr uint NO_LED_GPIO = 255;
constexpr uint LED_GPIO = 25;

// UART selection Pin mapping. You can move these for your design if you want to
// Make sure all these values are consistent with your choice of midi_uart
// The default is to use UART 1, but you are free to use UART 0 if you make
// the changes in the CMakeLists.txt file or in your environment. Note
// that if you use UART0, then serial port debug will not be enabled
#ifndef MIDI_UART_NUM
#define MIDI_UART_NUM 0
#endif
#ifndef MIDI_UART_TX_GPIO
#define MIDI_UART_TX_GPIO 16
#endif
#ifndef MIDI_UART_RX_GPIO
#define MIDI_UART_RX_GPIO 17
#endif

static void *midi_uart_instance;
static uint8_t midi_dev_addr = 0;

using namespace encoder;

// Create an encoder on the 3 ADC pins, using PIO 0 and State Machine 0
constexpr uint PIN_A_GPIO = 28;  // The A channel pin
constexpr uint PIN_C_GPIO = 27;  // The common pin
constexpr uint PIN_B_GPIO = 26;  // The B channel pin
Encoder enc(pio0, 0, {PIN_A_GPIO, PIN_B_GPIO}, PIN_C_GPIO);

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
    constexpr uint I2C_SDA_GPIO = 20;
    constexpr uint I2C_SCL_GPIO = 21;

    i2c_init(i2c0, 1000000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    sleep_ms(250); // wait for display to power up
}

const char sound_names[128][33] = {
    /*   1 */ "Acoustic Piano",
    /*   2 */ "Bright Piano",
    /*   3 */ "Elec.Grand Piano",
    /*   4 */ "Honky-tonk Piano",
    /*   5 */ "Electric Piano 1",
    /*   6 */ "Electric Piano 2",
    /*   7 */ "Harpsichord",
    /*   8 */ "Clavinet",
    /*   9 */ "Celesta",
    /*  10 */ "Glockenspiel",
    /*  11 */ "Music Box",
    /*  12 */ "Vibraphone",
    /*  13 */ "Marimba",
    /*  14 */ "Xylophone",
    /*  15 */ "Tubular Bells",
    /*  16 */ "Dulcimer",
    /*  17 */ "Drawbar Organ",
    /*  18 */ "Percussive Organ",
    /*  19 */ "Rock Organ",
    /*  20 */ "Church Organ",
    /*  21 */ "Reed Organ",
    /*  22 */ "Accordion",
    /*  23 */ "Harmonica",
    /*  24 */ "Tango Accordion",
    /*  25 */ "Acou.Guitar(nylon)",
    /*  26 */ "Acou.Guitar(steel)",
    /*  27 */ "Elec.Guitar(jazz)",
    /*  28 */ "Elec.Guitar(clean)",
    /*  29 */ "Elec.Guitar(muted)",
    /*  30 */ "Overdriv. Guitar",
    /*  31 */ "Dist. Guitar",
    /*  32 */ "Guitar Harmonics",
    /*  33 */ "Acoustic Bass",
    /*  34 */ "Elec.Bass(finger)",
    /*  35 */ "Elec.Bass(pick)",
    /*  36 */ "Fretless Bass",
    /*  37 */ "Slap Bass 1",
    /*  38 */ "Slap Bass 2",
    /*  39 */ "Synth Bass 1",
    /*  40 */ "Synth Bass 2",
    /*  41 */ "Violin",
    /*  42 */ "Viola",
    /*  43 */ "Cello",
    /*  44 */ "Contrabass",
    /*  45 */ "Tremolo Strings",
    /*  46 */ "Pizzicato Strings",
    /*  47 */ "Orchestral Harp",
    /*  48 */ "Timpani",
    /*  49 */ "Str. Ensemble 1",
    /*  50 */ "Str. Ensemble 2",
    /*  51 */ "Synth Strings 1",
    /*  52 */ "Synth Strings 2",
    /*  53 */ "Choir Aahs",
    /*  54 */ "Voice Oohs",
    /*  55 */ "Synth Choir",
    /*  56 */ "Orchestra Hit",
    /*  57 */ "Trumpet",
    /*  58 */ "Trombone",
    /*  59 */ "Tuba",
    /*  60 */ "Muted Trumpet",
    /*  61 */ "French Horn",
    /*  62 */ "Brass Section",
    /*  63 */ "Synth Brass 1",
    /*  64 */ "Synth Brass 2",
    /*  65 */ "Soprano Sax",
    /*  66 */ "Alto Sax",
    /*  67 */ "Tenor Sax",
    /*  68 */ "Baritone Sax",
    /*  69 */ "Oboe",
    /*  70 */ "English Horn",
    /*  71 */ "Bassoon",
    /*  72 */ "Clarinet",
    /*  73 */ "Piccolo",
    /*  74 */ "Flute",
    /*  75 */ "Recorder",
    /*  76 */ "Pan Flute",
    /*  77 */ "Blown Bottle",
    /*  78 */ "Shakuhachi",
    /*  79 */ "Whistle",
    /*  80 */ "Ocarina",
    /*  81 */ "Lead 1 (square)",
    /*  82 */ "Lead 2 (sawtooth)",
    /*  83 */ "Lead 3 (calliope)",
    /*  84 */ "Lead 4 (chiff)",
    /*  85 */ "Lead 5 (charang)",
    /*  86 */ "Lead 6 (voice)",
    /*  87 */ "Lead 7 (fifths)",
    /*  88 */ "Lead 8 (bass+lead)",
    /*  89 */ "Pad 1 (new age)",
    /*  90 */ "Pad 2 (warm)",
    /*  91 */ "Pad 3 (polysynth)",
    /*  92 */ "Pad 4 (choir)",
    /*  93 */ "Pad 5 (bowed)",
    /*  94 */ "Pad 6 (metallic)",
    /*  95 */ "Pad 7 (halo)",
    /*  96 */ "Pad 8 (sweep)",
    /*  97 */ "FX 1 (rain)",
    /*  98 */ "FX 2 (soundtrack)",
    /*  99 */ "FX 3 (crystal)",
    /* 100 */ "FX 4 (atmosphere)",
    /* 101 */ "FX 5 (brightness)",
    /* 102 */ "FX 6 (goblins)",
    /* 103 */ "FX 7 (echoes)",
    /* 104 */ "FX 8 (sci-fi)",
    /* 105 */ "Sitar",
    /* 106 */ "Banjo",
    /* 107 */ "Shamisen",
    /* 108 */ "Koto",
    /* 109 */ "Kalimba",
    /* 110 */ "Bagpipe",
    /* 111 */ "Fiddle",
    /* 112 */ "Shanai",
    /* 113 */ "Tinkle Bell",
    /* 114 */ "Agogo",
    /* 115 */ "Steel Drums",
    /* 116 */ "Woodblock",
    /* 117 */ "Taiko Drum",
    /* 118 */ "Melodic Tom",
    /* 119 */ "Synth Drum",
    /* 120 */ "Reverse Cymbal",
    /* 121 */ "Gtr.Fret Noise",
    /* 122 */ "Breath Noise",
    /* 123 */ "Seashore",
    /* 124 */ "Bird Tweet",
    /* 125 */ "Telephone Ring",
    /* 126 */ "Helicopter",
    /* 127 */ "Applause",
    /* 128 */ "Gunshot",
};

static void display_sound_name(pico_ssd1306::SSD1306 * pDisplay, uint8_t sound)
{
    using namespace pico_ssd1306;

    pDisplay->clear();

    // Top line is the General MIDI sound number
    char sound_num_str[4];
    snprintf(sound_num_str, sizeof(sound_num_str), "%d", sound + 1);
    drawText(pDisplay, font_12x16, sound_num_str, 0, 0);

    // Bottom line is the sound name
    const char *sound_name = sound_names[sound];
    drawText(pDisplay, font_8x8, sound_name, 0, 24);
    pDisplay->sendBuffer();

    printf("Sound = %d: %s\n", sound + 1, sound_name);
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

inline u_int32_t positive_modulo(int32_t dividend, uint32_t divisor) {
    return (dividend % divisor + divisor) % divisor;
}

int main() {
    bi_decl(bi_program_description("Provide a USB host interface for Serial Port MIDI."));
    bi_decl(bi_1pin_with_name(LED_GPIO, "On-board LED"));
    bi_decl(bi_2pins_with_names(MIDI_UART_TX_GPIO, "MIDI UART TX", MIDI_UART_RX_GPIO, "MIDI UART RX"));

    stdio_init_all();

    // Sleep 8 seconds to give enough time to connect up a terminal
    sleep_ms(8000);

    board_init();
    blink_led();
    printf("Pico MIDI Host to MIDI UART Adapter\r\n");
    tusb_init();

    // Map the pins to functions
    midi_uart_instance = midi_uart_configure(MIDI_UART_NUM, MIDI_UART_TX_GPIO, MIDI_UART_RX_GPIO);
    printf("Configured MIDI UART %u for 31250 baud\r\n", MIDI_UART_NUM);

    // Uncomment the below line to reverse the counting direction
    // enc.direction(REVERSED_DIR);

    enc.init();

    // Create a new display object
    init_display();
    // pico_ssd1306::SSD1306 display(i2c0, 0x3D, pico_ssd1306::Size::W128xH64);
    pico_ssd1306::SSD1306 display(i2c0, 0x3C, pico_ssd1306::Size::W128xH32);
    display.setOrientation(0);
    display_sound_name(&display, sound);

    while (1) {
        tuh_task();

        if (enc.delta() != 0) {
            sound = positive_modulo(enc.count(), GM_PROGRAM_NUMBER_SIZE);
            write_midi_uart_program_change(sound);
            display_sound_name(&display, sound);
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
