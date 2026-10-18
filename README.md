Raspberry Pi Pico USB-UART Bridge
=================================

This program bridges the Raspberry Pi Pico HW UARTs to two independent USB CDC serial devices in order to behave like any other USB-to-UART Bridge controllers.

Disclaimer
----------

This software is provided without warranty, according to the MIT License, and should therefore not be used where it may endanger life, financial stakes, or cause discomfort and inconvenience to others.

Raspberry Pi Pico Pinout
------------------------

| Raspberry Pi Pico GPIO | Function |
|:----------------------:|:--------:|
| GPIO16 (Pin 21)        | UART0 TX |
| GPIO17 (Pin 22)        | UART0 RX |
| GPIO4 (Pin 6)          | UART1 TX |
| GPIO5 (Pin 7)          | UART1 RX |

Board selection
---------------

Select the board with `PICO_BOARD`; the firmware uses the Pico SDK board
definition and the SDK's `pico_status_led` library for the onboard status LED.
The LED is on while at least one USB serial port is open. The library handles
GPIO, WS2812, and supported wireless-chip LEDs. Boards without a supported LED
run without a status LED. RGB LEDs use the SDK's default color.

| `PICO_BOARD` | UART0 TX / RX | UART1 TX / RX | Status LED |
|:-------------|:--------------|:--------------|:-----------|
| `pico` (default) | GP16 / GP17 | GP4 / GP5 | GPIO LED, GP25 |
| `pico2` | GP16 / GP17 | GP4 / GP5 | GPIO LED, GP25 |
| `waveshare_rp2040_zero` | GP0 / GP1 | GP4 / GP5 | WS2812, GP16 |
| `waveshare_rp2040_one` | GP0 / GP1 | GP4 / GP5 | WS2812, GP16 |
| `waveshare_rp2350_zero` | GP0 / GP1 | GP4 / GP5 | WS2812, GP16 |
| `pimoroni_tiny2040_2mb` (2 MB) | GP0 / GP1 | GP4 / GP5 | Green GPIO LED, GP19 (active low) |
| `pimoroni_tiny2040` (8 MB) | GP0 / GP1 | GP4 / GP5 | Green GPIO LED, GP19 (active low) |

The bridge preserves its original Pico UART wiring rather than adopting the
SDK's default UART pins for every board. Other SDK boards use that wiring unless
overridden; check pin availability and conflicts with onboard peripherals.

To override individual UART pins, configure CMake directly, for example:

```sh
cmake -S . -B build/custom -DPICO_BOARD=pico \
  -DBOARD_UART0_TX_PIN=0 -DBOARD_UART0_RX_PIN=1
cmake --build build/custom
```

`BOARD_UART1_TX_PIN` and `BOARD_UART1_RX_PIN` are also available. Select pins
that support the corresponding hardware UART.

Build for Raspberry Pi Pico / Pico 2
------------------------------------

Prerequisites:
- CMake
- ARM GCC toolchain (e.g. `arm-none-eabi-gcc`)
- Python 3

Build steps:
1. Initialize submodules if you haven't already:
   - `git submodule update --init --recursive`
2. Run the build script (defaults to Pico 1):
   - `./build.sh`

The `.uf2` output will be created at `build/<board>/uart_bridge.uf2`.

Board override:
- To build for Pico 2, set `PICO_BOARD`:
  - `PICO_BOARD=pico2 ./build.sh`
- To build for Waveshare RP2040-Zero:
  - `PICO_BOARD=waveshare_rp2040_zero ./build.sh`
