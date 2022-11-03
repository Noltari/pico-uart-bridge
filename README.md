Raspberry Pi Pico USB-UART Bridge
=================================

This program bridges the Raspberry Pi Pico HW UARTs to two independent USB CDC serial devices in order to behave like any other USB-to-UART Bridge controllers.

Disclaimer
----------

This software is provided without warranty, according to the MIT License, and should therefore not be used where it may endanger life, financial stakes, or cause discomfort and inconvenience to others.

Raspberry Pi Pico Pinout
------------------------

UART0:
| Raspberry Pi Pico GPIO | Function  |
|:----------------------:|:---------:|
| GPIO0 (Pin 1)          | TX        |
| GPIO1 (Pin 2)          | RX        |
| GPIO2 (Pin 4)          | CTS       |
| GPIO3 (Pin 5)          | RTS       |
| GPIO4 (Pin 6)          | DTR       |
| GPIO5 (Pin 7)          | DSR       |

UART1:
| Raspberry Pi Pico GPIO | Function  |
|:----------------------:|:---------:|
| GPIO8  (Pin 11)        | TX        |
| GPIO9  (Pin 12)        | RX        |
| GPIO10 (Pin 14)        | CTS       |
| GPIO11 (Pin 15)        | RTS       |
| GPIO12 (Pin 16)        | DTR       |
| GPIO13 (Pin 17)        | DSR       |

Optional Hardware Flow and Line control
------------------------------

Hardware Flow-control (RTS/CTS) is disabled by default, but can be compiled in by running:

``` bash
cmake -DFLOW_CONTROL .
make
```

Line control (DTR/DSR) is disabled by default, but can be compiled in by running:

``` bash
cmake -DLINE_CONTROL .
make
```

To enable both:
``` bash
cmake -DLINE_CONTROL -DFLOW_CONTROL .
make
```
