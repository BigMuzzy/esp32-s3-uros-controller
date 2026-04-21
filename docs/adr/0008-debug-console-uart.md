# ADR-0008: Debug Console on SH1.0 UART (GPIO1 / GPIO2)

**Status:** Proposed
**Date:** 2026-04-20

## Context

ESP-IDF's default console routes `printf` / `ESP_LOG*` output to UART0
on GPIO43 (TX) and GPIO44 (RX). On the Waveshare ESP32-S3-RS485-CAN
those pins are exposed on the 20-pin header (ADR-0006, revised).

The current `sdkconfig` has:

```
CONFIG_ESP_CONSOLE_UART_DEFAULT=y
CONFIG_ESP_CONSOLE_UART_NUM=0
CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG=y
```

Two problems with keeping this as-is:

1. **USB port conflict.** The secondary console uses the built-in
   USB-Serial/JTAG on GPIO19/20. Those pins are also the USB-CDC
   transport for micro-ROS (ADR-0002). During normal operation the
   agent owns the port, so the secondary console is unusable *and*
   risks interleaving log bytes into the DDS stream.
2. **Header real estate.** GPIO43/44 are on the main 20-pin header
   next to the application I/O. Using them for the console means
   running console wires alongside RC PWM / signal wiring, and the
   console cannot be disconnected without also disturbing the
   application harness.

The board also exposes GPIO1 and GPIO2 on a dedicated 4-pin SH1.0
connector (ADR-0006, revised). That connector is physically separate
from the main header and is a natural breakout for a debug UART.

## Decision

Move the ESP-IDF console to a custom UART on the SH1.0 connector:

| Signal | Pin        |
|--------|------------|
| TX     | GPIO1      |
| RX     | GPIO2      |
| UART   | UART0      |
| Baud   | 115200 8N1 |

UART0 is retained (rather than switching to UART1) so that the ROM
bootloader, 2nd-stage bootloader, and app all share one console
peripheral. Only the pin mapping changes — UART0 is routed to
GPIO1/2 through the GPIO matrix.

Disable the secondary USB-Serial/JTAG console to prevent conflicts
with the micro-ROS USB-CDC transport.

### sdkconfig changes

```
# Primary console: UART0 on GPIO1/2
CONFIG_ESP_CONSOLE_UART_DEFAULT=n
CONFIG_ESP_CONSOLE_UART_CUSTOM=y
CONFIG_ESP_CONSOLE_UART_NUM=0
CONFIG_ESP_CONSOLE_UART_TX_GPIO=1
CONFIG_ESP_CONSOLE_UART_RX_GPIO=2
CONFIG_ESP_CONSOLE_UART_BAUDRATE=115200

# No secondary console (USB is owned by micro-ROS)
CONFIG_ESP_CONSOLE_SECONDARY_NONE=y
# CONFIG_ESP_CONSOLE_SECONDARY_USB_SERIAL_JTAG is not set
```

These are applied via `idf.py menuconfig` →
*Component config → ESP System Settings → Channel for console output →
Custom UART*, and *Channel for console secondary output → No secondary
console output*.

### Physical connection

A 3.3 V USB-UART adapter (e.g. CP2102, FT232 at 3.3 V logic) connects
to the SH1.0 connector:

| Adapter | SH1.0 pin |
|---------|-----------|
| GND     | GND       |
| RX      | GPIO1 (TX) |
| TX      | GPIO2 (RX) |
| —       | 3.3 V (leave unconnected; board self-powered) |

**Do not connect a 5 V logic adapter** — GPIO1/2 are 3.3 V only.

## Alternatives Considered

### A. Keep UART0 on GPIO43/44 (default)
Simplest — no Kconfig changes. Rejected because it mixes console
wiring with application I/O on the 20-pin header and leaves the
USB-Serial/JTAG secondary console enabled, which conflicts with
micro-ROS on USB-CDC.

### B. Disable the console entirely (`CONFIG_ESP_CONSOLE_NONE`)
Smallest footprint, no UART peripheral consumed. Rejected because
it removes all diagnostic output — bring-up, field debugging, and
crash backtraces all become blind. The SH1.0 connector is essentially
free cost.

### C. Use UART1 on GPIO1/2, leave UART0 on GPIO43/44
Keeps the ROM bootloader log on GPIO43/44 and puts only the app
console on GPIO1/2. Rejected: splits log output across two physical
ports, doubling the adapter count during debugging, with no real
benefit — ROM log is only a few lines at boot.

### D. Route console over RS485
The board has an isolated RS485 transceiver on GPIO17/18/21. Would
give galvanic isolation for the debug link. Rejected as overkill for
a development console; adds an RS485-to-USB adapter to the toolchain
and consumes pins that could later host a Modbus / field bus.

### E. USB-Serial/JTAG only
Use the built-in USB-Serial/JTAG as the sole console. Rejected —
conflicts with micro-ROS USB-CDC transport (ADR-0002). The USB port
cannot host both at once in production.

## Consequences

- GPIO1 and GPIO2 are reserved for the debug console and removed
  from the pool of free application I/O.
- The USB-C port is owned exclusively by micro-ROS; no log bytes
  leak into the DDS stream.
- Developers need a 3.3 V USB-UART adapter and an SH1.0 pigtail to
  read the console. These are inexpensive and off-the-shelf.
- The **ROM bootloader**'s earliest messages (before the 2nd-stage
  bootloader reconfigures UART pins) still appear on the chip's
  default UART0 pads (GPIO43/44). In practice this is a handful of
  bytes at power-on and is not needed for normal debugging; both
  the 2nd-stage bootloader and the app honor the custom pin setting
  and output to GPIO1/2.
- `idf.py monitor` still works — just point it at the USB-UART
  adapter's tty (e.g. `idf.py -p /dev/ttyUSB0 monitor`) rather than
  the board's USB-C port.
- The existing statement in ADR-0002 that "serial monitor / logging
  should use UART0" remains correct; this ADR makes it concrete by
  pinning UART0 to GPIO1/2.
