# Earlier ESP32 controller variant

`spider_controller_debug.ino` is retained to show the project's development history. Its UART assignment uses GPIO 34 as RX and GPIO 35 as TX. On a standard ESP32, GPIO 35 is input-only, so this version cannot transmit UART data on that pin. The main sketch in `../spider_controller/` uses GPIO 16/17 and should be the starting point for hardware work.

The device-specific Bluetooth address in the original pasted sketch has been replaced with a placeholder in this archive copy.
