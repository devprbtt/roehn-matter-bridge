========================================
 ROEHN Matter Bridge v1.0.15
 ESP32-S3 DevKitC-1 N16R8
========================================

Changes in v1.0.15:
  - Per-device GETLOAD (Savant protocol style)
  - STA keep-alive every 10 seconds
  - MOD diagnostics every 57 seconds
  - REFRESH on connect for full dump
  - Per-device poll batching
  - Fix memset sizeof bug (ESP32-C6 build)

HOW TO FLASH
------------

Prerequisites:
  - esptool.py installed (pip install esptool)
  - Put the board in download mode
  - Connect via USB

Erase + Flash (all-in-one):

  esptool.py -p COM4 -b 460800 --before default_reset --after hard_reset write_flash ^
    0x0000 bootloader.bin ^
    0xc000 partition-table.bin ^
    0x1d000 ota_data_initial.bin ^
    0x20000 roehn-matter-bridge.bin

REPLACE "COM4" with your actual COM port.

Files in this folder:
  bootloader.bin         - ESP32-S3 bootloader (offset 0x0000)
  partition-table.bin    - Partition table (offset 0xC000)
  ota_data_initial.bin   - OTA data initial (offset 0x1D000)
  roehn-matter-bridge.bin - Firmware (offset 0x20000)
