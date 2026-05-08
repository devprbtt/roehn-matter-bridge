# roehn-matter-bridge

ESP32-C6 Matter bridge for ROEHN gateways.

Current scaffold:

- ESP-Matter light endpoints generated from persisted ROEHN discovery data
- ROEHN UDP discovery and connected-device enumeration
- ROEHN TCP `LOAD` control for relay and dimmer channels
- Embedded setup UI for gateway config, discovery, status, and discovered light inventory
- PlatformIO target for `esp32-c6-devkitc-1` with 16MB flash

The project intentionally lives beside `esp32-ir-server-matter` so the original HVAC bridge remains untouched.
