# ASCENT R3 Firmware

Guidance, navigation, and control firmware for the ASCENT R3 avionics stack built on ESP-IDF (ESP32-S3).

## Quick start

1. Install and configure ESP-IDF for your platform.
2. Plug in an ASCENT R3 board to an available USB port.
3. Flash and monitor using the ESP-IDF tooling (VSCode extension or CLI).

## Repository layout

- `main/`: Application entry point and feature modules (FSM, radio, sensors, memory).
- `ascent-r3-drivers/`: Submodule containing device drivers.
- `goober/`: Submodule for packet radio communication.
- `sitl/`: Software-in-the-loop utilities.

## Notes

- The firmware in the `dev` branch is not flight-proven and should not be used.
- The firmware in the `flight` branch has flown successfully and has passed all testing done by the SEDS-RD team.

## License

ASCENT R3 firmware uses the MIT license. See the `LICENSE` file in the repo.