# TUI 2026 — Audio Cube

Two openable cubes: you speak into one, the other plays the message back.
Both run on an Arduino Nano ESP32 and talk over their own WiFi network.

- `recording/` — the recorder (microphone + rotary cube sensor)
- `listen/` — the player (speaker via I2S + 12-pixel CJMCU-2812 LED ring on D8)

## Arduino IDE settings

- Board: Arduino Nano ESP32
- **Tools → Pin Numbering → "By GPIO number (legacy)"** — required for the
  Adafruit NeoPixel library, otherwise linking fails with
  `undefined reference to digitalPinToGPIONumber`.
- Library: Adafruit NeoPixel
