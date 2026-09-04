# Waveshare_ESP32S3_Knob
This is an update to the Waveshare ESP32-S3 Knob Touch LCD 1.8", an encoder module with built in LCD and other peripherals. All the examples provided by Waveshare use LVGL v.8.4. This project updates it to use LVGL v.9.5.
AI was used heavily in its creation.

## Known Issues

### Media volume arc orientation

The media-control page is functional, but the volume arc is currently rendered with the wrong orientation.

Current behavior:
- Volume control itself works correctly.
- The displayed volume percentage tracks knob changes.
- The arc's fill/orientation is visually upside down.
- Attempts to correct the arc using LVGL reverse mode/value inversion have not yet produced the intended geometry.

Desired behavior:
- Arc gap at the top of the circular display.
- Low volume begins near the lower-left side.
- Increasing volume fills clockwise/upward around the left side and around the display.
- BLE status remains centered in the top gap.

This is a UI-only issue. BLE media controls and the companion A2DP/I2S1 audio path are working.

TODO: Correct LVGL arc start/end angles and fill direction without changing the logical volume value.