# Organization notes

Source: user-supplied ESP32-S3 knob `main.c`

## What changed

- Added a top-level architecture and threading/data-flow guide.
- Grouped includes by purpose and documented why each family is present.
- Added subsystem subsection headers throughout the long single-file source.
- Added Doxygen-style comments to previously undocumented UI callbacks,
  diagnostic page constructors, LVGL bridge functions, encoder tasks, and
  `app_main()`.
- Rewrote stale microphone/audio documentation to match the current PDM +
  PCM5100A implementation.
- Corrected the stale SD comment that claimed GPIO42 was also a microphone pin.
  The conservative runtime guard itself was deliberately left unchanged.
- Corrected menu-button documentation to match the actual code: both the
  haptic callback and action callback use `LV_EVENT_PRESSED`.
- Removed old v8/v10/v11/v12 wording from comments so explanations describe the
  current architecture rather than its revision history.
- Expanded comments around LVGL DMA buffers, partial rendering, display flush,
  touch rotation, the LVGL mutex, encoder task hand-off, ADC worker, and audio
  pin ownership.

## Runtime behavior

This pass was intentionally documentation-only. A comments-aware comparison
verified that the executable token stream of the organized file is identical to
the uploaded source.

- Original lines: 4819
- Organized/commented lines: 5333
- Executable-token equivalence: **PASS**

## Suggested future refactor

The file is now easier to navigate as one compilation unit. The next structural
step, once the hardware tests are stable, would be to split it into modules such
as:

- `ui_menu.c/.h`
- `diagnostic_audio.c/.h`
- `diagnostic_sd.c/.h`
- `diagnostic_battery.c/.h`
- `drv2605.c/.h`
- `lvgl_port.c/.h`

I did not do that in this pass because moving static functions and shared state
across translation units would be a real code refactor rather than a safe
documentation/organization pass.
