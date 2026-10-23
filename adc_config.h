#pragma once
#include <stddef.h>
#include <stdint.h>
namespace adc_config {
// ADS1015: true (12 bit). ADS1115: false (16 bit).
constexpr bool ads1015 = true;
constexpr uint8_t address = 0x48;
constexpr int i2cPort = 0; // Existing host initialized by Board/CH422G, SDA=8 SCL=9.
// Keep a 120 ms rolling analysis window, but deliver fresh samples to the
// processing task every 40 ms (25 updates/s).
constexpr uint32_t captureUs = 120000;
constexpr uint32_t acquisitionChunkUs = 40000;
constexpr size_t capacity = 512;
// Continuous-conversion data rates selected by adc_module.h.
constexpr uint32_t ads1015DataRate = 3300;
constexpr uint32_t ads1115DataRate = 860;
constexpr uint32_t dataRate = ads1015 ? ads1015DataRate : ads1115DataRate;
constexpr uint32_t conversionPeriodUs = (1000000UL + dataRate - 1) / dataRate;
constexpr float triggerHysteresisV = 0.03f;
// REQUIRED calibration: measured mains RMS / reported ADC AC RMS.
// 0 = uncalibrated. No mains RMS or mains peak is claimed in this mode.
constexpr float mainsVoltsPerAdcVolt = 402.42f;
// Display-only gain so the signal is visible on the +/-300 V RMS axis.
constexpr float previewGain = 300.0f;
}
