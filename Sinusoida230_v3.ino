/*
 * Waveshare ESP32-S3-LCD-4.3, SKU 30493, bez dotyku.
 * Akwizycja ADC, przetwarzanie sygnalu i publikacja LVGL sa rozdzielone.
 * Wygladzanie dotyczy tylko prezentacji; diagnostyka pozostaje surowa.
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <cstring>

#include "lvgl_v8_port.h"
#include "sine_view.h"
#include "adc_acquisition.h"
#include "signal_processing_service.h"
#include "waveform_self_test.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

#if LVGL_PORT_TASK_CORE != 1
#error "Przywroc LVGL na Core 1 i ustaw Arduino Runs On: Core 1."
#endif

static acquisition::Service sampler;
static signal_processing::Service processor(sampler);
// Pelna diagnostyka UART jest kosztowna przy 115200 bit/s. Wlacz tylko na
// czas pomiarow wydajnosci; liczniki pozostaja zbierane w kazdej ramce.
constexpr bool kPeriodicDiagnostics = false;
// Pojedyncza odrzucona ramka nie kasuje ostatniego poprawnego pomiaru.
// HOLD oznacza dopiero trwaly brak wiarygodnych danych.
constexpr uint32_t kHoldAfterNoValidMs = 2000;
constexpr uint32_t kMissingFrameWarningMs = 750;
constexpr uint32_t kMissingFrameFault = 1UL << 31;

struct QualityCounters {
    uint32_t i2c = 0;
    uint32_t stale = 0;
    uint32_t timing = 0;
    uint32_t signal = 0;
    uint32_t periodRange = 0;
    uint32_t periodJitter = 0;
    uint32_t missingFrame = 0;
};

struct GuiMailboxMetrics {
    uint32_t published = 0;
    uint32_t updates = 0;
    uint32_t coalesced = 0;
    bool pending = false;
};

static portMUX_TYPE guiMailboxMux = portMUX_INITIALIZER_UNLOCKED;
static measurement::GuiSnapshot guiMailboxSnapshot;
static measurement::GuiSnapshot guiApplySnapshot;
static bool guiMailboxPending = false;
static uint32_t guiPublished = 0;
static uint32_t guiUpdated = 0;
static uint32_t guiMailboxCoalesced = 0;

// Diagnostyka nie przechowuje tablicy punktow. Pozwala zwolnic DisplayFrame
// przed formatowaniem i wysylaniem kosztownego logu UART.
struct FrameDiagnostics {
    uint32_t sequence = 0;
    bool valid = false;
    measurement::MeasurementRecord measurement;
    measurement::AcquisitionMetrics acquisition;
    measurement::ProcessingMetrics processing;
};

FrameDiagnostics captureDiagnostics(const measurement::DisplayFrame &frame) {
    FrameDiagnostics result;
    result.sequence = frame.sequence;
    result.valid = frame.valid;
    result.measurement = frame.measurement;
    result.acquisition = frame.acquisition;
    result.processing = frame.processing;
    return result;
}

void copyGuiSnapshot(
    const measurement::DisplayFrame &frame,
    measurement::GuiSnapshot &snapshot
) {
    snapshot.sequence = frame.sequence;
    std::memcpy(snapshot.y, frame.y, sizeof(snapshot.y));
    snapshot.rms_y = frame.rms_y;
    std::memcpy(
        snapshot.rms_value_text,
        frame.rms_value_text,
        sizeof(snapshot.rms_value_text)
    );
    std::memcpy(
        snapshot.maximum_value_text,
        frame.maximum_value_text,
        sizeof(snapshot.maximum_value_text)
    );
    std::memcpy(
        snapshot.frequency_value_text,
        frame.frequency_value_text,
        sizeof(snapshot.frequency_value_text)
    );
    snapshot.frequency = frame.measurement.frequency;
}

void queueGuiSnapshot(const measurement::DisplayFrame &frame) {
    portENTER_CRITICAL(&guiMailboxMux);
    if (guiMailboxPending) ++guiMailboxCoalesced;
    copyGuiSnapshot(frame, guiMailboxSnapshot);
    guiMailboxPending = true;
    ++guiPublished;
    portEXIT_CRITICAL(&guiMailboxMux);
}

bool applyPendingGui(void *) {
    portENTER_CRITICAL(&guiMailboxMux);
    if (!guiMailboxPending) {
        portEXIT_CRITICAL(&guiMailboxMux);
        return false;
    }
    guiApplySnapshot = guiMailboxSnapshot;
    guiMailboxPending = false;
    portEXIT_CRITICAL(&guiMailboxMux);

    sine_view::updatePrepared(
        guiApplySnapshot.y,
        guiApplySnapshot.rms_y,
        guiApplySnapshot.rms_value_text,
        guiApplySnapshot.maximum_value_text,
        guiApplySnapshot.frequency_value_text,
        guiApplySnapshot.frequency
    );
    portENTER_CRITICAL(&guiMailboxMux);
    ++guiUpdated;
    portEXIT_CRITICAL(&guiMailboxMux);
    return true;
}

GuiMailboxMetrics getGuiMailboxMetrics() {
    GuiMailboxMetrics metrics;
    portENTER_CRITICAL(&guiMailboxMux);
    metrics.published = guiPublished;
    metrics.updates = guiUpdated;
    metrics.coalesced = guiMailboxCoalesced;
    metrics.pending = guiMailboxPending;
    portEXIT_CRITICAL(&guiMailboxMux);
    return metrics;
}

void incrementSaturating(uint32_t &counter) {
    if (counter != UINT32_MAX) ++counter;
}

void countQuality(uint32_t quality, QualityCounters &counters) {
    if (quality & measurement::kI2cError) incrementSaturating(counters.i2c);
    if (quality & measurement::kStaleBlock) incrementSaturating(counters.stale);
    if (quality & measurement::kSampleGap) incrementSaturating(counters.timing);
    if (quality & measurement::kNoStableTrigger) incrementSaturating(counters.signal);
    if (quality & measurement::kPeriodOutOfRange) incrementSaturating(counters.periodRange);
    if (quality & measurement::kPeriodJitter) incrementSaturating(counters.periodJitter);
}

void logQualityCounters(const QualityCounters &counters) {
    Serial.printf(
        "QUALITY_COUNT i2c=%lu stale=%lu timing=%lu signal=%lu "
        "period_range=%lu period_jitter=%lu missing_frame=%lu\n",
        (unsigned long)counters.i2c,
        (unsigned long)counters.stale,
        (unsigned long)counters.timing,
        (unsigned long)counters.signal,
        (unsigned long)counters.periodRange,
        (unsigned long)counters.periodJitter,
        (unsigned long)counters.missingFrame
    );
}

void fatalError(const char *message) {
    Serial.println(message);
    Serial.flush();
    while (true) delay(1000);
}

void bootStage(const char *message) {
    Serial.println(message);
    Serial.flush();
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("Sinusoida230: start");

    if (!psramFound()) {
        fatalError("BLAD: brak PSRAM. Ustaw Tools > PSRAM > OPI PSRAM.");
    }
    bootStage("BOOT: PSRAM OK");

    static Board board;
    bootStage("BOOT: board.init start");
    if (!board.init()) fatalError("BLAD: board.init()");
    bootStage("BOOT: board.init OK");

#if LVGL_PORT_AVOID_TEARING_MODE
    auto lcd = board.getLCD();
    if (lcd == nullptr) fatalError("BLAD: board.getLCD() == nullptr");
    bootStage("BOOT: LCD framebuffer config start");
    if (!lcd->configFrameBufferNumber(LVGL_PORT_DISP_BUFFER_NUM)) {
        fatalError("BLAD: configFrameBufferNumber()");
    }
    bootStage("BOOT: LCD framebuffer config OK");

#if ESP_PANEL_DRIVERS_BUS_ENABLE_RGB && CONFIG_IDF_TARGET_ESP32S3
    auto lcdBus = lcd->getBus();
    if (lcdBus == nullptr) fatalError("BLAD: lcd->getBus() == nullptr");
    if (lcdBus->getBasicAttributes().type != ESP_PANEL_BUS_TYPE_RGB) {
        fatalError("BLAD: LCD nie korzysta z magistrali RGB");
    }
    bootStage("BOOT: RGB bounce buffer config start");
    if (!static_cast<BusRGB *>(lcdBus)->configRGB_BounceBufferSize(
            lcd->getFrameWidth() * 10
        )) {
        fatalError("BLAD: configRGB_BounceBufferSize()");
    }
    bootStage("BOOT: RGB bounce buffer config OK");
#endif
#endif

    bootStage("BOOT: board.begin start");
    if (!board.begin()) fatalError("BLAD: board.begin()");
    bootStage("BOOT: board.begin OK");
    bootStage("BOOT: lvgl_port_init start");
    if (!lvgl_port_init(board.getLCD(), board.getTouch())) {
        fatalError("BLAD: lvgl_port_init()");
    }
    bootStage("BOOT: lvgl_port_init OK");
    if (!lvgl_port_lock(-1)) fatalError("BLAD: mutex LVGL");
    bootStage("BOOT: sine_view::create start");
    sine_view::create();
    lvgl_port_unlock();
    if (!lvgl_port_set_update_callback(applyPendingGui, nullptr)) {
        fatalError("BLAD: callback publikacji GUI");
    }
    lvgl_port_request_refresh();
    bootStage("BOOT: UI ready");

    Serial.printf(
        "ADC %s, adres 0x%02X, host I2C=%d, SDA=8 SCL=9, DR=%lu SPS\n",
        adc_config::ads1015 ? "ADS1015" : "ADS1115",
        adc_config::address,
        adc_config::i2cPort,
        (unsigned long)adc_config::dataRate
    );
    Serial.println(
        "Kalibracja: mainsVoltsPerAdcVolt = U_RMS_referencyjne / ADC_RMS."
    );
    Serial.printf(
        "Pipeline: Arduino=%d ADC=%d/%u processing=%d/%u LVGL=%d; tick=%u Hz\n",
        xPortGetCoreID(),
        int(acquisition::kCore),
        unsigned(acquisition::kPriority),
        int(signal_processing::kCore),
        unsigned(signal_processing::kPriority),
        LVGL_PORT_TASK_CORE,
        unsigned(configTICK_RATE_HZ)
    );

    if (!waveform_self_test::run(Serial)) {
        fatalError("BLAD: deterministyczny test przetwarzania");
    }

    if (!sampler.begin()) {
        fatalError("BLAD: utworzenie zadania/kolejek ADC");
    }
    if (!processor.begin()) {
        fatalError("BLAD: utworzenie zadania/kolejek przetwarzania");
    }
}

const char *diagnosticState(uint32_t quality) {
    if (quality & measurement::kI2cError) return "I2C_ERROR";
    if (quality & measurement::kStaleBlock) return "STALE";
    if (quality & measurement::kSampleGap) return "TIMING";
    if (quality & measurement::kPeriodOutOfRange) return "PERIOD_RANGE";
    if (quality & measurement::kPeriodJitter) return "PERIOD_JITTER";
    if (quality & measurement::kNoStableTrigger) return "SIGNAL";
    return "OK";
}

void logDiagnostics(
    const FrameDiagnostics &frame
) {
    lvgl_port_metrics_t port{};
    lvgl_port_get_metrics(&port, true);
    const GuiMailboxMetrics gui = getGuiMailboxMetrics();
    const auto &a = frame.acquisition;
    const auto &p = frame.processing;
    Serial.printf(
        "ADC_DIAG seq=%lu state=%s n=%lu block_n=%lu "
        "gap=%lu us idx=%lu hist_gap=%lu us hist_idx=%lu "
        "boundary=%lu us zero_gap=%u between=%lu us "
        "read=%lu us write=%lu us poll=%lu us "
        "result=%lu us wait=%lu us block=%lu us "
        "dropped=%lu stack=%lu B error=%s "
        "mean_gap=%lu us hist_mean=%lu us "
        "events=%lu missed=%lu dup_ts=%lu nonmono_ts=%lu "
        "adc_q=%lu adc_errors=%lu "
        "processing=%lu us frame=%lu us proc_stack=%lu B "
        "frame_dropped=%lu frame_skipped=%lu frame_coalesced=%lu frame_q=%lu "
        "gui_published=%lu gui_updated=%lu gui_coalesced=%lu gui_pending=%u "
        "lvgl_update_max=%lu us lvgl_update_int=%lu us "
        "lvgl_hold_max=%lu us lvgl_hold_int=%lu us "
        "lv_timer_max=%lu us lv_timer_int=%lu us "
        "vsync_wait_max=%lu us vsync_wait_int=%lu us lvgl_fail=%lu\n",
        (unsigned long)frame.sequence,
        diagnosticState(frame.measurement.quality_flags),
        (unsigned long)a.sample_count,
        (unsigned long)a.block_sample_count,
        (unsigned long)a.gap_max_us,
        (unsigned long)a.gap_index,
        (unsigned long)a.history_gap_max_us,
        (unsigned long)a.history_gap_index,
        (unsigned long)a.boundary_gap_us,
        unsigned(a.history_zero_gap),
        (unsigned long)a.between_reads_max_us,
        (unsigned long)a.read_max_us,
        (unsigned long)a.write_max_us,
        (unsigned long)a.poll_max_us,
        (unsigned long)a.result_max_us,
        (unsigned long)a.wait_max_us,
        (unsigned long)a.block_us,
        (unsigned long)a.dropped_blocks,
        (unsigned long)a.stack_free_bytes,
        esp_err_to_name(static_cast<esp_err_t>(a.error)),
        (unsigned long)a.gap_mean_us,
        (unsigned long)a.history_gap_mean_us,
        (unsigned long)a.cadence_events,
        (unsigned long)a.missed_conversions,
        (unsigned long)a.duplicate_timestamps,
        (unsigned long)a.non_monotonic_timestamps,
        (unsigned long)a.ready_high_water,
        (unsigned long)a.error_count,
        (unsigned long)p.processing_us,
        (unsigned long)p.frame_prepare_us,
        (unsigned long)p.stack_free_bytes,
        (unsigned long)p.dropped_frames,
        (unsigned long)p.skipped_frames,
        (unsigned long)p.coalesced_frames,
        (unsigned long)p.ready_high_water,
        (unsigned long)gui.published,
        (unsigned long)gui.updates,
        (unsigned long)(gui.coalesced + p.coalesced_frames),
        unsigned(gui.pending),
        (unsigned long)port.application_update_max_us,
        (unsigned long)port.application_update_interval_max_us,
        (unsigned long)port.mutex_hold_max_us,
        (unsigned long)port.mutex_hold_interval_max_us,
        (unsigned long)port.timer_handler_max_us,
        (unsigned long)port.timer_handler_interval_max_us,
        (unsigned long)port.vsync_wait_max_us,
        (unsigned long)port.vsync_wait_interval_max_us,
        (unsigned long)port.failure_count
    );

    if (frame.valid) {
        const auto &m = frame.measurement;
        Serial.printf(
            "ADC_RMS=%.6f V ADC_PEAK=%.6f V "
            "OFFSET=%.4f V MIN=%.4f V MAX=%.4f V "
            "f=%.3f Hz fs=%.0f quality=0x%08lX\n",
            m.rms,
            m.peak,
            m.offset,
            m.minimum,
            m.maximum,
            m.frequency,
            m.sample_rate,
            (unsigned long)m.quality_flags
        );
    }
}

void loop() {
    static uint32_t lastValidMs = millis();
    static uint32_t lastFrameMs = millis();
    static uint32_t lastDiagnosticMs = 0;
    static uint32_t activeFault = measurement::kQualityNone;
    static bool holdVisible = false;
    static QualityCounters qualityCounters;

    measurement::DisplayFrame *frame =
        processor.takeLatest(pdMS_TO_TICKS(100));
    if (frame == nullptr) {
        const uint32_t nowMs = millis();
        if (uint32_t(nowMs - lastFrameMs) >= kMissingFrameWarningMs) {
            if (activeFault != kMissingFrameFault) {
                activeFault = kMissingFrameFault;
                incrementSaturating(qualityCounters.missingFrame);
                Serial.println("ADC_FAULT state=MISSING_FRAME");
                logQualityCounters(qualityCounters);
            }
            if (
                !holdVisible &&
                uint32_t(nowMs - lastValidMs) >= kHoldAfterNoValidMs
            ) {
                Serial.println("HOLD | brak nowej ramki pomiarowej");
                holdVisible = true;
            }
        }
        return;
    }

    const uint32_t nowMs = millis();
    lastFrameMs = nowMs;
    const bool logNow = kPeriodicDiagnostics &&
        uint32_t(nowMs - lastDiagnosticMs) >= 1000;
    const FrameDiagnostics diagnostics = captureDiagnostics(*frame);

    if (!frame->valid) {
        const uint32_t quality = frame->measurement.quality_flags;
        processor.release(frame);
        countQuality(quality, qualityCounters);

        if (activeFault != quality) {
            activeFault = quality;
            Serial.printf(
                "ADC_FAULT state=%s quality=0x%08lX\n",
                diagnosticState(quality),
                (unsigned long)quality
            );
            logDiagnostics(diagnostics);
            logQualityCounters(qualityCounters);
        }

        if (
            !holdVisible &&
            uint32_t(nowMs - lastValidMs) >= kHoldAfterNoValidMs
        ) {
            Serial.printf("HOLD | state=%s\n", diagnosticState(quality));
            holdVisible = true;
        }
        delay(5);
        return;
    }

    const bool recovered =
        activeFault != measurement::kQualityNone || holdVisible;
    queueGuiSnapshot(*frame);
    processor.release(frame);
    lvgl_port_request_refresh();

    if (recovered) {
        Serial.printf(
            "ADC_RECOVERY seq=%lu downtime=%lu ms\n",
            (unsigned long)diagnostics.sequence,
            (unsigned long)uint32_t(nowMs - lastValidMs)
        );
        logQualityCounters(qualityCounters);
        logDiagnostics(diagnostics);
        lastDiagnosticMs = nowMs;
    }
    activeFault = measurement::kQualityNone;
    holdVisible = false;
    lastValidMs = nowMs;

    if (logNow && !recovered) {
        logDiagnostics(diagnostics);
        lastDiagnosticMs = nowMs;
    }

    delay(5);
}
