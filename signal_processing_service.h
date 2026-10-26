#pragma once

#include <Arduino.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include "adc_acquisition.h"
#include "measurement_types.h"
#include "sine_view.h"
#include "waveform_processing.h"

namespace signal_processing {

constexpr BaseType_t kCore = 0;
constexpr UBaseType_t kPriority = 2;
constexpr uint32_t kStackBytes = 8192;
// Jedna ramka moze byc wyswietlana, druga czekac w kolejce, a trzecia jest
// przygotowywana. Zapobiega to utracie aktualizacji podczas krotkiego VSYNC.
constexpr unsigned kFramePoolSize = 3;
constexpr float kCurveTauMs = 80.0f;
constexpr float kValuesTauMs = 160.0f;
constexpr uint32_t kPresentationResetMs = 500;
constexpr uint32_t kGuiPeriodMs = 33;
constexpr size_t kHarmonicCount = 3;
constexpr size_t kHarmonicFitPointCount = 192;

struct Rotation {
    float cosine;
    float sine;
};

// Obroty fazy dla harmonicznych 1, 3 i 5 na trzech pelnych okresach.
// Stale unikaja kosztownych sinf()/cosf() w kazdej ramce.
constexpr Rotation kFitRotation[kHarmonicCount] = {
    {0.995184727f, 0.098017140f},
    {0.956940336f, 0.290284677f},
    {0.881921264f, 0.471396737f},
};
constexpr Rotation kPixelRotation[kHarmonicCount] = {
    {0.999615827f, 0.027716385f},
    {0.996544216f, 0.083063989f},
    {0.990410431f, 0.138156355f},
};

static_assert(
    measurement::kDisplayPointCount == 681,
    "Harmonic pixel rotations require 680 display intervals"
);
static_assert(
    kHarmonicCount == 3,
    "Phase lock expects harmonic order 1, 3 and 5"
);

static_assert(
    measurement::kDisplayPointCount == sine_view::kPointCount,
    "Display-frame point count must match sine_view"
);

class Service {
public:
    explicit Service(acquisition::Service &source) : source_(source) {}

    bool begin() {
        if (task_ != nullptr) return false;

        free_ = xQueueCreate(kFramePoolSize, sizeof(measurement::DisplayFrame *));
        ready_ = xQueueCreate(1, sizeof(measurement::DisplayFrame *));
        if (free_ == nullptr || ready_ == nullptr) {
            cleanup();
            return false;
        }

        for (auto &frame : frames_) {
            measurement::DisplayFrame *pointer = &frame;
            if (xQueueSend(free_, &pointer, 0) != pdTRUE) {
                cleanup();
                return false;
            }
        }

        const BaseType_t result = xTaskCreatePinnedToCore(
            entry,
            "signal_process",
            kStackBytes,
            this,
            kPriority,
            &task_,
            kCore
        );
        if (result != pdPASS) {
            cleanup();
            return false;
        }
        return true;
    }

    measurement::DisplayFrame *takeLatest(TickType_t timeout) {
        measurement::DisplayFrame *frame = nullptr;
        if (xQueueReceive(ready_, &frame, timeout) != pdTRUE) return nullptr;
        return frame;
    }

    void release(measurement::DisplayFrame *frame) {
        configASSERT(frame != nullptr);
        const BaseType_t result = xQueueSend(free_, &frame, 0);
        configASSERT(result == pdTRUE);
    }

private:
    acquisition::Service &source_;
    measurement::DisplayFrame frames_[kFramePoolSize]{};
    float displayVolts_[measurement::kDisplayPointCount]{};
    waveform::Sample history_[adc_config::capacity]{};
    size_t historyCount_ = 0;

    QueueHandle_t free_ = nullptr;
    QueueHandle_t ready_ = nullptr;
    TaskHandle_t task_ = nullptr;

    bool presentationReady_ = false;
    uint32_t lastPresentationMs_ = 0;
    uint32_t lastPublishedMs_ = 0;
    float smoothRms_ = 0;
    float smoothPeak_ = 0;
    float smoothFrequency_ = 0;
    uint32_t droppedFrames_ = 0;
    uint32_t skippedFrames_ = 0;
    uint32_t readyHighWater_ = 0;

    void cleanup() {
        if (free_ != nullptr) vQueueDelete(free_);
        if (ready_ != nullptr) vQueueDelete(ready_);
        free_ = nullptr;
        ready_ = nullptr;
    }

    static void entry(void *context) {
        static_cast<Service *>(context)->run();
    }

    measurement::DisplayFrame *acquireWritableFrame() {
        measurement::DisplayFrame *frame = nullptr;
        if (xQueueReceive(free_, &frame, 0) == pdTRUE) return frame;

        // Reclaim only an obsolete frame still owned by the latest-only queue.
        if (xQueueReceive(ready_, &frame, 0) == pdTRUE) {
            ++droppedFrames_;
            return frame;
        }
        return nullptr;
    }

    static uint32_t qualityFor(waveform::AnalysisStatus status) {
        using waveform::AnalysisStatus;
        switch (status) {
            case AnalysisStatus::Ok: return measurement::kQualityNone;
            case AnalysisStatus::SampleGap: return measurement::kSampleGap;
            case AnalysisStatus::PeriodOutOfRange:
                return measurement::kPeriodOutOfRange;
            case AnalysisStatus::PeriodJitter: return measurement::kPeriodJitter;
            case AnalysisStatus::InvalidArguments:
            case AnalysisStatus::NoStableTrigger:
            default:
                return measurement::kNoStableTrigger;
        }
    }

    static const char *holdText(uint32_t quality) {
        if (quality & measurement::kI2cError)
            return "HOLD | blad I2C/ADC; szczegoly w monitorze UART";
        if (quality & measurement::kStaleBlock)
            return "HOLD | blok ADC nieaktualny";
        if (quality & measurement::kSampleGap)
            return "HOLD | opoznienie akwizycji >5 ms; sprawdz ADC_DIAG";
        if (quality & measurement::kPeriodOutOfRange)
            return "HOLD | okres poza zakresem 40-80 Hz";
        if (quality & measurement::kPeriodJitter)
            return "HOLD | niestabilny okres przebiegu";
        return "HOLD | brak stabilnego wyzwolenia 40-80 Hz";
    }

    static const char *diagnosticState(uint32_t quality) {
        if (quality & measurement::kI2cError) return "I2C_ERROR";
        if (quality & measurement::kStaleBlock) return "STALE";
        if (quality & measurement::kSampleGap) return "TIMING";
        if (quality & measurement::kPeriodOutOfRange) return "PERIOD_RANGE";
        if (quality & measurement::kPeriodJitter) return "PERIOD_JITTER";
        if (quality & measurement::kNoStableTrigger) return "SIGNAL";
        return "OK";
    }

    static void rotatePhase(float &cosine, float &sine, Rotation rotation) {
        const float previousCosine = cosine;
        cosine = previousCosine * rotation.cosine - sine * rotation.sine;
        sine = sine * rotation.cosine + previousCosine * rotation.sine;
    }

    void copyAcquisition(
        const acquisition::Block &block,
        size_t analysisSampleCount,
        const waveform::GapInfo &historyGaps,
        uint32_t boundaryGapUs,
        measurement::AcquisitionMetrics &out
    ) {
        const auto &d = block.diagnostic;
        out.sample_count = static_cast<uint32_t>(analysisSampleCount);
        out.block_sample_count = static_cast<uint32_t>(block.count);
        out.gap_mean_us = d.gapCount == 0
            ? 0
            : static_cast<uint32_t>(d.gapTotalUs / d.gapCount);
        out.gap_max_us = d.gapMaxUs;
        out.gap_index = d.gapIndex;
        out.history_gap_mean_us = historyGaps.count == 0
            ? 0
            : static_cast<uint32_t>(historyGaps.totalUs / historyGaps.count);
        out.history_gap_max_us = historyGaps.maximumUs;
        out.history_gap_index = historyGaps.maximumIndex;
        out.boundary_gap_us = boundaryGapUs;
        out.history_zero_gap = historyGaps.hasZero;
        out.between_reads_max_us = d.betweenReadsMaxUs;
        out.read_max_us = d.readMaxUs;
        out.write_max_us = d.writeMaxUs;
        out.poll_max_us = d.pollMaxUs;
        out.result_max_us = d.resultMaxUs;
        out.wait_max_us = d.waitMaxUs;
        out.block_us = d.elapsedUs;
        out.dropped_blocks = block.droppedBefore;
        out.ready_high_water = d.readyHighWater;
        out.stack_free_bytes = d.stackFreeBytes;
        out.error_count = d.errorCount;
        out.missed_conversions = d.missedConversions;
        out.error = static_cast<int32_t>(block.error);
    }

    void appendHistory(const acquisition::Block &block) {
        if (block.error != ESP_OK || block.count == 0) {
            historyCount_ = 0;
            return;
        }

        size_t source = 0;
        size_t incoming = block.count;
        if (incoming > adc_config::capacity) {
            source = incoming - adc_config::capacity;
            incoming = adc_config::capacity;
        }

        if (historyCount_ + incoming > adc_config::capacity) {
            const size_t remove = historyCount_ + incoming - adc_config::capacity;
            if (remove < historyCount_) {
                memmove(
                    history_,
                    history_ + remove,
                    (historyCount_ - remove) * sizeof(history_[0])
                );
                historyCount_ -= remove;
            } else {
                historyCount_ = 0;
            }
        }

        memcpy(
            history_ + historyCount_,
            block.samples + source,
            incoming * sizeof(history_[0])
        );
        historyCount_ += incoming;

        const uint32_t newestUs = history_[historyCount_ - 1].us;
        size_t first = 0;
        while (
            first + 1 < historyCount_ &&
            uint32_t(newestUs - history_[first].us) > adc_config::captureUs
        ) {
            ++first;
        }
        if (first > 0) {
            memmove(
                history_,
                history_ + first,
                (historyCount_ - first) * sizeof(history_[0])
            );
            historyCount_ -= first;
        }
    }

    void prepareValidFrame(
        const acquisition::Block &block,
        const waveform::Sample *samples,
        size_t sampleCount,
        const waveform::Result &result,
        measurement::DisplayFrame &frame
    ) {
        const bool calibrated = adc_config::mainsVoltsPerAdcVolt > 0.0f;
        const float gain = calibrated
            ? adc_config::mainsVoltsPerAdcVolt
            : adc_config::previewGain;
        const uint32_t nowMs = block.completedMs;
        const bool resetPresentation =
            !presentationReady_ ||
            uint32_t(nowMs - lastPresentationMs_) > kPresentationResetMs;
        const float dtMs = presentationReady_
            ? float(uint32_t(nowMs - lastPresentationMs_))
            : 0.0f;
        const float curveAlpha = resetPresentation
            ? 1.0f
            : 1.0f - expf(-dtMs / kCurveTauMs);
        const float valuesAlpha = resetPresentation
            ? 1.0f
            : 1.0f - expf(-dtMs / kValuesTauMs);

        const float measuredRms = result.rms * gain;
        const float measuredPeak = result.peak * gain;
        if (resetPresentation) {
            smoothRms_ = measuredRms;
            smoothPeak_ = measuredPeak;
            smoothFrequency_ = result.frequency;
        } else {
            smoothRms_ += valuesAlpha * (measuredRms - smoothRms_);
            smoothPeak_ += valuesAlpha * (measuredPeak - smoothPeak_);
            smoothFrequency_ += valuesAlpha * (result.frequency - smoothFrequency_);
        }

        const uint32_t frameStart = micros();
        size_t index = 0;
        const bool outsideAxis =
            result.peak * gain > sine_view::kAxisLimitVolts;
        const float displayWindowUs = result.displayEndUs - result.triggerUs;

        float coefficientCosine[kHarmonicCount]{};
        float coefficientSine[kHarmonicCount]{};
        float phaseCosine[kHarmonicCount] = {1.0f, 1.0f, 1.0f};
        float phaseSine[kHarmonicCount]{};

        // Dopasowanie DFT do 192 rownomiernych punktow usuwa wplyw
        // niejednorodnego czasu I2C. Zachowujemy skladowe 1, 3 i 5, ktore
        // opisuja podstawowy ksztalt bez wysokoczestotliwosciowych zalaman.
        for (size_t i = 0; i < kHarmonicFitPointCount; ++i) {
            const float t = result.triggerUs +
                float(i) * displayWindowUs / kHarmonicFitPointCount;
            const float value =
                (waveform::at(samples, sampleCount, t, index) -
                 result.offset) * gain;

            for (size_t harmonic = 0; harmonic < kHarmonicCount; ++harmonic) {
                coefficientCosine[harmonic] +=
                    value * phaseCosine[harmonic];
                coefficientSine[harmonic] +=
                    value * phaseSine[harmonic];
                rotatePhase(
                    phaseCosine[harmonic],
                    phaseSine[harmonic],
                    kFitRotation[harmonic]
                );
            }
        }

        constexpr float coefficientScale =
            2.0f / float(kHarmonicFitPointCount);
        for (size_t harmonic = 0; harmonic < kHarmonicCount; ++harmonic) {
            coefficientCosine[harmonic] *= coefficientScale;
            coefficientSine[harmonic] *= coefficientScale;
        }

        // Zablokuj faze na rosnacym przejsciu przez zero pierwszej
        // harmonicznej. Dla A*cos(theta)+B*sin(theta) poczatek fazy ma
        // cos(theta0)=B/R i sin(theta0)=-A/R. Potegi zespolone przenosza
        // identyczna korekte czasu na harmoniczne 3 i 5 bez trygonometrii.
        const float fundamentalCosine = coefficientCosine[0];
        const float fundamentalSine = coefficientSine[0];
        const float fundamentalSquared =
            fundamentalCosine * fundamentalCosine +
            fundamentalSine * fundamentalSine;
        if (fundamentalSquared > 1.0e-12f) {
            const float inverseMagnitude = 1.0f / sqrtf(fundamentalSquared);
            const float baseCosine = fundamentalSine * inverseMagnitude;
            const float baseSine = -fundamentalCosine * inverseMagnitude;
            const float secondCosine =
                baseCosine * baseCosine - baseSine * baseSine;
            const float secondSine = 2.0f * baseCosine * baseSine;

            phaseCosine[0] = baseCosine;
            phaseSine[0] = baseSine;
            phaseCosine[1] =
                secondCosine * baseCosine - secondSine * baseSine;
            phaseSine[1] =
                secondSine * baseCosine + secondCosine * baseSine;
            phaseCosine[2] =
                phaseCosine[1] * secondCosine -
                phaseSine[1] * secondSine;
            phaseSine[2] =
                phaseSine[1] * secondCosine +
                phaseCosine[1] * secondSine;
        } else {
            for (size_t harmonic = 0; harmonic < kHarmonicCount; ++harmonic) {
                phaseCosine[harmonic] = 1.0f;
                phaseSine[harmonic] = 0.0f;
            }
        }

        // Rekonstrukcja harmoniczna jest gladka i zakotwiczona fazowo.
        // Rekurencyjne obroty zastepuja 2040 wywolan trygonometrycznych.
        constexpr size_t lastPoint = measurement::kDisplayPointCount - 1;
        for (size_t i = 0; i < lastPoint; ++i) {
            float measuredVolts = 0.0f;
            for (size_t harmonic = 0; harmonic < kHarmonicCount; ++harmonic) {
                measuredVolts +=
                    coefficientCosine[harmonic] * phaseCosine[harmonic] +
                    coefficientSine[harmonic] * phaseSine[harmonic];
                rotatePhase(
                    phaseCosine[harmonic],
                    phaseSine[harmonic],
                    kPixelRotation[harmonic]
                );
            }

            if (resetPresentation) {
                displayVolts_[i] = measuredVolts;
            } else {
                displayVolts_[i] +=
                    curveAlpha * (measuredVolts - displayVolts_[i]);
            }
            const float clipped = fmaxf(
                -sine_view::kAxisLimitVolts,
                fminf(sine_view::kAxisLimitVolts, displayVolts_[i])
            );
            frame.y[i] = static_cast<int16_t>(sine_view::voltageToY(clipped));
        }

        // Punkt 681 zamyka dokladnie te sama faze co punkt pierwszy.
        displayVolts_[lastPoint] = displayVolts_[0];
        frame.y[lastPoint] = frame.y[0];
        const float clippedRms = fmaxf(
            -sine_view::kAxisLimitVolts,
            fminf(sine_view::kAxisLimitVolts, smoothRms_)
        );
        frame.rms_y = static_cast<int16_t>(sine_view::voltageToY(clippedRms));
        frame.processing.frame_prepare_us = uint32_t(micros() - frameStart);

        presentationReady_ = true;
        lastPresentationMs_ = nowMs;

        frame.measurement.rms = result.rms;
        frame.measurement.peak = result.peak;
        frame.measurement.frequency = result.frequency;
        frame.measurement.offset = result.offset;
        frame.measurement.minimum = result.minimum;
        frame.measurement.maximum = result.maximum;
        frame.measurement.sample_rate = result.sampleRate;
        if (result.nearRail) frame.measurement.quality_flags |= measurement::kNearAdcRail;
        if (outsideAxis) frame.measurement.quality_flags |= measurement::kOutsideDisplayAxis;

        if (calibrated) {
            snprintf(
                frame.rms_value_text,
                sizeof(frame.rms_value_text),
                "%.1f",
                smoothRms_
            );
            snprintf(
                frame.maximum_value_text,
                sizeof(frame.maximum_value_text),
                "%.1f",
                smoothPeak_
            );
            snprintf(
                frame.frequency_value_text,
                sizeof(frame.frequency_value_text),
                "%.1f",
                smoothFrequency_
            );
            snprintf(
                frame.status_text,
                sizeof(frame.status_text),
                "POMIAR | A0 | %.0f probek/s",
                result.sampleRate
            );
        } else {
            snprintf(
                frame.rms_value_text,
                sizeof(frame.rms_value_text),
                "--"
            );
            snprintf(
                frame.maximum_value_text,
                sizeof(frame.maximum_value_text),
                "--"
            );
            snprintf(
                frame.frequency_value_text,
                sizeof(frame.frequency_value_text),
                "%.1f",
                smoothFrequency_
            );
            snprintf(
                frame.status_text,
                sizeof(frame.status_text),
                "BEZ KALIBRACJI | podglad x%.0f | ADC RMS=%.3f V",
                gain, result.rms
            );
        }

        if (result.nearRail) {
            snprintf(
                frame.status_text,
                sizeof(frame.status_text),
                "UWAGA: OUT blisko 0/5 V | sprawdz wzmocnienie czujnika"
            );
        } else if (outsideAxis) {
            snprintf(
                frame.status_text,
                sizeof(frame.status_text),
                "%s | przebieg poza osia +/-400; obraz przyciety",
                calibrated ? "POMIAR" : "BEZ KALIBRACJI"
            );
        }
    }

    void process(
        acquisition::Block &block,
        measurement::DisplayFrame &frame
    ) {
        memset(&frame, 0, sizeof(frame));
        const uint32_t processingStart = micros();
        frame.sequence = block.sequence;
        frame.completed_ms = block.completedMs;
        frame.measurement.sequence = block.sequence;
        frame.measurement.timestamp_us = block.completedUs;
        frame.measurement.dropped_blocks = block.droppedBefore;

        waveform::GapInfo historyGaps{};
        const size_t currentBlockSamples = block.count < historyCount_
            ? block.count
            : historyCount_;
        const size_t currentBlockStart = historyCount_ - currentBlockSamples;
        const uint32_t boundaryGapUs = currentBlockStart > 0
            ? uint32_t(
                history_[currentBlockStart].us -
                history_[currentBlockStart - 1].us
            )
            : 0;
        uint32_t quality = measurement::kQualityNone;
        waveform::Result result{};
        if (block.error != ESP_OK) {
            quality |= measurement::kI2cError;
        } else if (uint32_t(millis() - block.completedMs) > 500) {
            quality |= measurement::kStaleBlock;
        } else if (
            block.diagnostic.gapMaxUs > acquisition::kMaxGapUs ||
            block.diagnostic.readMaxUs > acquisition::kMaxGapUs
        ) {
            quality |= measurement::kSampleGap;
        } else {
            quality |= qualityFor(waveform::analyzeDetailed(
                history_,
                historyCount_,
                adc_config::triggerHysteresisV,
                sine_view::kDisplayedPeriods,
                result,
                &historyGaps
            ));
        }

        if (historyGaps.count == 0 && historyCount_ > 1) {
            historyGaps = waveform::inspectGaps(history_, historyCount_);
        }
        copyAcquisition(
            block,
            historyCount_,
            historyGaps,
            boundaryGapUs,
            frame.acquisition
        );

        frame.measurement.quality_flags = quality;
        frame.valid = quality == measurement::kQualityNone;
        if (frame.valid) {
            prepareValidFrame(block, history_, historyCount_, result, frame);
        } else {
            snprintf(
                frame.status_text,
                sizeof(frame.status_text),
                "%s",
                holdText(quality)
            );
        }

        frame.processing.processing_us = uint32_t(micros() - processingStart);
        frame.processing.stack_free_bytes = uxTaskGetStackHighWaterMark(nullptr);
        frame.processing.dropped_frames = droppedFrames_;
        frame.processing.skipped_frames = skippedFrames_;
    }

    bool shouldPublish(const measurement::DisplayFrame &frame) const {
        return !frame.valid ||
            lastPublishedMs_ == 0 ||
            uint32_t(frame.completed_ms - lastPublishedMs_) >= kGuiPeriodMs;
    }

    void publish(measurement::DisplayFrame *frame) {
        if (readyHighWater_ < 1) readyHighWater_ = 1;
        frame->processing.ready_high_water = readyHighWater_;

        if (xQueueSend(ready_, &frame, 0) != pdTRUE) {
            measurement::DisplayFrame *obsolete = nullptr;
            if (xQueueReceive(ready_, &obsolete, 0) == pdTRUE) {
                ++droppedFrames_;
                release(obsolete);
            }
            frame->processing.dropped_frames = droppedFrames_;
            if (xQueueSend(ready_, &frame, 0) != pdTRUE) {
                ++droppedFrames_;
                release(frame);
            }
        }
    }

    void run() {
        while (true) {
            acquisition::Block *block = source_.take(portMAX_DELAY);
            if (block == nullptr) continue;

            appendHistory(*block);

            measurement::DisplayFrame *frame = acquireWritableFrame();
            if (frame == nullptr) {
                ++droppedFrames_;
                source_.release(block);
                continue;
            }

            process(*block, *frame);
            source_.release(block);
            if (shouldPublish(*frame)) {
                lastPublishedMs_ = frame->completed_ms;
                publish(frame);
            } else {
                ++skippedFrames_;
                release(frame);
            }
        }
    }
};

} // namespace signal_processing
