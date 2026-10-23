#pragma once

#include <Arduino.h>
#include <cmath>
#include <stdint.h>

#include "waveform_processing.h"

namespace waveform_self_test {

constexpr float kPi = 3.14159265358979323846f;
constexpr size_t kCapacity = 256;

inline size_t makeSine(
    waveform::Sample *samples,
    float frequency,
    float offset,
    float amplitude,
    uint32_t startUs,
    bool nonuniform
) {
    uint32_t elapsed = 0;
    size_t count = 0;
    while (count < kCapacity && elapsed <= 120000) {
        samples[count].us = startUs + elapsed;
        samples[count].volts = offset + amplitude * sinf(
            2.0f * kPi * frequency * float(elapsed) / 1000000.0f
        );
        elapsed += nonuniform ? ((count & 1U) ? 550 : 450) : 500;
        ++count;
    }
    return count;
}

inline bool closeRelative(float actual, float expected, float fraction) {
    return fabsf(actual - expected) <= fabsf(expected) * fraction;
}

inline bool verifySine(
    waveform::Sample *samples,
    float frequency,
    float offset,
    float amplitude,
    uint32_t startUs,
    bool nonuniform
) {
    const size_t count = makeSine(
        samples, frequency, offset, amplitude, startUs, nonuniform
    );
    waveform::Result result{};
    const auto status = waveform::analyzeDetailed(
        samples, count, 0.03f, 3, result
    );
    return status == waveform::AnalysisStatus::Ok &&
        fabsf(result.frequency - frequency) <= 0.1f &&
        closeRelative(result.rms, amplitude / sqrtf(2.0f), 0.005f) &&
        fabsf(result.offset - offset) <= 0.002f;
}

inline size_t makeJittered(waveform::Sample *samples) {
    constexpr uint32_t periods[] = {20000, 24000, 16000, 20000, 20000, 20000};
    constexpr size_t pointsPerPeriod = 16;
    size_t count = 0;
    uint32_t elapsed = 0;
    for (uint32_t period : periods) {
        for (size_t i = 0; i < pointsPerPeriod; ++i) {
            samples[count].us = elapsed +
                uint32_t(uint64_t(period) * i / pointsPerPeriod);
            samples[count].volts = 2.5f + sinf(
                2.0f * kPi * float(i) / pointsPerPeriod
            );
            ++count;
        }
        elapsed += period;
    }
    samples[count].us = elapsed;
    samples[count].volts = 2.5f;
    return count + 1;
}

inline bool run(Print &output) {
    static waveform::Sample samples[kCapacity];
    bool ok = true;

    ok = ok && verifySine(samples, 50.0f, 2.5f, 1.0f, 1000, false);
    ok = ok && verifySine(samples, 60.0f, 2.2f, 0.8f, 2000, true);
    ok = ok && verifySine(samples, 50.0f, 2.5f, 1.0f, 0xFFFF0000UL, true);

    size_t count = makeSine(samples, 50.0f, 2.5f, 2.48f, 0, false);
    waveform::Result result{};
    ok = ok && waveform::analyzeDetailed(samples, count, 0.03f, 3, result) ==
        waveform::AnalysisStatus::Ok && result.nearRail;

    count = makeSine(samples, 50.0f, 2.5f, 1.0f, 0, false);
    for (size_t i = count / 2; i < count; ++i) samples[i].us += 6000;
    const size_t gapIndex = count / 2;
    const waveform::GapInfo gapInfo = waveform::inspectGaps(samples, count);
    ok = ok && !gapInfo.hasZero &&
        gapInfo.maximumUs == 6500 &&
        gapInfo.maximumIndex == gapIndex;
    ok = ok && waveform::analyzeDetailed(samples, count, 0.03f, 3, result) ==
        waveform::AnalysisStatus::SampleGap;

    samples[gapIndex].us = samples[gapIndex - 1].us;
    ok = ok && waveform::inspectGaps(samples, count).hasZero;

    count = makeJittered(samples);
    ok = ok && waveform::analyzeDetailed(samples, count, 0.03f, 3, result) ==
        waveform::AnalysisStatus::PeriodJitter;

    for (size_t i = 0; i < 20; ++i) {
        samples[i].us = uint32_t(i * 1000);
        samples[i].volts = 2.5f;
    }
    ok = ok && waveform::analyzeDetailed(samples, 20, 0.03f, 3, result) ==
        waveform::AnalysisStatus::NoStableTrigger;

    output.println(ok
        ? "WAVEFORM_SELF_TEST PASS"
        : "WAVEFORM_SELF_TEST FAIL");
    return ok;
}

} // namespace waveform_self_test
