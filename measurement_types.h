#pragma once

#include <stddef.h>
#include <stdint.h>

namespace measurement {

// Jeden punkt na kazda kolumne wykresu (680 px) oraz punkt koncowy.
constexpr size_t kDisplayPointCount = 681;

enum QualityFlag : uint32_t {
    kQualityNone = 0,
    kI2cError = 1UL << 0,
    kStaleBlock = 1UL << 1,
    kSampleGap = 1UL << 2,
    kNoStableTrigger = 1UL << 3,
    kPeriodOutOfRange = 1UL << 4,
    kPeriodJitter = 1UL << 5,
    kNearAdcRail = 1UL << 6,
    kOutsideDisplayAxis = 1UL << 7,
    kStorageDropped = 1UL << 8,
    kProcessingDropped = 1UL << 9,
};

struct MeasurementRecord {
    uint32_t sequence = 0;
    uint64_t timestamp_us = 0;
    float rms = 0;
    float frequency = 0;
    float offset = 0;
    float minimum = 0;
    float maximum = 0;
    float sample_rate = 0;
    uint32_t dropped_blocks = 0;
    uint32_t quality_flags = kQualityNone;
};

struct AcquisitionMetrics {
    uint32_t sample_count = 0;
    uint32_t block_sample_count = 0;
    uint32_t gap_mean_us = 0;
    uint32_t gap_max_us = 0;
    uint32_t gap_index = 0;
    uint32_t history_gap_mean_us = 0;
    uint32_t history_gap_max_us = 0;
    uint32_t history_gap_index = 0;
    uint32_t boundary_gap_us = 0;
    bool history_zero_gap = false;
    uint32_t between_reads_max_us = 0;
    uint32_t read_max_us = 0;
    uint32_t write_max_us = 0;
    uint32_t poll_max_us = 0;
    uint32_t result_max_us = 0;
    uint32_t wait_max_us = 0;
    uint32_t block_us = 0;
    uint32_t dropped_blocks = 0;
    uint32_t ready_high_water = 0;
    uint32_t stack_free_bytes = 0;
    uint32_t error_count = 0;
    uint32_t missed_conversions = 0;
    int32_t error = 0;
};

struct ProcessingMetrics {
    uint32_t processing_us = 0;
    uint32_t frame_prepare_us = 0;
    uint32_t stack_free_bytes = 0;
    uint32_t dropped_frames = 0;
    uint32_t skipped_frames = 0;
    uint32_t ready_high_water = 0;
};

struct DisplayFrame {
    uint32_t sequence = 0;
    uint32_t completed_ms = 0;
    bool valid = false;
    int16_t y[kDisplayPointCount]{};
    MeasurementRecord measurement;
    AcquisitionMetrics acquisition;
    ProcessingMetrics processing;
    char rms_value_text[16]{};
    char frequency_value_text[16]{};
    char status_text[120]{};
};

} // namespace measurement
