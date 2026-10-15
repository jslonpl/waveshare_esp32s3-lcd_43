#pragma once

#include <stddef.h>
#include <stdint.h>

#include "measurement_types.h"

namespace measurement {

// Bounded in-RAM history. Persistent storage is deliberately outside this
// pipeline so it cannot delay acquisition, processing, or LVGL.
class Store {
public:
    static constexpr size_t kCapacity = 60;

    bool push(const MeasurementRecord &record) {
        const bool overwroteOldest = count_ == kCapacity;
        records_[write_] = record;
        write_ = (write_ + 1) % kCapacity;

        if (overwroteOldest) {
            ++dropped_;
        } else {
            ++count_;
        }

        return overwroteOldest;
    }

    uint32_t dropped() const { return dropped_; }
    size_t size() const { return count_; }

private:
    MeasurementRecord records_[kCapacity]{};
    size_t write_ = 0;
    size_t count_ = 0;
    uint32_t dropped_ = 0;
};

} // namespace measurement
