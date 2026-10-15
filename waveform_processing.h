#pragma once
#include <cmath>
#include <stddef.h>
#include <stdint.h>

namespace waveform {
struct Sample { float volts; uint32_t us; };
enum class AnalysisStatus : uint8_t {
    Ok,
    InvalidArguments,
    SampleGap,
    NoStableTrigger,
    PeriodOutOfRange,
    PeriodJitter,
};
struct Result {
    float offset = 0, rms = 0, peak = 0, frequency = 0, sampleRate = 0;
    float minimum = 0, maximum = 0;
    float triggerUs = 0;
    float displayEndUs = 0;
    bool nearRail = false;
};
struct GapInfo {
    uint32_t maximumUs = 0;
    uint32_t maximumIndex = 0;
    uint64_t totalUs = 0;
    uint32_t count = 0;
    bool hasZero = false;
};
inline GapInfo inspectGaps(const Sample *s, size_t n) {
    GapInfo info{};
    if (s == nullptr || n < 2) return info;

    for (size_t i = 1; i < n; ++i) {
        const uint32_t dt = uint32_t(s[i].us - s[i - 1].us);
        if (dt == 0) info.hasZero = true;
        if (dt > info.maximumUs) {
            info.maximumUs = dt;
            info.maximumIndex = static_cast<uint32_t>(i);
        }
        info.totalUs += dt;
        ++info.count;
    }
    return info;
}
inline float timeAt(const Sample *s, size_t i) {
    return static_cast<float>(uint32_t(s[i].us - s[0].us));
}
inline float at(const Sample *s, size_t n, float t, size_t &i) {
    while (i + 1 < n && timeAt(s, i + 1) < t) ++i;
    if (i + 1 == n) return s[i].volts;
    const float t0 = timeAt(s, i);
    const float dt = timeAt(s, i + 1) - t0;
    if (dt <= 0) return s[i].volts;
    const float f = fmaxf(0.0f, fminf(1.0f, (t - t0) / dt));
    return s[i].volts + f * (s[i + 1].volts - s[i].volts);
}
// Integrals of piecewise-linear data. Accounts for nonuniform I2C timing.
inline void moments(const Sample *s, size_t n, float start, float end,
                    float &mean, float &rms, float offset) {
    float sum = 0.0f, squares = 0.0f;
    for (size_t i = 1; i < n; ++i) {
        const float t0 = timeAt(s, i - 1);
        const float t1 = timeAt(s, i);
        const float a = fmaxf(start, t0);
        const float b = fminf(end, t1);
        if (b <= a) continue;
        const float dt = t1 - t0;
        if (dt <= 0.0f) continue;
        const float v0 = s[i - 1].volts - offset;
        const float slope = (s[i].volts - s[i - 1].volts) / dt;
        const float va = v0 + slope * (a - t0);
        const float vb = v0 + slope * (b - t0);
        const float span = b - a;
        sum += span * (va + vb) * 0.5f;
        squares += span * (va * va + va * vb + vb * vb) / 3.0f;
    }
    const float duration = end - start;
    mean = sum / duration;
    rms = sqrtf(fmaxf(0.0f, squares / duration));
}
inline size_t crossings(const Sample *s, size_t n, float offset, float h,
                        float *out, size_t capacity) {
    bool armed = false, pending = false;
    float crossing = 0.0f;
    size_t count = 0;
    for (size_t i=1; i<n; ++i) {
        if (s[i-1].volts <= offset-h) { armed=true; pending=false; }
        if (!armed) continue;
        if (!pending && s[i-1].volts < offset && s[i].volts >= offset) {
            crossing = timeAt(s,i-1) + (offset-s[i-1].volts) /
              (s[i].volts-s[i-1].volts) * (timeAt(s,i)-timeAt(s,i-1));
            pending=true;
        }
        if (pending && s[i].volts >= offset+h) {
            if (count < capacity) out[count++] = crossing;
            armed=false; pending=false;
        }
    }
    return count;
}
inline AnalysisStatus analyzeDetailed(
    const Sample *s,
    size_t n,
    float hysteresis,
    size_t displayPeriods,
    Result &r,
    GapInfo *gapInfo = nullptr
) {
    if (n < 4 || displayPeriods == 0 || displayPeriods >= 32) {
        return AnalysisStatus::InvalidArguments;
    }
    const GapInfo gaps = inspectGaps(s, n);
    if (gapInfo != nullptr) *gapInfo = gaps;
    if (gaps.hasZero || gaps.maximumUs > 5000)
        return AnalysisStatus::SampleGap;
    float mean, rms;
    moments(s,n,0.0f,timeAt(s,n-1),mean,rms,0.0f);
    float edges[32];
    size_t count = 0;
    // Refine the DC offset over whole periods, not arbitrary partial cycles.
    for (int pass=0;pass<2;++pass) {
        count=crossings(s,n,mean,hysteresis,edges,32);
        if(count<3) return AnalysisStatus::NoStableTrigger;
        moments(s,n,edges[0],edges[count-1],mean,rms,0);
    }
    count=crossings(s,n,mean,hysteresis,edges,32);
    // N periods require N+1 accepted rising crossings.
    if(count <= displayPeriods) return AnalysisStatus::NoStableTrigger;
    const float period=(edges[count-1]-edges[0])/(count-1);
    // Application is for mains-frequency waveforms, not arbitrary pulses.
    if(period<12500 || period>25000) return AnalysisStatus::PeriodOutOfRange;
    for(size_t i=1;i<count;++i)
        if(fabsf((edges[i]-edges[i-1])-period)>0.15f*period)
            return AnalysisStatus::PeriodJitter;
    r.offset=mean;
    moments(s,n,edges[0],edges[count-1],mean,rms,r.offset);
    r.rms=rms;
    r.frequency=1000000.0f/period;
    r.triggerUs=edges[0];
    r.displayEndUs=edges[displayPeriods];
    r.sampleRate=float(n-1)*1000000.0f/timeAt(s,n-1);
    r.minimum=r.maximum=s[0].volts;
    r.peak=0;
    for(size_t i=0;i<n;++i) {
        r.minimum=fminf(r.minimum,s[i].volts);
        r.maximum=fmaxf(r.maximum,s[i].volts);
        if(timeAt(s,i)>=edges[0] && timeAt(s,i)<=edges[count-1])
            r.peak=fmaxf(r.peak,fabsf(s[i].volts-r.offset));
    }
    r.nearRail=r.minimum<0.05f || r.maximum>4.95f;
    return AnalysisStatus::Ok;
}

inline bool analyze(const Sample *s, size_t n, float hysteresis,
                    size_t displayPeriods, Result &r) {
    return analyzeDetailed(s, n, hysteresis, displayPeriods, r) ==
        AnalysisStatus::Ok;
}
}
