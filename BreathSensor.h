#pragma once
#include <Arduino.h>

/*
 * BreathSensor — Algoritma Hybrid v27 Clean
 *
 * Filter IIR + adaptive baseline + period-based mode (FAST/SLOW).
 * Interface kompatibel dengan main.cpp lama:
 *   - getCurrentPhase() → +1 hirup, -1 buang (kontinu — bukan event sekali)
 *     dipakai writeLogEntry per langkah supaya log dapat pola 1,1,1,-1,-1
 */

#define BREATH_SENSOR_PIN       34
#define BREATH_SAMPLE_RATE_HZ   50
#define BREATH_SAMPLING_PERIOD  (1000 / BREATH_SAMPLE_RATE_HZ)

namespace BreathConfig {
    const float FILTER_B[] = {0.007821f, 0.015642f, 0.007821f};
    const float FILTER_A[] = {1.0f, -1.73471f, 0.76599f};

    const float DETREND_ALPHA_FAST = 0.98f;
    const float DETREND_ALPHA_HOLD = 0.999f;

    const float PERIOD_FAST_SLOW_THRESHOLD = 1000.0f;
    const float PERIOD_ALPHA_START         = 600.0f;
    const float PERIOD_ALPHA_END           = 1400.0f;

    const float MIN_DELTA_FAST    = 30.0f;
    const float DEAD_ZONE_FAST    = 30.0f;
    const float RATIO_EXHALE_FAST = 0.08f;
    const float RATIO_INHALE_FAST = 0.15f;

    const float MIN_DELTA_SLOW    = 80.0f;
    const float DEAD_ZONE_SLOW    = 80.0f;
    const float RATIO_EXHALE_SLOW = 0.25f;
    const float RATIO_INHALE_SLOW = 0.20f;

    const float MAX_DELTA         = 400.0f;
    const float MIN_BREATH_SIZE   = 80.0f;
    const int   MIN_BREATH_DURATION = 150;
    const int   HOLD_LOCKDOWN_MS    = 3000;
}

struct IIR_State {
    float x[3] = {0, 0, 0};
    float y[3] = {0, 0, 0};
};

class BreathSensor {
public:
    BreathSensor();

    void begin();
    void update();

    // Dipertahankan untuk kompatibilitas main.cpp — tidak dipakai sensor asli
    void triggerGuide(int8_t direction, float phase_duration_ms);

    bool     isInhaleDetected();
    bool     isExhaleDetected();
    uint32_t getLastEventTimestamp();
    int8_t   getLastEventType();
    int8_t   getCurrentPhase();   // +1 hirup, -1 buang, 0 belum diketahui

private:
    unsigned long _last_sample_ms;

    IIR_State _iir;
    float _applyIIR(float input);

    float _baseline;
    bool  _baseline_initialized;
    float _smoothed_period;
    unsigned long _last_event_ms;

    bool  _is_inhaling;
    float _local_max;
    float _local_min;
    unsigned long _time_at_max;
    unsigned long _time_at_min;
    float _last_amplitude;
    float _last_peak_level;
    float _last_valley_level;

    bool     _inhale_ready;
    bool     _exhale_ready;
    uint32_t _last_event_ts;
    int8_t   _last_event_type;
    int8_t   _current_phase;
};
