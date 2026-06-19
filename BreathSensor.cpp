#include "BreathSensor.h"

/*
 * BreathSensor.cpp — Hybrid v27 Clean
 * Algoritma: Fast Detection + Anti-Drift Adaptive Baseline (decoupled)
 *
 * _current_phase di-update saat event terdeteksi dan tetap bertahan
 * sampai event berikutnya → main.cpp dapat pola continuous 1,1,1,-1,-1.
 */

BreathSensor::BreathSensor()
    : _last_sample_ms(0),
      _baseline(0.0f),
      _baseline_initialized(false),
      _smoothed_period(800.0f),
      _last_event_ms(0),
      _is_inhaling(false),
      _local_max(-99999.0f),
      _local_min(99999.0f),
      _time_at_max(0),
      _time_at_min(0),
      _last_amplitude(400.0f),
      _last_peak_level(200.0f),
      _last_valley_level(-200.0f),
      _inhale_ready(false),
      _exhale_ready(false),
      _last_event_ts(0),
      _last_event_type(0),
      _current_phase(0)
{}

void BreathSensor::begin() {
    Serial.println("[BreathSensor] Mode: SENSOR ASLI (Hybrid v27 Clean)");
    Serial.printf("[BreathSensor] Pin ADC: GPIO%d, Sample rate: %d Hz\n",
                  BREATH_SENSOR_PIN, BREATH_SAMPLE_RATE_HZ);

    float sum = 0;
    for (int i = 0; i < 50; i++) {
        sum += (float)analogRead(BREATH_SENSOR_PIN);
        delay(20);
    }
    _baseline             = sum / 50.0f;
    _baseline_initialized = true;
    _last_event_ms        = millis();

    for (int i = 0; i < 3; i++) {
        _iir.x[i] = _baseline;
        _iir.y[i] = _baseline;
    }

    _last_peak_level   = 200.0f;
    _last_valley_level = -200.0f;

    Serial.printf("[BreathSensor] Baseline awal: %.1f\n", _baseline);
}

float BreathSensor::_applyIIR(float input) {
    _iir.x[2] = _iir.x[1];
    _iir.x[1] = _iir.x[0];
    _iir.x[0] = input;

    float output =
        BreathConfig::FILTER_B[0] * _iir.x[0] +
        BreathConfig::FILTER_B[1] * _iir.x[1] +
        BreathConfig::FILTER_B[2] * _iir.x[2]
        - BreathConfig::FILTER_A[1] * _iir.y[1]
        - BreathConfig::FILTER_A[2] * _iir.y[2];

    _iir.y[2] = _iir.y[1];
    _iir.y[1] = output;
    _iir.y[0] = output;

    return output;
}

void BreathSensor::update() {
    if (!_baseline_initialized) return;

    unsigned long now = millis();
    if (now - _last_sample_ms < BREATH_SAMPLING_PERIOD) return;
    _last_sample_ms = now;

    // 1. Baca ADC
    float raw = (float)analogRead(BREATH_SENSOR_PIN);

    // 2. Filter IIR
    float filtered = _applyIIR(raw);

    // 3. Estimasi periode napas (smoothed)
    float raw_period = (float)(now - _last_event_ms);
    _smoothed_period = 0.8f * _smoothed_period + 0.2f * raw_period;

    // 4. Alpha adaptif (anti-drift)
    float t = (_smoothed_period - BreathConfig::PERIOD_ALPHA_START) /
              (BreathConfig::PERIOD_ALPHA_END - BreathConfig::PERIOD_ALPHA_START);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float current_alpha = (1.0f - t) * BreathConfig::DETREND_ALPHA_FAST
                        + t           * BreathConfig::DETREND_ALPHA_HOLD;

    // 5. Update baseline
    _baseline = current_alpha * _baseline + (1.0f - current_alpha) * filtered;

    // 6. Sinyal AC
    float ac_signal = filtered - _baseline;

    // 7. Hold detection
    bool is_holding = (raw_period > (float)BreathConfig::HOLD_LOCKDOWN_MS);

    // 8. Pilih mode deteksi (decoupled dari baseline!)
    float active_min_delta, active_dead_zone;
    float active_ratio_exhale, active_ratio_inhale;

    if (_smoothed_period > BreathConfig::PERIOD_FAST_SLOW_THRESHOLD) {
        active_min_delta    = BreathConfig::MIN_DELTA_SLOW;
        active_dead_zone    = BreathConfig::DEAD_ZONE_SLOW;
        active_ratio_exhale = BreathConfig::RATIO_EXHALE_SLOW;
        active_ratio_inhale = BreathConfig::RATIO_INHALE_SLOW;
    } else {
        active_min_delta    = BreathConfig::MIN_DELTA_FAST;
        active_dead_zone    = BreathConfig::DEAD_ZONE_FAST;
        active_ratio_exhale = BreathConfig::RATIO_EXHALE_FAST;
        active_ratio_inhale = BreathConfig::RATIO_INHALE_FAST;
    }

    // 9. Deteksi ekstremum
    if (_is_inhaling) {
        if (ac_signal > _local_max) {
            _local_max   = ac_signal;
            _time_at_max = now;
        }

        float delta = _last_amplitude * active_ratio_exhale;
        if (delta < active_min_delta) delta = active_min_delta;
        if (is_holding) delta = BreathConfig::MAX_DELTA;

        if (ac_signal < (_local_max - delta)) {
            if (_local_max > active_dead_zone) {
                if ((long)(_time_at_max - _last_event_ms) > BreathConfig::MIN_BREATH_DURATION) {
                    float height = _local_max - _last_valley_level;
                    if (height > BreathConfig::MIN_BREATH_SIZE) {
                        _is_inhaling       = false;
                        _last_amplitude    = height;
                        _last_peak_level   = _local_max;
                        _last_event_ms     = _time_at_max;
                        _local_min         = ac_signal;
                        _time_at_min       = now;

                        _last_event_ts     = _time_at_max;
                        _last_event_type   = 1;
                        _exhale_ready      = true;
                        _current_phase     = -1;

                        Serial.printf("[BREATH] EXHALE detected, AC_peak=%.1f\n", _local_max);
                    }
                }
            }
        }
    } else {
        if (ac_signal < _local_min) {
            _local_min   = ac_signal;
            _time_at_min = now;
        }

        float delta = _last_amplitude * active_ratio_inhale;
        if (delta < active_min_delta) delta = active_min_delta;
        if (is_holding) delta = BreathConfig::MAX_DELTA;

        if (ac_signal > (_local_min + delta)) {
            if (_local_min < -active_dead_zone) {
                if ((long)(_time_at_min - _last_event_ms) > BreathConfig::MIN_BREATH_DURATION) {
                    float depth = _last_peak_level - _local_min;
                    if (depth > BreathConfig::MIN_BREATH_SIZE) {
                        _is_inhaling       = true;
                        _last_amplitude    = depth;
                        _last_valley_level = _local_min;
                        _last_event_ms     = _time_at_min;
                        _local_max         = ac_signal;
                        _time_at_max       = now;

                        _last_event_ts     = _time_at_min;
                        _last_event_type   = -1;
                        _inhale_ready      = true;
                        _current_phase     = 1;

                        Serial.printf("[BREATH] INHALE detected, AC_valley=%.1f\n", _local_min);
                    }
                }
            }
        }
    }
}

void BreathSensor::triggerGuide(int8_t direction, float phase_duration_ms) {
    (void)direction;
    (void)phase_duration_ms;
}

bool BreathSensor::isInhaleDetected() {
    return _inhale_ready;
}

bool BreathSensor::isExhaleDetected() {
    return _exhale_ready;
}

uint32_t BreathSensor::getLastEventTimestamp() {
    _inhale_ready = false;
    _exhale_ready = false;
    return _last_event_ts;
}

int8_t BreathSensor::getLastEventType() {
    return _last_event_type;
}

int8_t BreathSensor::getCurrentPhase() {
    return _current_phase;
}
