#include "BreathSensorV2.h"

/*
 * BreathSensorV2.cpp — Peak/Valley + Auto-Calibration
 *
 * Algoritma DETEKSI identik dengan BreathSensor v27 Clean.
 * Kalibrasi: 12 detik, baca ADC 50Hz, hitung P5-P95 signal range +
 * MAD noise floor, hasilkan dead_zone & min_breath_size.
 * Fallback: nilai hardcoded sama persis dengan v27 Clean.
 */

BreathSensorV2::BreathSensorV2()
    : _baseline(0.0f),
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
      _current_phase(0),
      _last_sample_ms(0)
{
    // Default fallback threshold (sama seperti BreathSensor v27 Clean)
    _calib.baseline        = 2048.0f;
    _calib.noise_floor     = FALLBACK_NOISE_FLOOR;
    _calib.signal_range    = FALLBACK_SIGNAL_RANGE;
    _calib.dead_zone       = FALLBACK_DEAD_ZONE_SLOW;
    _calib.min_breath_size = FALLBACK_MIN_BREATH_SIZE;
    _calib.is_valid        = false;
}

// ============================================================================
// SORT HELPER
// ============================================================================
void BreathSensorV2::_sort_floats(float *arr, int n) {
    for (int i = 1; i < n; i++) {
        float key = arr[i];
        int j = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// ============================================================================
// IIR FILTER
// ============================================================================
float BreathSensorV2::_applyIIR(float input) {
    _iir.x[2] = _iir.x[1];
    _iir.x[1] = _iir.x[0];
    _iir.x[0] = input;

    float output =
        IIR_B[0] * _iir.x[0] +
        IIR_B[1] * _iir.x[1] +
        IIR_B[2] * _iir.x[2]
        - IIR_A[1] * _iir.y[1]
        - IIR_A[2] * _iir.y[2];

    _iir.y[2] = _iir.y[1];
    _iir.y[1] = output;
    _iir.y[0] = output;

    return output;
}

// ============================================================================
// NVS HELPERS (namespace: "b300_brc")
// ============================================================================
void BreathSensorV2::_saveCalibration() {
    Preferences prefs;
    prefs.begin("b300_brc", false);
    prefs.putFloat("bl", _calib.baseline);
    prefs.putFloat("nf", _calib.noise_floor);
    prefs.putFloat("sr", _calib.signal_range);
    prefs.putUChar("iv", _calib.is_valid ? 1 : 0);
    prefs.end();
}

void BreathSensorV2::_loadCalibration() {
    Preferences prefs;
    prefs.begin("b300_brc", true);
    _calib.baseline     = prefs.getFloat("bl", 2048.0f);
    _calib.noise_floor  = prefs.getFloat("nf", 0.0f);
    _calib.signal_range = prefs.getFloat("sr", 0.0f);
    _calib.is_valid     = (prefs.getUChar("iv", 0) == 1);
    prefs.end();

    if (_calib.is_valid && _calib.signal_range > 1.0f) {
        _applyCalibrationThresholds(_calib);
    } else {
        _calib.is_valid = false;
    }
}

bool BreathSensorV2::_applyCalibrationThresholds(CalibData &c) {
    float nf = c.noise_floor;
    float r_ = c.signal_range;

    c.dead_zone = constrain(nf * 3.0f, 4.0f, r_ * 0.25f);
    c.dead_zone = min(c.dead_zone, 80.0f);
    c.min_breath_size = constrain(r_ * 0.08f, max(nf * 2.0f, 8.0f), r_ * 0.6f);
    c.min_breath_size = min(c.min_breath_size, 80.0f);
    return true;
}

// ============================================================================
// KALIBRASI: 12 detik, baca ADC 50Hz
// ============================================================================
bool BreathSensorV2::_calibration_run_once(float baseline_seed, void (*cb)(float, float, bool)) {
    // IDENTICAL to calib_run() in kode_pernapasan_v27_kalibrasi
    IIR_State cal_filt;
    for (int i = 0; i < 3; i++) {
        cal_filt.x[i] = 0.0f;
        cal_filt.y[i] = 0.0f;
    }

    float seed_sum     = 0.0f;
    int   seed_count   = 0;
    float filt_sum     = 0.0f;
    int   sc           = 0;
    float baseline_ema = 0.0f;

    float samp_buf[CALIB_MAX_SAMPLES];
    float diff_buf[CALIB_MAX_SAMPLES];
    int   buf_count  = 0;
    int   diff_count = 0;
    float prev_ac    = 0.0f;

    uint32_t t0          = millis();
    uint32_t last_sample = 0;

    while (true) {
        uint32_t now = millis();
        if (now - last_sample >= BREATH_SAMPLING_PERIOD) {
            last_sample = now;

            float raw  = (float)analogRead(BREATH_SENSOR_PIN);

            // IIR filter — identical to apply_iir() in original
            cal_filt.x[2] = cal_filt.x[1];
            cal_filt.x[1] = cal_filt.x[0];
            cal_filt.x[0] = raw;
            float filt = IIR_B[0] * cal_filt.x[0] + IIR_B[1] * cal_filt.x[1] + IIR_B[2] * cal_filt.x[2]
                       - IIR_A[1] * cal_filt.y[1] - IIR_A[2] * cal_filt.y[2];
            cal_filt.y[2] = cal_filt.y[1];
            cal_filt.y[1] = filt;
            cal_filt.y[0] = filt;

            if (seed_count < CALIB_SEED_SAMPLES) {
                seed_sum += filt;
                seed_count++;
                if (seed_count == CALIB_SEED_SAMPLES) {
                    baseline_ema = seed_sum / (float)CALIB_SEED_SAMPLES;
                }
            } else {
                baseline_ema = CALIB_BASELINE_ALPHA * baseline_ema
                             + (1.0f - CALIB_BASELINE_ALPHA) * filt;
            }

            float ac = filt - baseline_ema;

            if (now - t0 >= CALIB_WARMUP_MS && seed_count >= CALIB_SEED_SAMPLES) {
                filt_sum += filt;
                sc++;
                if (buf_count < CALIB_MAX_SAMPLES) {
                    samp_buf[buf_count++] = ac;
                }
                if (buf_count > 1) {
                    float d = fabsf(ac - prev_ac);
                    if (diff_count < CALIB_MAX_SAMPLES) {
                        diff_buf[diff_count++] = d;
                    }
                }
                prev_ac = ac;
            }

            float progress = constrain((float)(now - t0) / CALIB_DURATION_MS, 0.0f, 1.0f);

            if (cb) {
                cb(progress, raw, false);
            }

            if (now - t0 >= CALIB_DURATION_MS) break;
        }
        delay(5);
    }

    // ---- Hitung threshold dari sampel (IDENTICAL to original) ----
    CalibData result;
    result.baseline     = 2048.0f;
    result.noise_floor  = FALLBACK_NOISE_FLOOR;
    result.signal_range = FALLBACK_SIGNAL_RANGE;
    result.is_valid     = false;

    if (sc >= 10 && buf_count >= 10) {
        result.baseline = filt_sum / (float)sc;

        _sort_floats(samp_buf, buf_count);
        int p5_idx  = max((int)(buf_count * 0.05f), 0);
        int p95_idx = min((int)(buf_count * 0.95f), buf_count - 1);
        float p5  = samp_buf[p5_idx];
        float p95 = samp_buf[p95_idx];
        float ac_peak = max(fabsf(p5), fabsf(p95));
        float ac_range = p95 - p5;
        result.signal_range = max(ac_range, 3.0f);
        result.signal_range = min(result.signal_range, ac_peak * 6.0f);

        // Noise from median absolute difference
        if (diff_count > 10) {
            _sort_floats(diff_buf, diff_count);
            float median_diff = diff_buf[diff_count / 2];
            result.noise_floor = median_diff * 1.05f;
        } else {
            result.noise_floor = 4.0f;
        }
        result.noise_floor = max(result.noise_floor, 0.5f);
        result.noise_floor = min(result.noise_floor, result.signal_range * 0.3f);

        if (result.signal_range >= CALIB_MIN_SIGNAL_RANGE
            && result.signal_range <= CALIB_MAX_SIGNAL_RANGE
            && result.noise_floor < (result.signal_range * CALIB_MAX_NOISE_TO_RANGE)) {
            result.is_valid = true;
        }
    }

    _applyCalibrationThresholds(result);

    if (cb) {
        cb(1.0f, 0.0f, true);
    }

    if (result.is_valid) {
        _calib = result;
        _saveCalibration();
        return true;
    }

    return false;
}

// ============================================================================
// PUBLIC: calibrate() — panggil dari luar
// ============================================================================
bool BreathSensorV2::calibrate(void (*cb)(float progress, float raw, bool done)) {
    if (!_baseline_initialized) {
        Serial.println("[BreathSensorV2] ERROR: calibrate() dipanggil sebelum begin()!");
        return false;
    }

    Serial.println("[BreathSensorV2] Kalibrasi 12 detik dimulai...");
    bool ok = _calibration_run_once(_baseline, cb);

    if (ok) {
        Serial.printf("[BreathSensorV2] Kalibrasi BERHASIL: R=%.0f N=%.1f DZ=%.0f MB=%.0f\n",
                      _calib.signal_range, _calib.noise_floor,
                      _calib.dead_zone, _calib.min_breath_size);
    } else {
        Serial.println("[BreathSensorV2] Kalibrasi GAGAL, pakai threshold fallback");
    }

    // Reset state engine dengan threshold baru
    _smoothed_period    = 800.0f;
    _last_event_ms      = millis();
    _is_inhaling        = false;
    _local_max          = -99999.0f;
    _local_min          = 99999.0f;
    _last_amplitude     = _calib.min_breath_size * 2.0f;
    _last_peak_level    = _baseline + _calib.min_breath_size;
    _last_valley_level  = _baseline - _calib.min_breath_size;
    _inhale_ready       = false;
    _exhale_ready       = false;
    _current_phase      = 0;

    return ok;
}

bool BreathSensorV2::isCalibrated() {
    return _calib.is_valid;
}

CalibData BreathSensorV2::getCalibInfo() {
    return _calib;
}

// ============================================================================
// begin() — seeding 50 sampel, load NVS kalibrasi kalau ada
// ============================================================================
void BreathSensorV2::begin() {
    Serial.println("[BreathSensorV2] Mode: SENSOR + AUTO-CALIBRATION");
    Serial.printf("[BreathSensorV2] Pin ADC: GPIO%d, Sample rate: %d Hz\n",
                  BREATH_SENSOR_PIN, BREATH_SAMPLE_RATE_HZ);

    float sum = 0.0f;
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

    // Coba load kalibrasi dari NVS
    _loadCalibration();

    // Seed level
    _last_peak_level   = _baseline + _calib.min_breath_size;
    _last_valley_level = _baseline - _calib.min_breath_size;
    _last_amplitude    = _calib.min_breath_size * 2.0f;

    Serial.printf("[BreathSensorV2] Baseline awal: %.1f, calibrated=%d, DZ=%.0f MB=%.0f\n",
                  _baseline, _calib.is_valid, _calib.dead_zone, _calib.min_breath_size);
}

// ============================================================================
// update() — identik algoritma dengan BreathSensor v27 Clean
// ============================================================================
void BreathSensorV2::update() {
    if (!_baseline_initialized) return;

    unsigned long now = millis();
    if (now - _last_sample_ms < BREATH_SAMPLING_PERIOD) return;
    _last_sample_ms = now;

    // 1. Baca ADC
    float raw = (float)analogRead(BREATH_SENSOR_PIN);

    // 2. Filter IIR
    float filtered = _applyIIR(raw);

    // 4. Estimasi periode napas (smoothed)
    float raw_period = (float)(now - _last_event_ms);
    if (raw_period < 1.0f) raw_period = 1.0f;
    _smoothed_period = 0.8f * _smoothed_period + 0.2f * raw_period;

    // 5. Alpha adaptif (anti-drift)
    float t = (_smoothed_period - PERIOD_ALPHA_START) /
              (PERIOD_ALPHA_END - PERIOD_ALPHA_START);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float current_alpha = (1.0f - t) * DETREND_ALPHA_FAST
                        + t           * DETREND_ALPHA_HOLD;

    // 6. Update baseline
    _baseline = current_alpha * _baseline + (1.0f - current_alpha) * filtered;

    // 7. Sinyal AC
    float ac_signal = filtered - _baseline;

    // 8. Hold detection
    bool is_holding = (raw_period > (float)HOLD_LOCKDOWN_MS);

    // 9. Pilih threshold: kalibrasi vs fallback
    float min_d_f, dz_f, min_d_s, dz_s, ratio_ef, ratio_if, ratio_es, ratio_is;

    if (_calib.is_valid) {
        // --- MODE KALIBRASI ---
        dz_s   = _calib.dead_zone;
        dz_f   = max(_calib.dead_zone * 0.3f, 30.0f);
        min_d_s = _calib.dead_zone * 1.5f;
        min_d_f = max(_calib.dead_zone * 0.4f, 30.0f);
        ratio_ef = 0.08f;  ratio_if = 0.15f;
        ratio_es = 0.25f;  ratio_is = 0.20f;
    } else {
        // --- FALLBACK (identik BreathSensor v27 Clean) ---
        dz_f    = FALLBACK_DEAD_ZONE_FAST;
        dz_s    = FALLBACK_DEAD_ZONE_SLOW;
        min_d_f = FALLBACK_MIN_DELTA_FAST;
        min_d_s = FALLBACK_MIN_DELTA_SLOW;
        ratio_ef = FALLBACK_RATIO_EXHALE_FAST;
        ratio_if = FALLBACK_RATIO_INHALE_FAST;
        ratio_es = FALLBACK_RATIO_EXHALE_SLOW;
        ratio_is = FALLBACK_RATIO_INHALE_SLOW;
    }

    float active_min_delta, active_dead_zone;
    float active_ratio_exhale, active_ratio_inhale;

    if (_smoothed_period > PERIOD_FAST_SLOW_THRES) {
        active_min_delta    = min_d_s;
        active_dead_zone    = dz_s;
        active_ratio_exhale = ratio_es;
        active_ratio_inhale = ratio_is;
    } else {
        active_min_delta    = min_d_f;
        active_dead_zone    = dz_f;
        active_ratio_exhale = ratio_ef;
        active_ratio_inhale = ratio_if;
    }

    float active_min_breath = _calib.min_breath_size;

    // 10. Deteksi ekstremum
    if (_is_inhaling) {
        if (ac_signal > _local_max) {
            _local_max   = ac_signal;
            _time_at_max = now;
        }

        float delta = _last_amplitude * active_ratio_exhale;
        if (delta < active_min_delta) delta = active_min_delta;
        if (is_holding) delta = MAX_DELTA;

        if (ac_signal < (_local_max - delta)) {
            if (_local_max > active_dead_zone) {
                if ((long)(_time_at_max - _last_event_ms) > MIN_BREATH_DURATION) {
                    float height = _local_max - _last_valley_level;
                    if (height > active_min_breath) {
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

                        // Serial.printf("[BREATH] EXHALE detected, AC_peak=%.1f\n", _local_max);
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
        if (is_holding) delta = MAX_DELTA;

        if (ac_signal > (_local_min + delta)) {
            if (_local_min < -active_dead_zone) {
                if ((long)(_time_at_min - _last_event_ms) > MIN_BREATH_DURATION) {
                    float depth = _last_peak_level - _local_min;
                    if (depth > active_min_breath) {
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

                        // Serial.printf("[BREATH] INHALE detected, AC_valley=%.1f\n", _local_min);
                    }
                }
            }
        }
    }
}

// ============================================================================
// API KOMPATIBEL
// ============================================================================
void BreathSensorV2::triggerGuide(int8_t direction, float phase_duration_ms) {
    (void)direction;
    (void)phase_duration_ms;
}

bool BreathSensorV2::isInhaleDetected() {
    return _inhale_ready;
}

bool BreathSensorV2::isExhaleDetected() {
    return _exhale_ready;
}

uint32_t BreathSensorV2::getLastEventTimestamp() {
    _inhale_ready = false;
    _exhale_ready = false;
    return _last_event_ts;
}

int8_t BreathSensorV2::getLastEventType() {
    return _last_event_type;
}

int8_t BreathSensorV2::getCurrentPhase() {
    return _current_phase;
}