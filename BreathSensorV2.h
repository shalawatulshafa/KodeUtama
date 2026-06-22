#pragma once
#include <Arduino.h>
#include <Preferences.h>

/*
 * BreathSensorV2 — Peak/Valley + Auto-Calibration
 *
 * Identik algorithm dengan kode_pernapasan_v27_kalibrasi, dibungkus class.
 * Threshold (dead_zone, min_breath_size) dihitung dari auto-kalibrasi 12 detik.
 * Fallback: nilai hardcoded sama seperti BreathSensor v27 Clean.
 *
 * API kompatibel dengan BreathSensor lama:
 *   begin(), update(), isInhaleDetected(), isExhaleDetected(),
 *   getLastEventTimestamp(), getLastEventType(), getCurrentPhase(),
 *   triggerGuide()
 *
 * Tambahan:
 *   calibrate()       — auto-kalibrasi 12 detik, return true jika berhasil
 *   isCalibrated()    — cek apakah threshold pakai hasil kalibrasi
 *   getCalibInfo()    — baca nilai R, N, DZ, MB (untuk dump ke Serial)
 */

#define BREATH_SENSOR_PIN        34
#define BREATH_SAMPLE_RATE_HZ    50
#define BREATH_SAMPLING_PERIOD   (1000 / BREATH_SAMPLE_RATE_HZ)

// ---- Filter IIR orde-2 (1.5Hz) ----
static const float IIR_B[3] = {0.007821f, 0.015642f, 0.007821f};
static const float IIR_A[3] = {1.0f, -1.73471f, 0.76599f};

// ---- DSP Constants (fallback, identik BreathSensor v27 Clean) ----
#define DETREND_ALPHA_FAST          0.98f
#define DETREND_ALPHA_HOLD          0.999f
#define PERIOD_FAST_SLOW_THRES      1000.0f
#define PERIOD_ALPHA_START          600.0f
#define PERIOD_ALPHA_END            1400.0f
#define MIN_BREATH_DURATION         150
#define HOLD_LOCKDOWN_MS            3000
#define MAX_DELTA                   400.0f

#define FALLBACK_MIN_DELTA_FAST     30.0f
#define FALLBACK_DEAD_ZONE_FAST     30.0f
#define FALLBACK_RATIO_EXHALE_FAST  0.08f
#define FALLBACK_RATIO_INHALE_FAST  0.15f

#define FALLBACK_MIN_DELTA_SLOW     80.0f
#define FALLBACK_DEAD_ZONE_SLOW     80.0f
#define FALLBACK_RATIO_EXHALE_SLOW  0.25f
#define FALLBACK_RATIO_INHALE_SLOW  0.20f

#define FALLBACK_MIN_BREATH_SIZE    80.0f

// ---- Kalibrasi ----
#define CALIB_DURATION_MS           12000
#define CALIB_WARMUP_MS             1500
#define CALIB_SEED_SAMPLES          5
#define CALIB_MAX_SAMPLES           450
#define CALIB_BASELINE_ALPHA        0.98f
#define CALIB_MIN_SIGNAL_RANGE      30.0f
#define CALIB_MAX_SIGNAL_RANGE      2000.0f
#define CALIB_MAX_NOISE_TO_RANGE    0.25f
#define FALLBACK_NOISE_FLOOR        10.0f
#define FALLBACK_SIGNAL_RANGE       400.0f

// ---- Calibration Data Struct ----
struct CalibData {
    float baseline        = 2048.0f;
    float noise_floor     = 10.0f;
    float signal_range    = 400.0f;
    float dead_zone       = 80.0f;
    float min_breath_size = 80.0f;
    bool  is_valid        = false;
};

// ---- IIR State ----
struct IIR_State {
    float x[3] = {0, 0, 0};
    float y[3] = {0, 0, 0};
};

class BreathSensorV2 {
public:
    BreathSensorV2();

    // ---- API Kompatibel ----
    void     begin();
    void     update();

    bool     isInhaleDetected();
    bool     isExhaleDetected();
    uint32_t getLastEventTimestamp();
    int8_t   getLastEventType();
    int8_t   getCurrentPhase();
    void     triggerGuide(int8_t direction, float phase_duration_ms);

    // ---- Kalibrasi ----
    bool     calibrate(/*progress callback*/ void (*cb)(float progress, float raw, bool done) = nullptr);
    bool     isCalibrated();
    CalibData getCalibInfo();

private:
    // ---- IIR Filter ----
    float    _applyIIR(float input);
    IIR_State _iir;

    // ---- Baseline + Deteksi ----
    float    _baseline;
    bool     _baseline_initialized;
    float    _smoothed_period;
    uint32_t _last_event_ms;
    bool     _is_inhaling;
    float    _local_max;
    float    _local_min;
    uint32_t _time_at_max;
    uint32_t _time_at_min;
    float    _last_amplitude;
    float    _last_peak_level;
    float    _last_valley_level;

    // ---- Event flags ----
    bool     _inhale_ready;
    bool     _exhale_ready;
    uint32_t _last_event_ts;
    int8_t   _last_event_type;
    int8_t   _current_phase;

    // ---- Kalibrasi ----
    CalibData _calib;
    void     _loadCalibration();
    void     _saveCalibration();
    bool     _applyCalibrationThresholds(CalibData &c);
    bool     _calibration_run_once(float baseline_seed, void (*cb)(float, float, bool));
    void     _sort_floats(float *arr, int n);

    uint32_t _last_sample_ms;
};