#include "StepSensor.h"
extern SemaphoreHandle_t wireMutex;

StepSensor::StepSensor() {
  stepTriggered = false;
  total_steps = 0;
  last_step_time = 0;
  last_event_time = 0;
  last_sample_time = 0;
  adaptive_threshold = 0.2;
  peak_history_index = 0;
  is_waiting_for_step = true;
  spm_smoothed = 0.0;

  memset(BPF_in_history, 0, sizeof(BPF_in_history));
  memset(BPF_out_history, 0, sizeof(BPF_out_history));
  memset(signal_window, 0, sizeof(signal_window));
  for (int i = 0; i < PEAK_HISTORY_SIZE; i++) peak_history[i] = 0.2;
}

void StepSensor::begin() {
  Serial.println("[SENSOR] Menginisialisasi MPU6050 via Wire...");

  // Mutex untuk begin() — 3 transmisi Wire dibungkus satu mutex
  if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(50)) == pdTRUE) {

    // 1. Bangunkan MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // 2. Set Sensitivitas +/- 4G
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1C);
    Wire.write(0x08);
    Wire.endTransmission(true);

    // 3. Set DLPF 10Hz
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A);
    Wire.write(0x05);
    Wire.endTransmission(true);

    xSemaphoreGive(wireMutex); // Lepas mutex setelah 3 transmisi selesai
  }

  last_event_time = millis();
}

float StepSensor::applyBandPassFilter(float input) {
  float output = BPF_B0 * input + BPF_B2 * BPF_in_history[1] + BPF_B4 * BPF_in_history[3];
  output -= (BPF_A1 * BPF_out_history[0] + BPF_A2 * BPF_out_history[1] + BPF_A3 * BPF_out_history[2] + BPF_A4 * BPF_out_history[3]);
  for (int i = 3; i > 0; i--) {
    BPF_in_history[i] = BPF_in_history[i - 1];
    BPF_out_history[i] = BPF_out_history[i - 1];
  }
  BPF_in_history[0] = input;
  BPF_out_history[0] = output;
  return output;
}

void StepSensor::updateAdaptiveThreshold(float new_peak) {
  float safe_peak = (new_peak > THRESHOLD_MAX) ? THRESHOLD_MAX : new_peak;
  if (new_peak < (adaptive_threshold * 0.7)) {
    for (int i = 0; i < PEAK_HISTORY_SIZE; i++) peak_history[i] = (peak_history[i] + safe_peak) / 2.0;
  }
  peak_history[peak_history_index] = safe_peak;
  peak_history_index = (peak_history_index + 1) % PEAK_HISTORY_SIZE;

  float sum = 0;
  for (int i = 0; i < PEAK_HISTORY_SIZE; i++) sum += peak_history[i];
  adaptive_threshold = (sum / PEAK_HISTORY_SIZE) * THRESHOLD_FACTOR;
  adaptive_threshold = constrain(adaptive_threshold, THRESHOLD_MIN, THRESHOLD_MAX);
}

void StepSensor::update() {
  unsigned long now_us = micros();

  if (now_us - last_sample_time >= SAMPLE_PERIOD_US) {
    last_sample_time += SAMPLE_PERIOD_US;

    // --- Baca sensor dengan mutex ---
    float ax = 0, ay = 0, az = 0;

    if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
      Wire.beginTransmission(MPU_ADDR);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(MPU_ADDR, 6, true);

      if (Wire.available() == 6) {
        int16_t AcX = Wire.read() << 8 | Wire.read();
        int16_t AcY = Wire.read() << 8 | Wire.read();
        int16_t AcZ = Wire.read() << 8 | Wire.read();
        ax = AcX / 8192.0;
        ay = AcY / 8192.0;
        az = AcZ / 8192.0;
      } else {
        // Baca I2C tidak lengkap — buang sampel ini, jangan inject 0g ke filter
        xSemaphoreGive(wireMutex);
        return;
      }

      xSemaphoreGive(wireMutex); // Lepas mutex segera setelah baca selesai
    } else {
      // Gagal ambil mutex (Wire sedang dipakai) — skip sampel ini
      return;
    }
    // --- Selesai akses Wire ---

    // Semua proses di bawah ini tidak butuh Wire, jadi di luar mutex
    float raw_mag = sqrt(ax*ax + ay*ay + az*az);

    float filtered = applyBandPassFilter(raw_mag);
    signal_window[0] = signal_window[1];
    signal_window[1] = signal_window[2];
    signal_window[2] = filtered;

    float prev = signal_window[0], curr = signal_window[1], next = signal_window[2];
    bool is_peak = (curr > prev) && (curr > next);

    unsigned long now_ms = millis();
    unsigned long time_since_last_step = now_ms - last_step_time;

    if (time_since_last_step > 650) {
      is_waiting_for_step = true;
      if (adaptive_threshold > THRESHOLD_MIN) adaptive_threshold *= 0.96;
      if (spm_smoothed > 0) spm_smoothed *= 0.98;
    }

    unsigned long step_period = (last_step_time > 0) ? (now_ms - last_step_time) : 600;
    unsigned long dynamic_mask = constrain((unsigned long)(0.40 * step_period), MASK_MIN_MS, MASK_MAX_MS);

    if (is_waiting_for_step && is_peak && (curr > adaptive_threshold)) {
      if (now_ms - last_event_time > dynamic_mask && raw_mag < IMPACT_LIMIT_G) {

        if (last_step_time > 0 && time_since_last_step < 4000) {
          float spm_raw = MS_PER_MINUTE / time_since_last_step;
          spm_smoothed = (1.0 - SPM_ALPHA) * spm_smoothed + SPM_ALPHA * spm_raw;
        }

        total_steps++;
        last_step_time = now_ms;
        last_event_time = now_ms;
        updateAdaptiveThreshold(curr);
        is_waiting_for_step = false;
        stepTriggered = true;
      }
    }

    float dynamic_hysteresis = -0.1 * (adaptive_threshold / THRESHOLD_MAX);
    if (!is_waiting_for_step && (curr < dynamic_hysteresis || (now_ms - last_event_time > 500))) {
      is_waiting_for_step = true;
    }
  }
}

bool StepSensor::isStepDetected() {
  if (stepTriggered) {
    stepTriggered = false;
    return true;
  }
  return false;
}

float StepSensor::getSPM() {
  return spm_smoothed;
}