#ifndef STEPSENSOR_H
#define STEPSENSOR_H

#include <Arduino.h>
#include <Wire.h>

class StepSensor {
  public:
    StepSensor();
    void begin();
    void update(); 
    bool isStepDetected(); 
    float getSPM(); 

  private:
    const int MPU_ADDR = 0x68; // Alamat I2C MPU6050

    volatile bool stepTriggered;
    volatile float spm_smoothed;

    // --- Konstanta dari Sensor Lari (Tuning) ---
    const float G_TO_MS2 = 9.80665;
    const float MS_PER_MINUTE = 60000.0;
    const float SAMPLING_FREQ = 100.0;
    const unsigned long SAMPLE_PERIOD_US = 10000UL; // 100Hz, cocok dg koefisien BPF
    
    const float THRESHOLD_FACTOR = 0.40;     
    const static int PEAK_HISTORY_SIZE = 5;         
    const float THRESHOLD_MIN = 0.12;
    const float THRESHOLD_MAX = 2.5;         
    const float IMPACT_LIMIT_G = 4.5;        

    const unsigned long MASK_MIN_MS = 230;
    const unsigned long MASK_MAX_MS = 600;   

    // --- Koefisien Band-Pass Filter (Orde-4) ---
    const float BPF_B0 = 0.1985;
    const float BPF_B2 = -0.3971; 
    const float BPF_B4 = 0.1985;
    const float BPF_A1 = -2.3525; 
    const float BPF_A2 = 1.9429;
    const float BPF_A3 = -0.7905; 
    const float BPF_A4 = 0.2010;

    // --- Variabel Global Sensor ---
    unsigned long total_steps;
    unsigned long last_step_time;
    unsigned long last_event_time;
    unsigned long last_sample_time;

    float BPF_in_history[4];
    float BPF_out_history[4];
    float signal_window[3];
    float adaptive_threshold;
    float peak_history[PEAK_HISTORY_SIZE]; 
    int peak_history_index;

    bool is_waiting_for_step; 
    const float SPM_ALPHA = 0.2; 

    // --- Fungsi Helper Internal ---
    float applyBandPassFilter(float input);
    void updateAdaptiveThreshold(float new_peak);
};

#endif