#include <Arduino.h>
#include <math.h>
#include "BluetoothA2DPSource.h"
#include <Preferences.h>
#include <Wire.h>
#include <vector>

#include <LittleFS.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include "StepSensor.h"
#include "BatteryMgr.h"
#include "BreathSensor.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "soc/rtc.h"
// ==========================================
// STRUKTUR DATA BINER B200 (Ukuran Pasti: 9 Byte)
// ==========================================
// ==========================================

// Forward declare — fungsi ada di ESP-IDF binary tapi header-nya private
// (esp_private/esp_clk.h) jadi tidak bisa di-include langsung dari Arduino sketch.
// Return: calibrated slow clock period * 2^19 (us per cycle, fractional).
extern "C" uint32_t esp_clk_slowclk_cal_get(void);

// Convert RTC slow clock ticks → microseconds pakai CALIBRATED period
// (bukan nominal freq). esp_clk_slowclk_cal_get() hasil kalibrasi otomatis
// ESP-IDF di boot pakai main XTAL sebagai referensi.
// Akurat ~0.1% vs ~20% kalau pakai rtc_clk_slow_freq_get_hz() (yang nominal).
static inline uint64_t rtcTicksToUs(uint64_t ticks) {
  return (ticks * (uint64_t)esp_clk_slowclk_cal_get()) >> RTC_CLK_CAL_FRACT;
}

void ensureBTControllerBTDM() {
    auto status = esp_bt_controller_get_status();
    Serial.printf("[BT] controller status: %d\n", (int)status);
    if (status == ESP_BT_CONTROLLER_STATUS_ENABLED) return;

    if (status == ESP_BT_CONTROLLER_STATUS_IDLE) {
        esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_err_t err = esp_bt_controller_init(&cfg);
        Serial.printf("[BT] controller init: %s\n", esp_err_to_name(err));
        uint32_t t = millis();
        while (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE
               && millis() - t < 2000);
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        esp_err_t err = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
        Serial.printf("[BT] controller enable BTDM: %s\n", esp_err_to_name(err));
        delay(200);
    }
    Serial.printf("[BT] status setelah ensure: %d\n",
                  (int)esp_bt_controller_get_status());
}
//#define TESTING_SPM 120   // ← DI-DISABLE untuk production. Uncomment untuk simulasi 200 SPM tanpa MPU6050.
// PENTING UNTUK TA: Pastikan macro ini OFF saat ambil data lari beneran,
// karena kalau ON, step sensor (MPU6050) DI-BYPASS dan diganti fake-step setiap 300ms.

// === TESTING_BREATH: dummy breath events kalau sensor napas belum ada ===
// Uncomment untuk simulasi transisi hirup/buang yang SYNC dengan pola aktif (3:2 atau 2:1).
// Fake event ditulis ke /run.dat dengan step=0 — sama format dengan data real.
// Hasilnya: file siap dipakai untuk test parser Flutter HP, tidak perlu sensor napas wired.
// #define TESTING_BREATH
// // Optional lag (ms): + = user telat dari aturan, - = user cepat, 0 = perfect adherence
// // Coba ubah 50 atau -50 untuk simulasi pola yang tidak sempurna (testing edge case parser).
// #define TESTING_BREATH_LAG_MS 0

// === TESTING_USB_ONLY: test tanpa batrai (cuma USB ke ESP32) ===
// Uncomment kalau lagi test/develop pakai USB doang tanpa colok batrai.
// Tanpa batrai, MAX17048 (fuel gauge) baca 0V/0% → trigger battery critical
// → enterDeepSleep → restart loop forever. Macro ini SKIP register cbCritical
// supaya battery critical tidak fire walau sensor baca 0.
// PENTING UNTUK TA: WAJIB di-DISABLE (comment) saat produksi pakai batrai beneran,
// kalau tidak alat tidak akan auto-sleep saat batrai mau habis = batrai over-discharge & rusak.
//#define TESTING_USB_ONLY

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

BluetoothA2DPSource a2dp_source;
Preferences preferences;
StepSensor stepSensor;
BreathSensor breathSensor;

BatteryMgr battery;
SemaphoreHandle_t wireMutex = NULL;
SemaphoreHandle_t logMutex  = NULL;
BLEServer *pServer          = nullptr;
bool ble_initialized        = false;

const int PIN_BTN_OK   = 32;
const int PIN_BTN_UP   = 18;
const int PIN_BTN_DOWN = 19;
const int PIN_LED      = 2;



enum UIState {
  STATE_IDLE,
  STATE_SCANNING,
  STATE_SELECTING,
  STATE_PAIRING,
  STATE_MENU_PATTERN,
  STATE_MAIN_MENU,
  STATE_SYNC
};

UIState current_state = STATE_IDLE;
unsigned long scan_start_time = 0;
const int SCAN_DURATION = 20000;

int main_menu_index  = 0;
int current_volume   = 50;

volatile bool is_shutting_down   = false;
volatile bool logging_enabled    = false;
volatile bool is_pattern_switching = false;

volatile bool guiding_ever_started = false;
volatile bool recording_active     = false;
unsigned long recording_start_ms   = 0;

volatile int  plot_guide_state  = 0;
volatile bool plot_step_trigger = false;

unsigned long startup_time             = 0;
bool is_initial_connect_attempt        = true;
bool has_shown_not_found               = false;
bool ble_deinit_done                   = false;
bool a2dp_active                       = false;

enum GuidanceState { STATE_ANALYZING, STATE_GUIDING };
GuidanceState algo_state = STATE_ANALYZING;

#define SMA_BUFFER_SIZE 5
long step_intervals[SMA_BUFFER_SIZE];
int  interval_idx   = 0;
float T_avg         = 0;
long  t_last_step   = 0;
bool  sma_initialized = false;

int stable_count        = 0;
int global_step_count   = 0;
int missed_step_count   = 0;
int sched_steps_ahead   = 1;   // penjadwalan adaptif: cue 1 atau 2 langkah ke depan
esp_timer_handle_t audio_timer;

// === MODEL PENJADWALAN PANDUAN (spek B200 §2.1.5) ===
// time_to_trigger = N×T_avg − (T_LEAD_MS + L_SYS); N=1 atau 2 (adaptif untuk SPM tinggi).
// L_SYS = buffer firmware (~18ms, terukur scope) + A2DP SBC (~150ms, literatur).
// 2-langkah −10ms karena suku error-prediksi (interval−T_avg) terhitung 2×.
static const int T_LEAD_MS = 170;   // lead di telinga (spek)
static const int L_SYS_1   = 168;   // 1-langkah (18 + 150)
static const int L_SYS_2   = 158;   // 2-langkah (168 − 10)
TaskHandle_t SensorTaskHandle = NULL;
int active_pattern_divisor    = 5;
int selected_rhythm_index     = 0;

volatile float phase_duration = 400.0f;
volatile float freq_start_g   = 400.0f;
volatile float freq_end_g     = 880.0f;

double phase = 0.0;
volatile bool is_playing_sound = false;

volatile bool  new_sound_requested = false;
volatile float new_freq_start, new_freq_end;
volatile float new_phase_duration;

volatile uint32_t sample_pos_g    = 0;
volatile uint32_t total_samples_g = 0;

// Pattern position untuk audio cue: di-set di handleStepAnalysis saat step
// real terdeteksi, di-consume oleh audioTimerCallback saat timer fire.
// Pakai pola ini supaya counter selalu sinkron dengan ACTUAL step detection,
// bukan timer fire (yang bisa di-cancel kalau step berikutnya datang cepat).
// -1 artinya tidak ada cue pending.
volatile int scheduled_pattern_pos = -1;

// ==========================================
// LOGGING (FORMAT TEKS CSV)
// ==========================================
// File /run.dat berisi teks CSV ringkas (tanggal TIDAK disimpan per baris):
//   SESSION:sesi_id:unix_time\n   ← penanda sesi, tanggal ada di sini
//   sesi_id,ts_ms,breathPhase,step,spm,patternID;\n
// Saat SYNC, SESSION header diparse → mulai_lari disisipkan per baris → HP terima 7 kolom.
File logFile;

// Nomor sesi dan unix time awal sesi aktif
uint32_t current_session_id   = 0;
uint32_t current_session_unix = 0;

bool isSyncing = false;
volatile bool sync_completed_ack = false;  // di-set TRUE saat HP kirim "OK"/"DONE"/"ACK"/"SELESAI"

// ==========================================
// TIME TRACKING
// ==========================================
RTC_DATA_ATTR uint32_t rtc_unix_at_sleep = 0;
RTC_DATA_ATTR uint64_t rtc_us_at_sleep   = 0;
RTC_DATA_ATTR bool     rtc_time_valid    = false;

uint32_t saved_unix_time = 0;
uint32_t millis_at_sync  = 0;
bool     time_is_valid   = false;

uint32_t getCurrentUnix() {
  if (!time_is_valid) return 0;
  uint32_t drift_sec = (millis() - millis_at_sync) / 1000UL;
  return saved_unix_time + drift_sec;
}

void unixToString(uint32_t unix_ts, char *buf) {
  uint32_t t   = unix_ts + 7 * 3600UL;
  uint32_t sec = t % 60; t /= 60;
  uint32_t min = t % 60; t /= 60;
  uint32_t hr  = t % 24; t /= 24;
  uint32_t days = t;
  uint32_t year = 1970;
  while (true) {
    bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
    uint32_t days_in_year = leap ? 366 : 365;
    if (days < days_in_year) break;
    days -= days_in_year;
    year++;
  }
  bool leap = (year % 4 == 0 && (year % 100 != 0 || year % 400 == 0));
  const uint8_t dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
  uint32_t month = 1;
  for (int m = 0; m < 12; m++) {
    uint32_t d = dim[m] + (m == 1 && leap ? 1 : 0);
    if (days < d) { month = m + 1; break; }
    days -= d;
  }
  uint32_t day = days + 1;
  sprintf(buf, "%02u/%02u/%04u %02u:%02u", day, month, year, hr, min);
}

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLECharacteristic *pCharacteristic;
BLECharacteristic *pBattCharacteristic = nullptr; // UUID 0x2A19, notify level baterai ke HP

struct FoundDevice {
  String name;
  esp_bd_addr_t address;
  bool is_cmd_rescan;
};
std::vector<FoundDevice> found_devices;
int selected_device_index = 0;

uint8_t current_mac[6] = {0};
bool is_mac_configured  = false;

unsigned long last_btn_press_time = 0;
const int DEBOUNCE_DELAY          = 200;
bool btn_ok_last_state            = HIGH;
unsigned long btn_ok_press_start  = 0;
bool btn_ok_long_detected         = false;
const int LONG_PRESS_MS           = 800;
const int SLEEP_PRESS_MS          = 2000;
unsigned long ignore_btn_until    = 0;
bool trigger_hard_scan            = false;

// ==========================================
// FORWARD DECLARATIONS
// ==========================================
void writeLogEntry(uint32_t ts_ms, int8_t breathPhase, float spm_val, bool is_step_event = true);
void flushLogBuffer();
bool target_filter_callback(const char *ssid, esp_bd_addr_t address, int rssi);
void connection_state_changed(esp_a2d_connection_state_t state, void *ptr);

// ==========================================
// BOOT MODE & SAFE MODE INFRASTRUCTURE
// ==========================================
// Single-mode-per-boot architecture: hindari BTDM transition crash dengan
// ESP.restart() saat user switch mode A2DP <-> BLE. RTC_DATA_ATTR + RTC
// counter SURVIVE soft reset, jadi waktu user TETAP persisten.
enum BootMode : uint8_t {
  BOOT_AUDIO      = 0,  // Mode A2DP untuk audio guidance lari
  BOOT_SYNC       = 1,  // Mode BLE untuk sync data ke HP
  BOOT_DEEP_SLEEP = 2   // Restart-to-sleep: BTDM ga di-init, langsung deep sleep
};

// boot_attempt: counter crash beruntun (anti infinite-restart-loop)
// DI-NVS, bukan RTC_DATA_ATTR — karena RTC_DATA_ATTR di-RESET saat ESP.restart()
// di Arduino ESP32 (bootloader reload .rtc.data dari flash), jadi counter tidak
// akan pernah naik > 1 dan safe mode tidak pernah trigger.
// NVS persisten lewat semua jenis reset kecuali corruption (sanity check via fallback 0).
uint8_t boot_attempt              = 0;      // di-load dari NVS di setup(), reset saat boot sukses
volatile bool restart_pending     = false;  // anti double-press selama restart in-flight

BootMode readBootMode();
void requestBootMode(BootMode mode, const char* reason);
void recoverI2CBus(uint8_t sda_pin, uint8_t scl_pin);
void enterSafeMode(const char* msg);
void initBLEForSync();

// Helper NVS untuk boot_attempt counter
uint8_t loadBootAttemptFromNVS() {
  preferences.begin("b300_cfg", true);
  uint8_t val = preferences.getUChar("boot_attempt", 0);
  preferences.end();
  // Sanity: kalau NVS corrupt return aneh, fallback ke 0
  return (val > 100) ? 0 : val;
}

void saveBootAttemptToNVS(uint8_t val) {
  preferences.begin("b300_cfg", false);
  preferences.putUChar("boot_attempt", val);
  preferences.end();
}

const int SAMPLE_RATE = 44100;

// ==========================================
// DISPLAY HELPER
// ==========================================
void safeDisplayUpdate() {
    if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        display.display();
        xSemaphoreGive(wireMutex);
    }
}

void updateDisplay(String header, String content, bool largeText = false) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(header);
  battery.draw(display);
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setTextSize(largeText ? 2 : 1);
  display.setCursor(0, 25);
  display.println(content);
  safeDisplayUpdate();
}

void showMainMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("MAIN MENU");
  battery.draw(display);
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setCursor(10, 20);
  if(main_menu_index == 0) display.print("> "); else display.print("  ");
  display.println("1. SCAN DEVICE");
  display.setCursor(10, 35);
  if(main_menu_index == 1) display.print("> "); else display.print("  ");
  display.println("2. SYNC");
  display.setCursor(10, 50);
  if(main_menu_index == 2) display.print("> "); else display.print("  ");
  display.println("3. EXIT");
  safeDisplayUpdate();
}

void updateMonitorDisplay() {
  static unsigned long last_disp = 0;
  if (millis() - last_disp < 250) return;
  last_disp = millis();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  if(active_pattern_divisor == 5) display.print("MODE: 3:2");
  else display.print("MODE: 2:1");
  battery.draw(display);
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setCursor(0, 20);
  if (algo_state == STATE_ANALYZING) display.print("Status: ANALYZING");
  else {
    display.print("Status: GUIDING >>");
    display.setCursor(110, 20);
    display.print(recording_active ? "*REC" : "    ");
  }
  display.setCursor(0, 35);
  display.print("Sim SPM: "); display.println((int)stepSensor.getSPM());
  display.setCursor(0, 50);
  display.print("Vol: "); display.print(current_volume);
  if(is_playing_sound) display.print(" *");
  safeDisplayUpdate();
}

void showSelectionScreen() {
  if (found_devices.empty()) return;
  FoundDevice &dev = found_devices[selected_device_index];
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Device "); display.print(selected_device_index + 1);
  display.print("/"); display.print(found_devices.size());
  battery.draw(display);
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setCursor(0, 25);
  display.println(dev.name);
  display.setCursor(0, 50);
  if (dev.is_cmd_rescan) display.print("[ OK TO RESCAN ]");
  else display.print("[ OK TO PAIR ]");
  safeDisplayUpdate();
}

void showRhythmMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("SELECT RHYTHM");
  battery.draw(display);
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setCursor(10, 25);
  if(selected_rhythm_index == 0) display.print("> "); else display.print("  ");
  display.println("Pola 3:2");
  display.setCursor(10, 45);
  if(selected_rhythm_index == 1) display.print("> "); else display.print("  ");
  display.println("Pola 2:1");
  safeDisplayUpdate();
}

// ==========================================
// AUDIO CALLBACK
// ==========================================
int32_t get_sound_data(uint8_t *data, int32_t len) {
  // --- TAMBAHAN PENGAMAN ---
  // Mencegah memory crash jika dipanggil saat koneksi sedang putus
  if (data == nullptr || len <= 0) {
    return 0;
  }
  // -------------------------
  if (is_shutting_down || current_state != STATE_IDLE) {
    memset(data, 0, len);
    return len;
  }

  int16_t *data16   = (int16_t*)data;
  int32_t sample_count = len / 2;

  if (new_sound_requested) {
    freq_start_g      = new_freq_start;
    freq_end_g        = new_freq_end;
    phase_duration    = new_phase_duration;
    sample_pos_g      = 0;
    total_samples_g   = (uint32_t)(new_phase_duration * (SAMPLE_RATE / 1000.0f));
    phase             = 0.0;
    is_playing_sound  = true;
    new_sound_requested = false;
  }

  if (!is_playing_sound || total_samples_g == 0) {
    memset(data, 0, len);
    return len;
  }

  const float TWO_PI_F = 2.0f * (float)M_PI;

  for (int i = 0; i < sample_count; i += 2) {
    if (sample_pos_g >= total_samples_g) {
      data16[i]   = 0;
      data16[i+1] = 0;
      if (sample_pos_g == total_samples_g) is_playing_sound = false;
      continue;
    }

    float progress = (float)sample_pos_g / (float)total_samples_g;
    float env;
    if (progress < 0.15f) {
      env = progress / 0.15f;
    } else {
      float t = (progress - 0.15f) / 0.85f;
      env = cosf(t * ((float)M_PI * 0.5f));
    }

    float amplitude = ((float)current_volume / 127.0f) * env * 26000.0f;
    float freq_now  = freq_start_g + (freq_end_g - freq_start_g) * progress;
    float phase_inc = (TWO_PI_F * freq_now) / (float)SAMPLE_RATE;
    int16_t sample_val = (int16_t)(amplitude * sinf((float)phase));

    phase += phase_inc;
    if (phase > TWO_PI_F) phase -= TWO_PI_F;

    data16[i]   = sample_val;
    data16[i+1] = sample_val;
    sample_pos_g++;
  }

  return len;
}

// ==========================================
// AUDIO TIMER CALLBACK
// ==========================================
void audioTimerCallback(void* arg) {
  if (is_shutting_down) return;
  // ── GUARD: suara panduan HANYA bunyi kalau recording aktif ──
  // Dengan ini: user dengar suara = jaminan data direkam.
  // Kalau recording pause (>3 langkah meleset), audio langsung hening
  // → user sadar & bisa lihat OLED: "Status: ANALYZING".
  if (!recording_active) return;
  if (algo_state != STATE_GUIDING || current_state != STATE_IDLE) return;

  // Counter pre-incremented di handleStepAnalysis (per actual step).
  // Di sini cuma consume pattern_pos yang sudah di-latch.
  // Kalau pos < 0 = tidak ada cue di-schedule (defensive).
  int pattern_pos = scheduled_pattern_pos;
  if (pattern_pos < 0) return;
  scheduled_pattern_pos = -1;  // consume — fire sekali aja per latch

  bool start_inhale = false;
  bool start_exhale = false;

  if (active_pattern_divisor == 5) {
    if (pattern_pos == 0) start_inhale = true;
    else if (pattern_pos == 3) start_exhale = true;
  } else if (active_pattern_divisor == 3) {
    if (pattern_pos == 0) start_inhale = true;
    else if (pattern_pos == 2) start_exhale = true;
  }

  if (start_inhale || start_exhale) {
    int steps_in_phase = (start_inhale) ? ((active_pattern_divisor == 5) ? 3 : 2)
                                        : ((active_pattern_divisor == 5) ? 2 : 1);
    float freq_s, freq_e;
    if (start_inhale) {
      freq_s = 440.0f; freq_e = 880.0f;
      // plot_guide_state sudah di-set di handleStepAnalysis (immediate, bukan delayed)
    } else {
      freq_s = 880.0f; freq_e = 440.0f;
    }

    float full_phase_ms = steps_in_phase * T_avg;
    if (full_phase_ms < 100.0f) full_phase_ms = 100.0f;
    float tone_ms = full_phase_ms * 0.8f;
    if (tone_ms > 1000.0f) tone_ms = 1000.0f;
    if (tone_ms < 150.0f)  tone_ms = 150.0f;

    new_freq_start      = freq_s;
    new_freq_end        = freq_e;
    new_phase_duration  = tone_ms;
    new_sound_requested = true;

    if (recording_active) {
      breathSensor.triggerGuide(start_inhale ? 1 : -1, full_phase_ms);
    }
  }
}

// ==========================================
// STEP ANALYSIS
// ==========================================
void handleStepAnalysis() {
  if (is_shutting_down) return;

  unsigned long t_now    = millis();
  long delta_t_inst      = t_now - t_last_step;
  t_last_step            = t_now;
  // plot_step_trigger DIPINDAHKAN ke bawah — hanya fire untuk step yang
  // ACCEPTED oleh pattern logic. Ini bikin visual count match dengan
  // pattern logic (3 spike di HIRUP, 2 spike di BUANG — tidak ada extra
  // dari false-positive MPU6050 atau irregular interval).

  if (delta_t_inst < 200 || delta_t_inst > 2000) return;

  if (!sma_initialized) {
    for (int i = 0; i < SMA_BUFFER_SIZE; i++) step_intervals[i] = delta_t_inst;
    sma_initialized = true;
  }

  step_intervals[interval_idx] = delta_t_inst;
  interval_idx = (interval_idx + 1) % SMA_BUFFER_SIZE;

  long sum = 0;
  for(int i = 0; i < SMA_BUFFER_SIZE; i++) sum += step_intervals[i];
  T_avg = (float)sum / SMA_BUFFER_SIZE;

  float current_spm = 60000.0f / T_avg;

  if (current_spm < 100.0f || current_spm > 260.0f) {
    stable_count      = 0;
    missed_step_count = 0;
    algo_state        = STATE_ANALYZING;
    esp_timer_stop(audio_timer);
    plot_guide_state  = 0;
    return;
  }

  float diff  = abs((float)delta_t_inst - T_avg);
  float limit = T_avg * 0.20f;

  if (diff < limit) {
    // Step ACCEPTED — fire visual spike. Step yang irregular (diff >= limit)
    // tidak fire spike supaya visual count match dengan pattern advance.
    plot_step_trigger = true;

    missed_step_count = 0;
    if (stable_count < 10) stable_count++;

    if (stable_count > 5) {
      algo_state = STATE_GUIDING;

      if (!guiding_ever_started) {
        guiding_ever_started = true;
        recording_active     = true;
        recording_start_ms   = millis();
        logging_enabled      = true;
        // Reset pattern counter biar GUIDING mulai dari pos 0 yang clean.
        // (Walaupun di-reset juga di connection_state_changed, ada edge case
        //  kalau user pakai mode tanpa BT reconnect — defensive double-reset.)
        global_step_count = 0;
        Serial.println("RECORDING STARTED");

        uint32_t unix_now    = getCurrentUnix();
        current_session_unix = unix_now;

        // Naikkan nomor sesi, simpan ke NVS agar persisten antar reboot
        preferences.begin("b300_cfg", false);
        current_session_id = preferences.getUInt("session_id", 0) + 1;
        preferences.putUInt("session_id", current_session_id);
        preferences.end();

        if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
          if (logFile) { logFile.flush(); logFile.close(); logFile = (File)NULL; }
          File hdrFile = LittleFS.open("/run.dat", FILE_APPEND);
          if (hdrFile) {
            uint8_t hdr[9];
            hdr[0] = 0xFF;  // magic byte: penanda awal sesi
            hdr[1] = (current_session_id >> 24) & 0xFF;
            hdr[2] = (current_session_id >> 16) & 0xFF;
            hdr[3] = (current_session_id >> 8)  & 0xFF;
            hdr[4] =  current_session_id        & 0xFF;
            hdr[5] = (unix_now >> 24) & 0xFF;
            hdr[6] = (unix_now >> 16) & 0xFF;
            hdr[7] = (unix_now >> 8)  & 0xFF;
            hdr[8] =  unix_now        & 0xFF;
            hdrFile.write(hdr, 9);
            hdrFile.flush();
            hdrFile.close();
            Serial.printf("[HDR] Sesi #%u dimulai, unix=%u\n", current_session_id, unix_now);
          } else {
            Serial.println("[HDR] GAGAL buka file untuk tulis SESSION header!");
          }
          xSemaphoreGive(logMutex);
        }

        char tbuf[17];
        if (time_is_valid) {
          unixToString(unix_now, tbuf);
          Serial.printf("Sesi dimulai: %s WIB\n", tbuf);
        } else {
          Serial.println("PERINGATAN: Waktu belum valid, unix_time = 0");
        }
      } else if (!recording_active && !is_pattern_switching) {
        // RESUME: panduan sempat PAUSE (>3 langkah meleset → ANALYZING) lalu
        // ritme stabil lagi. Tanpa blok ini, recording_active mati PERMANEN
        // setelah pause pertama walau audio tetap jalan → data sesi hilang
        // (gejala: lari 30 mnt tapi cuma ~5 mnt terekam).
        // Lanjut di sesi yang SAMA: JANGAN tulis header baru, JANGAN reset
        // recording_start_ms (timestamp tetap kontinu; jeda = waktu pause asli).
        recording_active = true;
        logging_enabled  = true;
        Serial.println("RECORDING RESUMED");
      }

      // === PENJADWALAN ADAPTIF (1 atau 2 langkah ke depan) ===
      // Ambang pakai comp 1-langkah (338ms ↔ ~177.5 SPM) = titik 1-langkah tak muat.
      // 2-langkah aktif di zona aman ~187 SPM (margin timer cukup); histeresis
      // comp1-18..comp1-8 (≈183-187 SPM) cegah flapping. Di 177-187 tetap 1-langkah.
      long comp1 = (long)(T_LEAD_MS + L_SYS_1);   // 338ms
      if      ((long)T_avg < comp1 - 18) sched_steps_ahead = 2;  // ~>187 SPM: 2-langkah
      else if ((long)T_avg > comp1 - 8)  sched_steps_ahead = 1;  // ~<183 SPM: 1-langkah

      // comp aktual per-mode (2-langkah pakai L_SYS_2 yang 10ms lebih kecil).
      long comp = (long)(T_LEAD_MS + (sched_steps_ahead == 2 ? L_SYS_2 : L_SYS_1));

      // Pattern position: cue di-schedule N langkah ke depan supaya sampai telinga
      // T_LEAD_MS SEBELUM step target (spek B200 §2.1.5). N = sched_steps_ahead.
      int this_step_pos = global_step_count % active_pattern_divisor;
      scheduled_pattern_pos = (this_step_pos + sched_steps_ahead) % active_pattern_divisor;
      global_step_count++;

      // ── ARM TIMER paling awal (SEBELUM logging) supaya file I/O tidak menggeser jadwal ──
      long time_to_trigger = (long)sched_steps_ahead * (long)T_avg - comp;
      if (time_to_trigger < 10) time_to_trigger = 10;
      esp_timer_stop(audio_timer);
      esp_timer_start_once(audio_timer, time_to_trigger * 1000);

      // Visualisasi serial plotter (align dengan actual step detection).
      if (active_pattern_divisor == 5) {
        if (this_step_pos == 0) plot_guide_state = 200;       // HIRUP start
        else if (this_step_pos == 3) plot_guide_state = 100;  // BUANG start
      } else if (active_pattern_divisor == 3) {
        if (this_step_pos == 0) plot_guide_state = 200;       // HIRUP start
        else if (this_step_pos == 2) plot_guide_state = 100;  // BUANG start
      }

      // Logging CSV (file I/O = paling lambat → PALING AKHIR).
      if (recording_active) {
        int8_t current_breath_phase;
#ifdef TESTING_BREATH
        current_breath_phase = getCurrentSimulatedBreathState();
#else
        current_breath_phase = breathSensor.getCurrentPhase();
#endif
        writeLogEntry(
          (uint32_t)(millis() - recording_start_ms),
          current_breath_phase,
          stepSensor.getSPM()
        );
      }
    }

  } else {
    if (algo_state == STATE_GUIDING) {
      missed_step_count++;
      if (missed_step_count <= 3) {
        // Re-arm pakai mode penjadwalan terkini (1 atau 2 langkah), comp per-mode.
        long comp = (long)(T_LEAD_MS + (sched_steps_ahead == 2 ? L_SYS_2 : L_SYS_1));
        long time_to_trigger = (long)sched_steps_ahead * (long)T_avg - comp;
        if (time_to_trigger < 10) time_to_trigger = 10;
        esp_timer_stop(audio_timer);
        esp_timer_start_once(audio_timer, time_to_trigger * 1000);
      } else {
        stable_count      = 0;
        missed_step_count = 0;
        algo_state        = STATE_ANALYZING;
        esp_timer_stop(audio_timer);
        plot_guide_state  = 0;

        if (recording_active && !is_pattern_switching) {
          recording_active = false;
          flushLogBuffer();
          Serial.println("RECORDING PAUSED");
          updateMonitorDisplay();  // force refresh → user langsung lihat "ANALYZING"
        }
      }
    } else {
      stable_count     = 0;
      algo_state       = STATE_ANALYZING;
      esp_timer_stop(audio_timer);
      plot_guide_state = 0;
    }
  }
}

// ==========================================
// CONNECTION HELPER
// ==========================================
void startBluetoothScan() {
  current_state     = STATE_SCANNING;
  trigger_hard_scan = false;

  is_initial_connect_attempt = false;
  has_shown_not_found        = true;

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0); display.print("PREPARING...");
  safeDisplayUpdate();

  esp_timer_stop(audio_timer);
  is_playing_sound = false;
  algo_state       = STATE_ANALYZING;
  a2dp_source.set_data_callback(nullptr);
  a2dp_source.set_auto_reconnect(false);

  if (a2dp_active) {
      if (a2dp_source.is_connected()) {
          a2dp_source.disconnect();
          delay(300);
      }
      a2dp_active = false;
      delay(300);
  }
  // ← TAMBAH: selalu end() agar state library bersih, apapun riwayatnya
  // a2dp_source.end();
  // delay(500);
  memset(current_mac, 0, 6);
  is_mac_configured          = false;
  found_devices.clear();
  selected_device_index      = 0;
  is_initial_connect_attempt = false;
  has_shown_not_found        = false;
  scan_start_time            = millis();

  a2dp_source.set_data_callback(get_sound_data);
  a2dp_source.set_ssid_callback(target_filter_callback);

  display.clearDisplay();
  display.setCursor(0,0); display.print("SCANNING...");
  safeDisplayUpdate();

  ensureBTControllerBTDM();
  a2dp_source.start(std::vector<const char*>());
  a2dp_active = true;
}

// ── Serial Plot untuk Excel Data Streamer ──
// Format: timestamp_ms, Step, GuideState, SPM
// Output CSV murni — bisa langsung dibaca Excel Data Streamer.
// Header dikirim SEKALI saat GUIDING pertama supaya Excel kenali kolom.
void debugSerialPlotter() {
  static bool header_sent = false;

  static unsigned long last_plot = 0;
  if (millis() - last_plot < 30) return;   // 30ms = ~33Hz → cukup rapat utk grafik halus
  last_plot = millis();

  // STEP sebagai PULSE EVENT diskrit
  static unsigned long step_pulse_until = 0;
  if (plot_step_trigger) { plot_step_trigger = false; step_pulse_until = millis() + 60; }
  int step_val = (millis() < step_pulse_until) ? 300 : 0;

  // Kirim header SEKALI di awal GUIDING
  if (!header_sent && algo_state == STATE_GUIDING) {
    header_sent = true;
    Serial.println("Timestamp,Step,GuideState,SPM");
  }

  Serial.printf("%lu,%d,%d,%d\n",
                millis(),
                step_val,
                plot_guide_state,
                (int)stepSensor.getSPM());
}

// ==========================================
// LOG HELPERS (FORMAT TEKS CSV)
// ==========================================

/*
/*
 * writeLogEntry — tulis 1 baris CSV ringkas ke file.
 *
 * Kolom (sesuai definisi aplikasi HP):
 *   breathPhase : 1=tarik napas, -1=buang napas, 0=tidak ada napas
 *   step        : 1=ada langkah, 0=tidak ada langkah
 *   spm         : langkah/menit (0 jika baris napas murni)
 *   patternID   : 0=pola 3:2, 1=pola 2:1
 *
 * Format di flash (ringkas, tanpa tanggal):
 *   sesi_id,ts_ms,breathPhase,step,spm,patternID;
 * Tanggal disisipkan saat sync, bukan di sini.
 */
/*
 * writeLogEntry — tulis 1 baris CSV ke file.
 *
 * Dual-event schema:
 *  - is_step_event = true  → row STEP: step=1, spm=current SPM, breathPhase=current state
 *  - is_step_event = false → row BREATH TRANSITION: step=0, spm=0, breathPhase=new state
 *
 * HP merekonstruksi continuous state dari sequence event-event ini.
 */
// Ganti writeLogEntry yang lama
void writeLogEntry(uint32_t ts_ms, int8_t breathPhase, float spm_val, bool is_step_event) {
    if (!logging_enabled) return;

    uint8_t flags = 0;
    if (is_step_event) flags |= 0x01;
    // breathPhase: 1=inhale→01, -1=exhale→10, 0=none→00
    if (breathPhase == 1)  flags |= (0x01 << 1);
    if (breathPhase == -1) flags |= (0x02 << 1);
    if (active_pattern_divisor != 5) flags |= (0x01 << 3);

    uint8_t spm_byte = (uint8_t)constrain((int)round(spm_val), 0, 255);

    uint8_t rec[6];
    rec[0] = (ts_ms >> 24) & 0xFF;
    rec[1] = (ts_ms >> 16) & 0xFF;
    rec[2] = (ts_ms >> 8)  & 0xFF;
    rec[3] =  ts_ms        & 0xFF;
    rec[4] = flags;
    rec[5] = spm_byte;

    if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (!logFile) logFile = LittleFS.open("/run.dat", FILE_APPEND);
        if (logFile) logFile.write(rec, 6);
        xSemaphoreGive(logMutex);
    }
}

/*
 * flushLogBuffer — flush & tutup file agar data tersimpan ke flash.
 * Dipanggil saat recording pause, deep sleep, dan sebelum sync.
 */
void flushLogBuffer() {
  if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    if (logFile) {
      logFile.flush();
    }
    xSemaphoreGive(logMutex);
  }
}

/*
 * logPatternChange — catat pergantian pola ke Serial.
 * Tidak perlu tulis baris khusus ke file — kolom patternID di baris
 * langkah/napas berikutnya otomatis mencerminkan pola baru karena
 * active_pattern_divisor sudah berubah saat writeLogEntry dipanggil.
 */
void logPatternChange(int new_divisor) {
  if (!recording_active) return;
  Serial.printf("[PATTERN] Ganti pola → %s, patternID=%d akan berlaku mulai baris berikutnya\n",
                new_divisor == 5 ? "3:2" : "2:1",
                new_divisor == 5 ? 0 : 1);
}

void toggleRunningPattern() {
  is_pattern_switching = true;

  if (active_pattern_divisor == 5) {
    active_pattern_divisor = 3;
    logPatternChange(3);
    updateDisplay("SWAPPED!!", "MODE: 2:1 (POWER)", true);
  } else {
    active_pattern_divisor = 5;
    logPatternChange(5);
    updateDisplay("SWAPPED!!", "MODE: 3:2 (RELAX)", true);
  }

  // Ganti delay(1000) dengan non-blocking agar watchdog tidak kelaparan
  unsigned long t = millis();
  while (millis() - t < 1000) {
    vTaskDelay(1);  // beri waktu ke FreeRTOS scheduler & watchdog
  }

  is_pattern_switching = false;  // ← WAJIB direset!
}

// ==========================================
// SYNC CALLBACKS (Flutter-compatible)
// ==========================================
class SyncCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) override {
    String val = pChar->getValue();
    Serial.printf("[BLE RX] raw: '%s' len=%d\n", val.c_str(), val.length()); // ← tambah ini

    // ── Terima waktu dari Flutter ──
    if (val.startsWith("TIME:")) {
      uint32_t ts = (uint32_t)val.substring(5).toInt();
      if (ts > 1000000000UL) {
        saved_unix_time   = ts;
        millis_at_sync    = millis();
        time_is_valid     = true;
        rtc_unix_at_sleep = ts;
        rtc_us_at_sleep   = rtcTicksToUs(rtc_time_get());
        rtc_time_valid    = true;

        // Simpan ke NVS bersama RTC counter — supaya kalau user batal sync
        // dan restart, recovery di boot berikutnya bisa hitung elapsed time
        // dengan akurat (RTC_DATA_ATTR sendiri TIDAK survive ESP.restart()
        // di Arduino ESP32 — bootloader reload .rtc.data dari flash).
        uint64_t ticks_now = rtc_time_get();
        preferences.begin("b300_cfg", false);
        preferences.putUInt("unix_time", ts);
        preferences.putULong64("rtc_ticks", ticks_now);
        preferences.end();

        Serial.printf("Waktu diperbarui dari HP: %u (NVS+ticks tersimpan)\n", ts);
        char tbuf[17];
        unixToString(ts, tbuf);
        Serial.printf("Waktu lokal: %s WIB\n", tbuf);
      }
      return;
    }

    // ── Flutter kirim "SYNC" → mulai kirim data ──
    // (Sesuai protokol Flutter: trigger kata "SYNC", bukan "START")
    if (val == "SYNC" && !isSyncing && !ble_deinit_done) {
      isSyncing = true;
      Serial.println("[SYNC] SYNC diterima dari Flutter, siap kirim data...");
      return;
    }

    // ── Flutter kirim ACK setelah berhasil parse semua data ──
    // Terima beberapa kata sebagai konfirmasi (HP dev bebas pilih)
    if (val == "OK" || val == "DONE" || val == "ACK" || val == "SELESAI" || val == "OKE") {
      sync_completed_ack = true;
      Serial.printf("[SYNC] ACK '%s' diterima dari HP — aman delete file & restart\n", val.c_str());
      return;
    }
  }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("[BLE] HP Berhasil Terhubung!");
    }

    void onDisconnect(BLEServer* pServer) {
      Serial.println("[BLE] HP Terputus (Disconnected)!");
      // Restart advertising agar HP bisa mencoba connect lagi
      BLEDevice::startAdvertising();
    }
};
// startSyncMode() LAMA: dihapus — diganti dengan requestBootMode(BOOT_SYNC).
// Logika BLE init pindah ke initBLEForSync() yang dipanggil dari setup() saat
// boot_mode = BOOT_SYNC. Single-mode-per-boot menghilangkan transisi BTDM yang crash.

// ==========================================
// DEEP SLEEP
// ==========================================
// enterDeepSleep — RESTART-TO-SLEEP pattern (avoid A2DP end() crash)
//
// Sebelumnya kita coba shutdown A2DP+BLE inline lalu esp_deep_sleep_start().
// Tapi BluetoothA2DPSource::end() crash intermittently saat audio task masih
// proses buffer (use-after-free) → ESP_RST_PANIC saat user mau tidur.
//
// Solusi: pola yang sama dengan requestBootMode — save state, ESP.restart()
// dengan boot_mode=BOOT_DEEP_SLEEP. Di setup(), BTDM tidak pernah di-init,
// langsung clean path ke esp_deep_sleep_start. Tidak ada BT cleanup = tidak
// ada race condition = tidak ada crash.
void enterDeepSleep() {
  if (restart_pending) return;

  restart_pending      = true;
  is_shutting_down     = true;
  recording_active     = false;   // SensorTask cek flag ini sebelum writeLogEntry
  guiding_ever_started = false;
  esp_timer_stop(audio_timer);
  is_playing_sound     = false;
  new_sound_requested  = false;
  digitalWrite(PIN_LED, LOW);

  // Stop A2DP callback supaya audio task tidak request data lagi.
  // PENTING: jangan panggil a2dp_source.end() — itu yang crash.
  // Restart akan reset semua BTDM state secara hardware.
  if (a2dp_active) {
    a2dp_source.set_data_callback(nullptr);
    a2dp_source.set_auto_reconnect(false);
  }

  // Tampilkan pesan "Mematikan..." dengan animasi zZz biar user tahu
  // proses sedang berjalan (bukan crash). 800ms total — cukup buat user
  // baca + lepas tombol.
  const char zzz_frames[3][8] = {"z..", "zZ.", "zZZ"};
  for (int i = 0; i < 8; i++) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(28, 8);
    display.println("Mematikan");
    display.setTextSize(2);
    display.setCursor(40, 26);
    display.println(zzz_frames[i % 3]);
    display.setTextSize(1);
    display.setCursor(8, 52);
    display.println("Lepas tombol...");
    if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      display.display();
      xSemaphoreGive(wireMutex);
    } else {
      display.display();
    }
    delay(100);
  }

  // Tunggu tombol dilepas (jangan masuk loop boot dengan OK tertekan)
  unsigned long start_wait = millis();
  while (digitalRead(PIN_BTN_OK) == LOW) {
    delay(10);
    yield();
    if (millis() - start_wait > 8000) break;
  }

  // Flush & close log file — pakai logMutex untuk sync dengan SensorTask
  // yang mungkin masih mid-write. Wait tanpa timeout supaya in-flight write
  // selesai dulu (max ~10ms karena writeLogEntry single-line).
  if (xSemaphoreTake(logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (logFile) {
      logFile.flush();
      logFile.close();
      logFile = (File)NULL;
    }
    xSemaphoreGive(logMutex);
  }

  // Simpan waktu ke RTC memory (PATH 1 wake — survive deep sleep)
  if (time_is_valid) {
    rtc_unix_at_sleep = getCurrentUnix();
    rtc_us_at_sleep   = rtcTicksToUs(rtc_time_get());
    rtc_time_valid    = true;
    Serial.printf("[SLEEP] Save time to RTC mem: unix=%u\n", rtc_unix_at_sleep);
  }

  // Simpan boot mode + time ke NVS (untuk setup() handler)
  preferences.begin("b300_cfg", false);
  preferences.putUChar("boot_mode", BOOT_DEEP_SLEEP);
  preferences.putBool("mode_switch", true);  // skip splash di setup()
  if (time_is_valid) {
    uint32_t now_unix  = getCurrentUnix();
    uint64_t now_ticks = rtc_time_get();
    preferences.putUInt("unix_time", now_unix);
    preferences.putULong64("rtc_ticks", now_ticks);
  }
  preferences.end();

  Serial.println("[SLEEP] Restart utk clean deep sleep path...");
  Serial.flush();
  ESP.restart();
}

// ==========================================
// INPUT HANDLING
// ==========================================
void handleInput() {
  unsigned long now = millis();

  if (now < ignore_btn_until) {
    // Tetap izinkan pengecekan deep-sleep meski dalam periode ignore
    bool _ok = digitalRead(PIN_BTN_OK);
    if (_ok == LOW && btn_ok_press_start > 0 && now - btn_ok_press_start > SLEEP_PRESS_MS) {
        enterDeepSleep();
    }
    btn_ok_last_state = _ok;
    return;
  }

  bool btn_ok_read   = digitalRead(PIN_BTN_OK);
  bool btn_up_read   = digitalRead(PIN_BTN_UP);
  bool btn_down_read = digitalRead(PIN_BTN_DOWN);

  if (btn_up_read == LOW) {
     if (now - last_btn_press_time > DEBOUNCE_DELAY) {
        last_btn_press_time = now;
        if (current_state == STATE_IDLE && a2dp_source.is_connected()) {
            current_volume = (current_volume >= 120) ? 127 : current_volume + 5;
            a2dp_source.set_volume(current_volume);
        } else if (current_state == STATE_MAIN_MENU) {
            if (main_menu_index > 0) main_menu_index--;
            showMainMenu();
        } else if (current_state == STATE_SELECTING && !found_devices.empty()) {
            if (selected_device_index > 0) selected_device_index--;
            showSelectionScreen();
        } else if (current_state == STATE_MENU_PATTERN) {
            selected_rhythm_index = 0; showRhythmMenu();
        }
     }
  }

  if (btn_down_read == LOW) {
     if (now - last_btn_press_time > DEBOUNCE_DELAY) {
        last_btn_press_time = now;
        if (current_state == STATE_IDLE && a2dp_source.is_connected()) {
            current_volume = (current_volume <= 5) ? 0 : current_volume - 5;
            a2dp_source.set_volume(current_volume);
        } else if (current_state == STATE_MAIN_MENU) {
            if (main_menu_index < 2) main_menu_index++;
            showMainMenu();
        } else if (current_state == STATE_SELECTING && !found_devices.empty()) {
            if (selected_device_index < (int)found_devices.size() - 1) selected_device_index++;
            showSelectionScreen();
        } else if (current_state == STATE_MENU_PATTERN) {
            selected_rhythm_index = 1; showRhythmMenu();
        }
     }
  }

  if (btn_ok_read == LOW && btn_ok_last_state == HIGH) {
     btn_ok_press_start   = now;
     btn_ok_long_detected = false;
  }

  if (btn_ok_read == LOW) {
    if (now - btn_ok_press_start > SLEEP_PRESS_MS) {
        enterDeepSleep();
    } else if (!btn_ok_long_detected && now - btn_ok_press_start > LONG_PRESS_MS) {
        btn_ok_long_detected = true;
        if (current_state == STATE_IDLE) {
            current_state   = STATE_MAIN_MENU;
            main_menu_index = 0;
            showMainMenu();
            ignore_btn_until = now + 1000;
        } else if (current_state == STATE_SYNC) {
            // ── BATAL SYNC: notify HP (best-effort) lalu restart ke AUDIO ──
            if (pCharacteristic && pServer && pServer->getConnectedCount() > 0) {
                pCharacteristic->setValue("DISCONNECT");
                pCharacteristic->notify();
                Serial.println("[BLE] Notify DISCONNECT terkirim ke HP");
                delay(300);
            }
            requestBootMode(BOOT_AUDIO, "User batal sync");
            return;
        } else if (current_state == STATE_SELECTING && !found_devices.empty()) {
            FoundDevice &target = found_devices[selected_device_index];
            if (target.is_cmd_rescan) {
                trigger_hard_scan = true;
                ignore_btn_until  = now + 1000;
            }
        }
    }
  }

  if (btn_ok_read == HIGH && btn_ok_last_state == LOW) {
     if (!btn_ok_long_detected && now - last_btn_press_time > 100) {
        if (current_state == STATE_IDLE && a2dp_source.is_connected()) {
            toggleRunningPattern();
        } else if (current_state == STATE_MAIN_MENU) {
            if (main_menu_index == 0) trigger_hard_scan = true;
            else if (main_menu_index == 1) {
              // ── SYNC MODE: konfirmasi dulu kalau ada recording aktif ──
              if (recording_active) {
                updateDisplay("RECORDING AKTIF",
                              "Data lari akan\ndisimpan.\nOK lagi=Sync",
                              false);
                delay(1500);
                // Tunggu OK lagi atau UP/DOWN untuk batal
                unsigned long confirm_start = millis();
                bool confirmed = false;
                while (millis() - confirm_start < 5000) {
                  if (digitalRead(PIN_BTN_OK) == LOW) {
                    delay(50);
                    if (digitalRead(PIN_BTN_OK) == LOW) { confirmed = true; break; }
                  }
                  if (digitalRead(PIN_BTN_UP) == LOW || digitalRead(PIN_BTN_DOWN) == LOW) {
                    showMainMenu();
                    return;
                  }
                  delay(20);
                  yield();
                }
                if (!confirmed) {
                  showMainMenu();
                  return;
                }
              }
              // Trigger restart ke BOOT_SYNC mode (waktu tetap persisten via RTC mem)
              requestBootMode(BOOT_SYNC, "User pilih SYNC");
              return;  // tidak akan sampai sini
            }
            else if (main_menu_index == 2) {
                if (a2dp_source.is_connected()) {
                    current_state = STATE_IDLE;
                    updateDisplay("RESUMING...", "Back to Guide");
                    delay(1000);
                } else {
                    updateDisplay("CANNOT EXIT", "No Connection!", true);
                    delay(1000);
                    showMainMenu();
                }
            }
        } else if (current_state == STATE_SELECTING && !found_devices.empty()) {
            FoundDevice &target = found_devices[selected_device_index];
            if (target.is_cmd_rescan) {
                trigger_hard_scan = true;
            } else {
                preferences.begin("b300_cfg", false);
                preferences.putBytes("target_mac", target.address, 6);
                preferences.end();
                memcpy(current_mac, target.address, 6);
                is_mac_configured = true;
                current_state     = STATE_PAIRING;
                updateDisplay("Connecting...", "Please Wait", true);
                a2dp_source.set_auto_reconnect(true);
                a2dp_source.start((std::vector<const char*>){{}});
                a2dp_active = true;
            }
        } else if (current_state == STATE_MENU_PATTERN) {
            // SAFETY: cek koneksi earphone sebelum transisi ke STATE_IDLE.
            // Kalau earphone mati saat user lagi di menu pola, jangan lanjut —
            // STATE_IDLE tanpa A2DP = audio guidance mati & user nyangka stuck.
            if (!a2dp_source.is_connected()) {
                updateDisplay("EARPHONE LEPAS!", "Reconnect dulu", false);
                delay(2000);
                current_state              = STATE_MAIN_MENU;
                main_menu_index            = 0;
                is_initial_connect_attempt = false;
                has_shown_not_found        = true;
                showMainMenu();
                return;
            }
            active_pattern_divisor = (selected_rhythm_index == 0) ? 5 : 3;
            updateDisplay("READY", "GO RUN!", true);
            delay(1500);
            current_state = STATE_IDLE;
            digitalWrite(PIN_LED, HIGH);
        }
        last_btn_press_time = now;
     }
  }
  btn_ok_last_state = btn_ok_read;
}

// ==========================================
// SETUP & LOOP
// ==========================================
void initAudioTimer() {
  const esp_timer_create_args_t timer_args = {
    .callback = &audioTimerCallback,
    .name     = "audio_guide_timer"
  };
  esp_timer_create(&timer_args, &audio_timer);
}

bool target_filter_callback(const char *ssid, esp_bd_addr_t address, int rssi) {
  if (current_state == STATE_SCANNING) {
    // Pastikan ssid tidak null dan tidak kosong, jika kosong beri label sementara
    String discovered_name = (ssid != nullptr && strlen(ssid) > 0) ? String(ssid) : "Unnamed";

    // 1. Cek apakah alamat MAC ini sudah ada di dalam daftar
    for (auto &d : found_devices) {
      if (memcmp(d.address, address, 6) == 0) {
        // Jika sudah ada, TAPI namanya sebelumnya kosong/"Unnamed", 
        // dan sekarang nama aslinya sudah terbaca, maka UPDATE namanya!
        if ((d.name == "Unnamed" || d.name == "") && discovered_name != "Unnamed") {
            d.name = discovered_name;
        }
        return false; // Jangan tambahkan perangkat ganda
      }
    }

    // 2. Jika belum ada di list sama sekali, tambahkan baru
    FoundDevice newDev;
    newDev.name          = discovered_name;
    memcpy(newDev.address, address, 6);
    newDev.is_cmd_rescan = false;
    found_devices.push_back(newDev);
    return false;
  }
  
  if (is_mac_configured && current_state != STATE_SELECTING) {
      return (memcmp(address, current_mac, 6) == 0);
  }
  return false;
}

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
  if (state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    if (current_state == STATE_SCANNING)  return;
    if (current_state == STATE_MAIN_MENU) return;

    guiding_ever_started = false;
    recording_active     = false;
    algo_state           = STATE_ANALYZING;
    stable_count         = 0;
    missed_step_count    = 0;
    global_step_count    = 0;
    sma_initialized      = false;
    Serial.println("[CONN] State recording di-reset untuk sesi baru");

    current_state         = STATE_MENU_PATTERN;
    selected_rhythm_index = 0;
    showRhythmMenu();
    is_initial_connect_attempt = false;
  }
}

void SensorTask(void *pvParameters) {
   for(;;) {
     stepSensor.update();
     breathSensor.update();
     vTaskDelay(1);
   }
}

// ==========================================
// BOOT MODE HELPERS
// ==========================================

// Baca boot mode dari NVS dengan sanity check (cegah corrupt value)
BootMode readBootMode() {
  preferences.begin("b300_cfg", true);
  uint8_t raw = preferences.getUChar("boot_mode", BOOT_AUDIO);
  preferences.end();
  // Sanity check: kalau NVS corrupt (mis. brownout mid-write), fallback ke AUDIO
  return (raw <= BOOT_DEEP_SLEEP) ? (BootMode)raw : BOOT_AUDIO;
}

// Request mode baru lalu ESP.restart() — single-mode-per-boot architecture
// Waktu di-save ke RTC memory (survive soft reset) sebelum restart
void requestBootMode(BootMode mode, const char* reason) {
  if (restart_pending) return;  // anti double-trigger

  restart_pending  = true;
  is_shutting_down = true;      // hentikan audio callback & sensor handling

  // ── Tampilan transisi mulus (bukan kesan "reset") ──
  // Header "Mempersiapkan" + nama mode besar + spinner berputar.
  // Spinner ngasih kesan loading aktif (bukan crash).
  const char* mode_name = (mode == BOOT_SYNC) ? "SYNC" : "AUDIO";
  const char  spinner_frames[4] = {'|', '/', '-', '\\'};

  // Frame statis (header + mode name) di-draw sekali, spinner di-update tiap 100ms.
  for (int i = 0; i < 15; i++) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(16, 6);
    display.println("Mempersiapkan");

    // Mode name (large, centered)
    display.setTextSize(2);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(mode_name, 0, 0, &x1, &y1, &w, &h);
    display.setCursor((SCREEN_WIDTH - (int)w) / 2, 24);
    display.println(mode_name);

    // Spinner [X] di tengah-bawah
    display.setTextSize(1);
    display.setCursor(56, 52);
    display.print('[');
    display.print(spinner_frames[i % 4]);
    display.print(']');

    if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      display.display();
      xSemaphoreGive(wireMutex);
    } else {
      display.display();
    }
    delay(100);
  }
  // Total ~1.5s — sama dengan delay lama, tapi terasa active

  // 1. Simpan waktu ke RTC memory (pattern sama persis dengan enterDeepSleep)
  if (time_is_valid) {
    rtc_unix_at_sleep = getCurrentUnix();
    rtc_us_at_sleep   = rtcTicksToUs(rtc_time_get());
    rtc_time_valid    = true;
    Serial.printf("[RESTART] Simpan waktu: unix=%u\n", rtc_unix_at_sleep);
  }

  // 2. Simpan boot mode + waktu ke NVS
  // PENTING: RTC_DATA_ATTR di-RESET saat ESP.restart() di ESP32 Arduino
  // (bootloader reload .rtc.data dari flash, jadi initializer = false dieksekusi ulang).
  // Solusinya: simpan unix_time DAN RTC counter ke NVS. RTC counter (hardware
  // register, bukan memory) BENERAN persisten lewat soft reset.
  preferences.begin("b300_cfg", false);
  preferences.putUChar("boot_mode", (uint8_t)mode);
  // Flag transisi-terencana → di setup() bisa skip splash & tampilkan
  // "Mempersiapkan MODE X" langsung biar transisi mulus (bukan kayak reset).
  preferences.putBool("mode_switch", true);
  if (time_is_valid) {
    uint32_t now_unix  = getCurrentUnix();
    uint64_t now_ticks = rtc_time_get();
    preferences.putUInt("unix_time", now_unix);
    preferences.putULong64("rtc_ticks", now_ticks);
    Serial.printf("[RESTART] NVS save: unix=%u, ticks=%llu\n", now_unix, now_ticks);
  }
  preferences.end();

  // 3. Flush log file (cegah data lari hilang)
  flushLogBuffer();
  if (logFile) {
    logFile.flush();
    logFile.close();
    logFile = (File)NULL;
  }

  // 4. BLE cleanup — deinit sebelum restart, cegah ipc0 stack overflow
  //    (BLE stack + BTDM berebut stack Core 0 saat restart cepat).
  if (ble_initialized) {
    BLEDevice::deinit(true);
    ble_initialized = false;
    delay(100);
  }

  // 5. Best-effort BT cleanup — JANGAN paksa, mode baru bakal fresh init.
  //    Skip kalau ada risiko crash (state library tidak stable)
  if (a2dp_active) {
    a2dp_source.set_data_callback(nullptr);
    a2dp_source.set_auto_reconnect(false);
  }

  Serial.printf("[RESTART] boot_mode=%d, reason=%s\n", (int)mode, reason);
  Serial.flush();
  // Delay sebelum restart sudah 1500ms di atas, tidak perlu tambah lagi
  ESP.restart();
}

// I2C bus recovery — toggle SCL 9x kalau SDA stuck LOW (sensor hang)
void recoverI2CBus(uint8_t sda_pin, uint8_t scl_pin) {
  pinMode(scl_pin, OUTPUT);
  pinMode(sda_pin, INPUT_PULLUP);
  digitalWrite(scl_pin, HIGH);
  delayMicroseconds(10);

  if (digitalRead(sda_pin) == LOW) {
    Serial.println("[I2C] SDA stuck LOW, mulai recovery...");
    for (int i = 0; i < 9; i++) {
      digitalWrite(scl_pin, LOW);  delayMicroseconds(5);
      digitalWrite(scl_pin, HIGH); delayMicroseconds(5);
      if (digitalRead(sda_pin) == HIGH) break;
    }
    // STOP condition: SDA LOW->HIGH while SCL HIGH
    pinMode(sda_pin, OUTPUT);
    digitalWrite(sda_pin, LOW);  delayMicroseconds(5);
    digitalWrite(sda_pin, HIGH); delayMicroseconds(5);
    Serial.println("[I2C] Recovery selesai");
  }
}

// Safe mode: dipanggil saat boot_attempt > 3 (3x crash beruntun)
// Reset NVS boot_mode ke AUDIO supaya next restart fresh, tunggu user cabut batrai
void enterSafeMode(const char* msg) {
  Serial.printf("[SAFE MODE] %s\n", msg);
  // Coba init display minimal
  Wire.begin();
  Wire.setClock(100000);  // turunkan clock untuk reliability di safe mode
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("!! SAFE MODE !!");
    display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);
    display.setCursor(0, 18);
    display.println(msg);
    display.setCursor(0, 45);
    display.println("Cabut batrai");
    display.println("untuk reset total");
    display.display();
  }
  // Reset boot state supaya next manual power-on = clean AUDIO mode
  preferences.begin("b300_cfg", false);
  preferences.putUChar("boot_mode", BOOT_AUDIO);
  preferences.putUChar("boot_attempt", 0);  // reset counter di NVS
  preferences.end();

  // Tunggu user cabut batrai (atau cabut+pasang manual)
  while (true) {
    delay(1000);
    yield();
  }
}

// Init BLE service untuk SYNC mode — dipanggil dari setup() saat boot_mode=SYNC
void initBLEForSync() {
  ensureBTControllerBTDM();

  BLEDevice::init("Cheaststrap");
  pServer = BLEDevice::createServer();
  if (!pServer) {
    Serial.println("[BLE] createServer GAGAL!");
    requestBootMode(BOOT_AUDIO, "BLE init fail");
    return;
  }
  pServer->setCallbacks(new MyServerCallbacks());
  ble_initialized = true;

  // ── Battery Service standar BLE (UUID 0x180F / 0x2A19) ──
  // Flutter bisa baca/subscribe level baterai secara real-time via notify.
  BLEService *pBattService = pServer->createService(BLEUUID((uint16_t)0x180F));
  pBattCharacteristic = pBattService->createCharacteristic(
      BLEUUID((uint16_t)0x2A19),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pBattCharacteristic->addDescriptor(new BLE2902());
  battery.update();
  int rawLevel = battery.getLevel();
  uint8_t battLevel = (uint8_t)(rawLevel < 0 ? 0 : rawLevel > 100 ? 100 : rawLevel);
  pBattCharacteristic->setValue(&battLevel, 1);
  pBattService->start();

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->setCallbacks(new SyncCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLEUUID((uint16_t)0x180F)); // Battery Service
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  current_state = STATE_SYNC;
  updateDisplay("SYNC MODE", "Waiting for phone...");
  Serial.println("[BLE] SYNC mode siap, advertising aktif");
}

// ==========================================
// FUNGSI STRESS TEST MEMORI (B200 - Spek 8)
// ==========================================
// void generateDummyData10SesiBinary() {
//   Serial.println("[DUMMY] Mulai menulis 10 sesi biner...");
//   LittleFS.format();
//   Serial.println("Memori diformat ulang.");

//   File f = LittleFS.open("/run.dat", FILE_WRITE);
//   if (!f) { Serial.println("Gagal buka file run.dat"); return; }

//   for (int sesi = 1; sesi <= 10; sesi++) {
//     // Tulis header sesi (9 byte)
//     uint32_t sid     = (uint32_t)sesi;
//     uint32_t unix_ts = 1700000000UL + sesi * 3600;
//     uint8_t hdr[9];
//     hdr[0] = 0xFF;
//     hdr[1] = (sid     >> 24) & 0xFF;
//     hdr[2] = (sid     >> 16) & 0xFF;
//     hdr[3] = (sid     >>  8) & 0xFF;
//     hdr[4] =  sid            & 0xFF;
//     hdr[5] = (unix_ts >> 24) & 0xFF;
//     hdr[6] = (unix_ts >> 16) & 0xFF;
//     hdr[7] = (unix_ts >>  8) & 0xFF;
//     hdr[8] =  unix_ts        & 0xFF;
//     f.write(hdr, 9);

//     // Tulis 2000 record per sesi (6 byte each)
//     for (int i = 0; i < 20000; i++) {
//       uint32_t ts_ms = (uint32_t)(i * 300);
//       uint8_t flags  = 0x03; // is_step=1, breathPhase=inhale
//       uint8_t spm    = 200;
//       uint8_t rec[6];
//       rec[0] = (ts_ms >> 24) & 0xFF;
//       rec[1] = (ts_ms >> 16) & 0xFF;
//       rec[2] = (ts_ms >>  8) & 0xFF;
//       rec[3] =  ts_ms        & 0xFF;
//       rec[4] = flags;
//       rec[5] = spm;
//       f.write(rec, 6);

//       if (i % 500 == 0) { f.flush(); vTaskDelay(2); }
//     }
//     Serial.printf("Sesi %d selesai.\n", sesi);
//   }
//   f.close();

//   File check = LittleFS.open("/run.dat", FILE_READ);
//   if (check) {
//     Serial.printf("UKURAN FILE FINAL: %u bytes\n", check.size());
//     check.close();
//   }
// }
// ==========================================
void setup() {
  Serial.begin(115200);

  // ── SAFE MODE GUARD: cegah infinite restart loop ──
  // boot_attempt disimpan di NVS (bukan RTC_DATA_ATTR yang ke-reset tiap ESP.restart).
  // Counter naik tiap boot, reset ke 0 di akhir setup() kalau boot sukses.
  // Kalau crash beruntun > 3 kali, counter tidak ter-reset → safe mode triggered.
  //
  // PENTING: Counter HANYA naik kalau boot ini hasil dari CRASH BENERAN
  // (panic, watchdog, brownout). Intentional restart dari requestBootMode()
  // (mode switch SYNC↔AUDIO) reset_reason=SW, BUKAN crash — counter di-reset
  // ke 0 supaya mode switching biasa tidak trigger safe mode false-positive.
  esp_reset_reason_t boot_reason = esp_reset_reason();
  bool is_real_crash = (boot_reason == ESP_RST_PANIC    ||
                        boot_reason == ESP_RST_INT_WDT  ||
                        boot_reason == ESP_RST_TASK_WDT ||
                        boot_reason == ESP_RST_WDT      ||
                        boot_reason == ESP_RST_BROWNOUT);

  if (is_real_crash) {
    // Boot ini hasil crash beneran — naikkan counter
    boot_attempt = loadBootAttemptFromNVS();
    boot_attempt++;
    saveBootAttemptToNVS(boot_attempt);
    Serial.printf("[BOOT] CRASH detected (reason=%d), attempt #%u\n",
                  (int)boot_reason, boot_attempt);
    if (boot_attempt > 3) {
      enterSafeMode("Crash beruntun 3x");
      // tidak akan return — safe mode loop forever
    }
  } else {
    // Boot bersih (power-on, SW restart dari requestBootMode, deep sleep wake, EXT button)
    // → reset counter, tidak ada crash di-detect.
    saveBootAttemptToNVS(0);
    boot_attempt = 0;
    Serial.printf("[BOOT] Boot normal (reason=%d), counter reset ke 0\n",
                  (int)boot_reason);
  }

  // 1. Mutex PERTAMA — sebelum apapun yang pakai Wire/I2C
  wireMutex = xSemaphoreCreateMutex();
  logMutex  = xSemaphoreCreateMutex();

  // 2. I2C recovery (in case bus stuck dari sesi sebelumnya / brownout)
  recoverI2CBus(SDA, SCL);

    // 4. Wire + Display (wireMutex sudah siap)
  Wire.begin();
  Wire.setClock(400000);
  // 2. Power management: MOSFET ON, gpio_hold release
  battery.begin(/*ledRed*/26, /*mosfet*/27, /*lowLevel%*/15, /*cutoffV*/3.2f);

  pinMode(PIN_BTN_OK, INPUT_PULLUP);

  // 3. LittleFS
  if (!LittleFS.begin(true)) {
      Serial.println("LittleFS mount failed");
  } else {
      Serial.println("LittleFS mounted OK");
      //generateDummyData10SesiBinary();
      // ==============================================
      
      size_t total = LittleFS.totalBytes();
      size_t used = LittleFS.usedBytes();
      Serial.printf("LittleFS total: %u, used: %u\n", total, used);
      File f = LittleFS.open("/run.dat", FILE_READ);
      if (!f) {
          Serial.println("FILE NOT FOUND: /run.dat");
      } else {
          size_t size = f.size();
          Serial.printf("FILE FOUND: /run.dat, size = %u bytes\n", size);
          if (size > 0) {
              Serial.println("=== Isi /run.dat ===");
              int lineCount = 0;
              while (f.available() && lineCount < 20) {
                  String line = f.readStringUntil('\n');
                  Serial.println(line);
                  lineCount++;
              }
              if (f.available()) Serial.println("... (terpotong)");
          } else {
              Serial.println("File is empty.");
          }
          f.close();
      }
  }

  // NOTE: Brownout detector DIBIARKAN AKTIF (default ESP32).
  // JANGAN disable — brownout adalah safety hardware untuk cegah NVS corrupt
  // saat batrai drop ke level kritis. Kalau false-trigger saat A2DP start,
  // solusinya tambah kapasitor 470uF di rail 3.3V, BUKAN disable detector.
  // WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // ← DIHAPUS, bahaya

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

  if (wakeup_reason != ESP_SLEEP_WAKEUP_EXT0) {
    esp_reset_reason_t reset_reason = esp_reset_reason();
    bool is_crash_restart = (reset_reason == ESP_RST_PANIC     ||
                             reset_reason == ESP_RST_INT_WDT   ||
                             reset_reason == ESP_RST_TASK_WDT  ||
                             reset_reason == ESP_RST_WDT       ||
                             reset_reason == ESP_RST_SW);

    if (is_crash_restart) {
        // Label "Crash restart" misleading — branch ini juga ke-trigger oleh
        // ESP_RST_SW (intentional restart via requestBootMode). Bedakan:
        if (reset_reason == ESP_RST_SW) {
          Serial.println("[BOOT] Software restart (intentional via requestBootMode)");
        } else if (reset_reason == ESP_RST_PANIC) {
          Serial.println("[BOOT] CRASH detected: PANIC (segfault/exception)");
        } else if (reset_reason == ESP_RST_INT_WDT || reset_reason == ESP_RST_TASK_WDT || reset_reason == ESP_RST_WDT) {
          Serial.println("[BOOT] CRASH detected: WATCHDOG timeout — code stuck di loop");
        }
    } else {
        Serial.println("Power On Reset -> Going to Sleep immediately...");
        if(display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
            display.clearDisplay();
            display.display();
            display.ssd1306_command(SSD1306_DISPLAYOFF);
        }
        pinMode(PIN_LED, OUTPUT);
        digitalWrite(PIN_LED, LOW);
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 0);
        esp_deep_sleep_start();
    }
  }

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BTN_UP, INPUT_PULLUP);
  pinMode(PIN_BTN_DOWN, INPUT_PULLUP);



  // ── OLED INIT dengan RETRY (cegah infinite loop saat kabel kendur) ──
  // SSD1306 begin() gagal kalau I2C tidak ACK — paling sering karena
  // kabel SDA/SCL kendur ke OLED. Coba 5x dengan I2C recovery di antara,
  // baru fallback ke restart (boot_attempt counter akan handle infinite loop).
  bool oled_ok = false;
  for (int attempt = 0; attempt < 5 && !oled_ok; attempt++) {
    if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
      oled_ok = true;
      Serial.printf("[OLED] init OK attempt %d\n", attempt + 1);
    } else {
      Serial.printf("[OLED] init GAGAL attempt %d, recovery + retry...\n", attempt + 1);
      delay(200);
      recoverI2CBus(SDA, SCL);
      Wire.begin();
      Wire.setClock(400000);
    }
  }
  if (!oled_ok) {
    Serial.println("[OLED] FATAL: cek SOLDER SDA/SCL ke OLED!");
    // LED blink rapid → signal ke user ada hardware problem
    pinMode(PIN_LED, OUTPUT);
    for (int i = 0; i < 10; i++) {
      digitalWrite(PIN_LED, HIGH); delay(150);
      digitalWrite(PIN_LED, LOW);  delay(150);
    }
    // Restart instead of hang — boot_attempt counter akan trigger safe mode
    // setelah 3x gagal beruntun
    delay(500);
    ESP.restart();
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // ── Cek flag mode_switch ──
  // Kalau boot ini hasil dari requestBootMode() (user pilih ganti mode), skip
  // splash "WAKE UP!" → langsung tampilkan "Mempersiapkan MODE X" biar terasa
  // seamless (bukan kayak alat reset). Flag di-clear setelah dibaca.
  preferences.begin("b300_cfg", false);
  bool is_mode_switch = preferences.getBool("mode_switch", false);
  if (is_mode_switch) preferences.putBool("mode_switch", false);
  preferences.end();

  if (is_mode_switch) {
    // Tampilkan pesan transisi (mirror frame terakhir di requestBootMode)
    BootMode incoming_mode = readBootMode();
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    if (incoming_mode == BOOT_DEEP_SLEEP) {
      // Transisi tidur — frame zZz singkat (DEEP_SLEEP handler akan paint "Bye")
      display.setTextSize(2);
      display.setCursor(40, 22);
      display.println("zZz");
      display.display();
      Serial.println("[BOOT] Mode-switch to DEEP_SLEEP");
    } else {
      // Transisi mode aktif (AUDIO/SYNC) — "Mempersiapkan X [ * ]"
      const char* mode_name = (incoming_mode == BOOT_SYNC) ? "SYNC" : "AUDIO";
      display.setTextSize(1);
      display.setCursor(16, 6);
      display.println("Mempersiapkan");
      display.setTextSize(2);
      int16_t x1, y1; uint16_t w, h;
      display.getTextBounds(mode_name, 0, 0, &x1, &y1, &w, &h);
      display.setCursor((SCREEN_WIDTH - (int)w) / 2, 24);
      display.println(mode_name);
      display.setTextSize(1);
      display.setCursor(48, 52);
      display.print("[ * ]");
      display.display();
      Serial.println("[BOOT] Mode-switch transition (skip splash)");
    }
    // Tidak ada delay panjang — biar setup cepat lanjut ke mode init
  } else {
    // Boot normal (power-on / crash) — splash seperti biasa
    updateDisplay("WAKE UP!", "System Ready");
    delay(1000);
  }

  // === HIDDEN WIPE FEATURE ===
  // User tahan UP+DOWN selama "WAKE UP!" tampil → hapus /run.dat.
  // Berguna saat testing/dev — tidak perlu nunggu HP kirim ACK untuk delete.
  // Skip kalau mode_switch (user tidak mungkin nge-wipe saat ganti mode).
  if (!is_mode_switch && digitalRead(PIN_BTN_UP) == LOW && digitalRead(PIN_BTN_DOWN) == LOW) {
    Serial.println("[WIPE] UP+DOWN dipencet saat boot → hapus /run.dat");
    if (LittleFS.exists("/run.dat")) {
      LittleFS.remove("/run.dat");
      updateDisplay("DATA WIPED!", "/run.dat dihapus\nLepas tombol...");
      Serial.println("[WIPE] /run.dat berhasil dihapus");
    } else {
      updateDisplay("WIPE INFO", "Tidak ada data\nuntuk dihapus");
      Serial.println("[WIPE] /run.dat tidak ada");
    }
    delay(2500);
    // Tunggu user lepas tombol biar tidak nyangkut di handleInput nanti
    while (digitalRead(PIN_BTN_UP) == LOW || digitalRead(PIN_BTN_DOWN) == LOW) {
      delay(50);
      yield();
    }
    delay(200);  // debounce final
  }

  // 5. Battery callback SETELAH display siap
#ifdef TESTING_USB_ONLY
  // Mode test USB-only: SKIP cbCritical karena MAX17048 baca 0 (no battery)
  // akan trigger critical terus → restart loop. JANGAN pakai mode ini di produksi.
  Serial.println("[TEST] TESTING_USB_ONLY aktif — battery critical handler DI-SKIP");
#else
  battery.onCritical([]() {
      float v = battery.getVoltage();
      int   l = battery.getLevel();

      // GUARD 1: tolak baca I2C gagal.
      // Saat readRegister16() gagal (mutex timeout / glitch bus saat BT spike),
      // rawVcell tetap 0 → voltage=0.0V/level=0 → evaluate() salah fire critical.
      // LiPo terhubung TIDAK PERNAH baca 0V (ESP sudah brownout di ~2.7V),
      // jadi v<2.5V = pasti baca gagal, BUKAN batrai habis.
      if (v < 2.5f) {
          Serial.printf("[BAT] FALSE critical diabaikan — baca I2C gagal (V=%.2f L=%d)\n", v, l);
          return;
      }

      // GUARD 2: konfirmasi 2x berturut (tahan voltage-sag sesaat dari arus BT).
      // evaluate() jalan ~1x/detik, jadi butuh ~2 detik kritis terus-menerus.
      static uint8_t        crit_count = 0;
      static unsigned long  last_crit  = 0;
      if (millis() - last_crit > 3000) crit_count = 0;  // reset kalau sudah lama
      last_crit = millis();
      if (++crit_count < 2) {
          Serial.printf("[BAT] Critical fire #%u (V=%.2f) — tunggu konfirmasi\n", crit_count, v);
          return;
      }

      Serial.printf("[BAT] KRITIS TERKONFIRMASI (V=%.2f L=%d) — sleep\n", v, l);
      updateDisplay("BATERAI HABIS", "Sistem mati...");
      delay(1500);
      enterDeepSleep();
  });
  battery.onLow([](bool isLow) {
      if (isLow) updateDisplay("LOW BATTERY", "Segera charge!", false);
  });
#endif

  // 6. Sensor (Wire + mutex sudah siap)
  breathSensor.begin();
  stepSensor.begin();

  // 7. SensorTask SETELAH sensor selesai begin()
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 20480, NULL, 2, &SensorTaskHandle, 1);

  preferences.begin("b300_cfg", true);
  if (preferences.isKey("target_mac")) {
    preferences.getBytes("target_mac", current_mac, 6);
    is_mac_configured = true;
  }

  // ── PATH 1: RTC memory valid (deep sleep wake) ──
  // RTC_DATA_ATTR persisten setelah deep sleep karena RTC controller tetap power-on
  if (rtc_time_valid && rtc_unix_at_sleep > 1000000000UL) {
    uint64_t now_rtc_us  = rtcTicksToUs(rtc_time_get());
    uint64_t elapsed_us  = now_rtc_us - rtc_us_at_sleep;
    uint32_t elapsed_sec = (uint32_t)(elapsed_us / 1000000ULL);
    saved_unix_time      = rtc_unix_at_sleep + elapsed_sec;
    millis_at_sync       = millis();
    time_is_valid        = true;
    char tbuf[17];
    unixToString(saved_unix_time, tbuf);
    Serial.printf("[TIME] Lanjut dari RTC mem (deep sleep): %s WIB (tidur %us)\n", tbuf, elapsed_sec);
  }
  // ── PATH 2: NVS + RTC counter (soft restart) ──
  // RTC_DATA_ATTR direset oleh bootloader saat ESP.restart() di ESP32 Arduino,
  // tapi RTC HARDWARE COUNTER (rtc_time_get) tetap jalan terus.
  // Jadi: ambil unix_time + ticks dari NVS, hitung elapsed dari counter sekarang.
  else {
    uint32_t nvs_unix  = preferences.getUInt("unix_time", 0);
    uint64_t nvs_ticks = preferences.getULong64("rtc_ticks", 0);

    if (nvs_unix > 1000000000UL && nvs_ticks > 0) {
      uint64_t now_ticks      = rtc_time_get();
      uint64_t elapsed_ticks  = now_ticks - nvs_ticks;
      uint64_t elapsed_us     = rtcTicksToUs(elapsed_ticks);
      uint32_t elapsed_sec    = (uint32_t)(elapsed_us / 1000000ULL);
      // Sanity check: kalau elapsed > 1 hari, anggap NVS data terlalu lama / tidak valid
      if (elapsed_sec < 86400UL) {
        saved_unix_time = nvs_unix + elapsed_sec;
        millis_at_sync  = millis();
        time_is_valid   = true;
        char tbuf[17];
        unixToString(saved_unix_time, tbuf);
        Serial.printf("[TIME] Lanjut dari NVS+RTC counter (soft restart): %s WIB (elapsed %us)\n",
                      tbuf, elapsed_sec);
      } else {
        saved_unix_time = nvs_unix;
        time_is_valid   = false;
        millis_at_sync  = 0;
        Serial.printf("[TIME] NVS=%u tapi elapsed %us > 1hari, anggap invalid — sync HP\n",
                      nvs_unix, elapsed_sec);
      }
    } else {
      saved_unix_time = nvs_unix;
      time_is_valid   = false;
      millis_at_sync  = 0;
      if (saved_unix_time > 0) {
        Serial.printf("[TIME] NVS=%u, RTC ticks 0 — perlu sync HP\n", saved_unix_time);
      } else {
        Serial.println("[TIME] Belum pernah sync waktu!");
      }
    }
  }
  preferences.end();

  esp_reset_reason_t rr = esp_reset_reason();
  bool booted_from_crash = (rr == ESP_RST_PANIC   ||
                              rr == ESP_RST_INT_WDT  ||
                              rr == ESP_RST_TASK_WDT ||
                              rr == ESP_RST_WDT);

  Serial.printf("[BOOT] reset_reason=%d, booted_from_crash=%d, time_is_valid=%d, saved_unix=%u\n",
                (int)rr, booted_from_crash, time_is_valid, saved_unix_time);

  // ────────────────────────────────────────────────────────────────
  // BOOT MODE BRANCHING — Single-Mode-Per-Boot Architecture
  // ────────────────────────────────────────────────────────────────
  BootMode current_boot_mode = readBootMode();
  const char* bm_name = (current_boot_mode == BOOT_SYNC)       ? "SYNC"
                      : (current_boot_mode == BOOT_DEEP_SLEEP) ? "DEEP_SLEEP"
                      : "AUDIO";
  Serial.printf("[BOOT] mode=%s\n", bm_name);

  // Reset boot_mode di NVS ke AUDIO untuk next boot (default safe).
  // Kalau user mau SYNC lagi, tinggal pilih menu — set ulang sebelum restart.
  preferences.begin("b300_cfg", false);
  preferences.putUChar("boot_mode", BOOT_AUDIO);
  preferences.end();

  // ═══ DEEP SLEEP MODE: restart-to-sleep, BTDM tidak di-init ═══
  // enterDeepSleep() sudah save state ke NVS+RTC, sekarang tinggal masuk
  // deep sleep beneran tanpa harus shutdown BTDM (yang sebelumnya crash).
  if (current_boot_mode == BOOT_DEEP_SLEEP) {
    Serial.println("[BOOT] Path bersih ke deep sleep (no BTDM)");
    saveBootAttemptToNVS(0);  // reset crash counter — deep sleep ini disengaja

    // Refresh waktu ke RTC memory dengan tick terbaru
    if (time_is_valid) {
      rtc_unix_at_sleep = getCurrentUnix();
      rtc_us_at_sleep   = rtcTicksToUs(rtc_time_get());
      rtc_time_valid    = true;
    }

    // Frame "Bye" singkat — konfirmasi ke user bahwa device tidur normal
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(34, 18);
    display.println("Bye!");
    display.setTextSize(1);
    display.setCursor(6, 44);
    display.println("Tekan OK utk bangun");
    display.display();
    delay(700);

    // Tunggu tombol dilepas (jaga-jaga kalau masih nekan)
    unsigned long wait_start = millis();
    while (digitalRead(PIN_BTN_OK) == LOW) {
      delay(10);
      yield();
      if (millis() - wait_start > 5000) break;
    }
    delay(100);  // debounce

    // Display off + power off + deep sleep
    display.clearDisplay();
    display.display();
    display.ssd1306_command(SSD1306_DISPLAYOFF);

    // Suspend SensorTask supaya tidak ada I2C transaction saat MOSFET off
    if (SensorTaskHandle != NULL) vTaskSuspend(SensorTaskHandle);

    battery.powerOff();
    rtc_gpio_pullup_en((gpio_num_t)PIN_BTN_OK);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_32, 0);
    Serial.println("[SLEEP] Bye.");
    Serial.flush();
    esp_deep_sleep_start();
    // unreachable
  }

  if (current_boot_mode == BOOT_SYNC) {
    // ═══ SYNC MODE: init BLE only, skip A2DP ═══
    if (!time_is_valid) {
      updateDisplay("SYNC MODE", "Hubungkan HP\nuntuk set waktu");
    } else {
      updateDisplay("SYNC MODE", "Menunggu HP...");
    }
    initBLEForSync();
    // current_state sudah di-set ke STATE_SYNC di initBLEForSync()
    // CATATAN: boot_attempt JANGAN direset di sini — di-reset oleh loop() setelah
    // device stable 30 detik. Kalau crash di loop, counter tetap akumulate.
    return;
  }

  // ═══ AUDIO MODE: init A2DP, tidak ada BLE ═══
  a2dp_source.set_auto_reconnect(is_mac_configured);
  a2dp_source.set_on_connection_state_changed(connection_state_changed);
  a2dp_source.set_data_callback(get_sound_data);
  a2dp_source.set_volume(current_volume);
  a2dp_source.set_ssid_callback(target_filter_callback);
  a2dp_source.start((std::vector<const char*>){{}});
  a2dp_active = true;

  initAudioTimer();

  for(int i = 0; i < SMA_BUFFER_SIZE; i++) step_intervals[i] = 0;
  sma_initialized             = false;
  startup_time                = millis();
  is_initial_connect_attempt  = is_mac_configured;

  if (booted_from_crash) {
      is_mac_configured          = false;
      is_initial_connect_attempt = false;
      has_shown_not_found        = true;
      current_state              = STATE_MAIN_MENU;
      updateDisplay("RECONNECT", "Pilih Scan");
      delay(1500);
      if (!time_is_valid) {
        updateDisplay("WAKTU BELUM SET", "Sync ke HP dulu\n(Menu > SYNC)");
        delay(2500);
      }
      showMainMenu();
  } else if (!is_mac_configured) {
      has_shown_not_found = true;
      current_state       = STATE_MAIN_MENU;
      if (!time_is_valid) {
        updateDisplay("WAKTU BELUM SET", "Sync ke HP dulu\n(Menu > SYNC)");
        delay(2500);
      }
      showMainMenu();
  } else {
      if (!time_is_valid) {
        updateDisplay("WAKTU BELUM SET", "Sync ke HP dulu\n(Menu > SYNC)");
        delay(2500);
        current_state   = STATE_MAIN_MENU;
        main_menu_index = 1;
        showMainMenu();
      } else {
        updateDisplay("Connecting...", "Wait 20s...");
      }
  }

  // CATATAN: boot_attempt JANGAN direset di sini.
  // Di-reset oleh loop() setelah device stable 30 detik (lihat loop() awal).
  // Tujuannya: kalau crash terjadi di loop() (mis. batrai critical false-trigger),
  // counter tetap akumulate → eventually trigger safe mode setelah 4 boot crash.
}

#ifdef TESTING_BREATH
// Hitung state napas simulasi pada timestamp SEKARANG, berdasarkan
// elapsed time + active_pattern_divisor. Dipanggil dari handleStepAnalysis
// supaya step rows juga membawa breath state yang konsisten dengan simulasi.
// Tanpa ini, step rows akan pakai breathSensor.getCurrentPhase() yang return 0
// (karena sensor belum wired) → kolom napas 0 walau transisi sudah ditulis.
int8_t getCurrentSimulatedBreathState() {
  if (!recording_active) return 0;

  float spm = stepSensor.getSPM();
  if (spm < 100.0f) spm = 200.0f;
  unsigned long step_interval_ms = (unsigned long)(60000.0f / spm);
  int steps_inhale = (active_pattern_divisor == 5) ? 3 : 2;
  int steps_total  = active_pattern_divisor;
  unsigned long inhale_duration_ms = steps_inhale * step_interval_ms;
  unsigned long cycle_duration_ms  = steps_total  * step_interval_ms;

  long elapsed_signed = (long)(millis() - recording_start_ms) - (long)TESTING_BREATH_LAG_MS;
  if (elapsed_signed < 0) return 1;  // sebelum cycle pertama, default hirup
  unsigned long elapsed = (unsigned long)elapsed_signed;
  unsigned long position = elapsed % cycle_duration_ms;
  return (position < inhale_duration_ms) ? 1 : -1;
}

// Generator fake breath events sinkron dengan pola aktif.
// Sambil recording active, hitung "user seharusnya napas apa sekarang"
// berdasarkan elapsed time dan active_pattern_divisor.
// Saat phase berubah, tulis row breath transition (step=0).
void simulateBreathEvent() {
  if (!recording_active || !logging_enabled) return;

  // Hitung step interval dari current SPM (atau fallback 200 SPM = 300ms)
  float spm = stepSensor.getSPM();
  if (spm < 100.0f) spm = 200.0f;
  unsigned long step_interval_ms = (unsigned long)(60000.0f / spm);

  // Pola: 3:2 (divisor=5) → 3 step hirup, 2 step buang
  //       2:1 (divisor=3) → 2 step hirup, 1 step buang
  int steps_inhale = (active_pattern_divisor == 5) ? 3 : 2;
  int steps_total  = active_pattern_divisor;

  unsigned long inhale_duration_ms = steps_inhale * step_interval_ms;
  unsigned long cycle_duration_ms  = steps_total  * step_interval_ms;

  // Tentukan state yang SEHARUSNYA sekarang (dengan optional lag offset)
  long elapsed_signed = (long)(millis() - recording_start_ms) - (long)TESTING_BREATH_LAG_MS;
  if (elapsed_signed < 0) return;  // belum waktunya
  unsigned long elapsed = (unsigned long)elapsed_signed;

  unsigned long position_in_cycle = elapsed % cycle_duration_ms;
  int8_t expected_state = (position_in_cycle < inhale_duration_ms) ? 1 : -1;

  // Detect transition
  static int8_t last_simulated_state = 0;
  if (expected_state != last_simulated_state) {
    last_simulated_state = expected_state;

    uint32_t ts_relative = (uint32_t)(millis() - recording_start_ms);
    writeLogEntry(ts_relative, expected_state, 0.0f, /*is_step_event=*/false);
    Serial.printf("[FAKE BREATH] event=%s @ t=%ums (simulated)\n",
                  expected_state == 1 ? "HIRUP" : "BUANG", ts_relative);
  }
}
#endif

void loop() {
  // ── Defensive reset: kalau is_pattern_switching stuck (mis. crash di toggle),
  //   reset di sini supaya RESUME recording tidak terblokir permanen.
  is_pattern_switching = false;

  // ── DUAL-EVENT LOGGING: Catat napas dengan independent timestamp ──
  // Saat BreathSensor detect transisi (hirup↔buang), langsung tulis row dengan
  // timestamp PRESISI (resolusi ~20ms), bukan menunggu step berikutnya.
  // Row breath-only: step=0, spm=0, breathPhase=new state.
  // HP rekonstruksi continuous state dari sequence event step+breath.
#ifdef TESTING_BREATH
  // === MODE TEST: pakai simulasi karena sensor napas belum wired ===
  simulateBreathEvent();
#else
  // === MODE PRODUCTION: pakai sensor napas real ===
  if (recording_active && logging_enabled) {
    if (breathSensor.isInhaleDetected() || breathSensor.isExhaleDetected()) {
        int8_t btype = breathSensor.getLastEventType();              // baca type DULU (tidak reset flag)
        uint32_t breath_ts_abs = breathSensor.getLastEventTimestamp(); // reset flag setelah baca

        // Konversi absolute millis() → relatif terhadap recording_start_ms
        uint32_t ts_relative;
        if (breath_ts_abs > 0 && breath_ts_abs >= recording_start_ms) {
            ts_relative = breath_ts_abs - recording_start_ms;
        } else {
            ts_relative = (uint32_t)(millis() - recording_start_ms);
        }

        // Tulis breath transition row (step=0, spm=0)
        writeLogEntry(ts_relative, btype, 0.0f, /*is_step_event=*/false);
        Serial.printf("[BREATH] event=%s @ t=%ums (presisi)\n",
                      btype == 1 ? "HIRUP" : "BUANG", ts_relative);
    }
  }
#endif

  // ── Flush file secara berkala ──
  static unsigned long lastLogFlush = 0;
  if (millis() - lastLogFlush > 5000) {
    flushLogBuffer();
    lastLogFlush = millis();
  }

  handleInput();
  battery.update();  // baca sensor + evaluasi kondisi otomatis via callback

  static bool last_connected_status = false;
  bool current_connected_status     = false;

  // ── STATE SYNC ──
  if (current_state == STATE_SYNC) {
      // SYNC TIMEOUT: 60 detik TOTAL dari masuk SYNC mode sampai HP kirim "SYNC".
      // Counter mulai saat enter SYNC, TIDAK reset walau HP connect — supaya kasus
      // HP-konek-tapi-diam (Flutter lupa kirim SYNC command) tetap kena timeout.
      // 60s = cukup buat HP scan + connect + user tap tombol sync di app.
      static unsigned long sync_enter_ms = 0;
      if (sync_enter_ms == 0) sync_enter_ms = millis();
      int connCount = pServer ? pServer->getConnectedCount() : 0;

      if (!isSyncing && (millis() - sync_enter_ms > 60000UL)) {
          if (connCount > 0) {
              updateDisplay("SYNC TIMEOUT", "HP konek tapi\ntdk kirim SYNC");
          } else {
              updateDisplay("SYNC TIMEOUT", "HP tidak konek\nKembali ke menu");
          }
          delay(2000);
          requestBootMode(BOOT_AUDIO, "Sync timeout");
          return;
      }

      if (!isSyncing) {
          static unsigned long lastWaitDisp = 0;
          if (millis() - lastWaitDisp > 1000) {
              lastWaitDisp = millis();
              updateDisplay("SYNC MODE",
                connCount > 0 ? "HP connected!\nKirim SYNC..." : "Waiting HP...");
          }
          return;
      }

      if (isSyncing && !ble_deinit_done) {
          static unsigned long lastBattNotify = 0;
          if (pBattCharacteristic && millis() - lastBattNotify > 5000) {
              lastBattNotify = millis();
              int rawLvl = battery.getLevel();
              uint8_t lvl = (uint8_t)(rawLvl < 0 ? 0 : rawLvl > 100 ? 100 : rawLvl);
              pBattCharacteristic->setValue(&lvl, 1);
              pBattCharacteristic->notify();
          }

          // ── Cek HP masih terkoneksi ──
          if (!pServer || pServer->getConnectedCount() == 0) {
              isSyncing = false;
              updateDisplay("SYNC GAGAL", "HP disconnect");
              delay(2000);
              requestBootMode(BOOT_AUDIO, "HP disconnect");
              return;
          }

          // ── Tutup logFile sebelum baca ──
          if (logFile) {
              logFile.flush();
              logFile.close();
              logFile = (File)NULL;
          }

          File readFile = LittleFS.open("/run.dat", FILE_READ);
          bool has_data = readFile && readFile.size() > 0;

          if (has_data) {
              uint32_t totalBytes = readFile.size();
              Serial.printf("[SYNC] Mengirim %u bytes...\n", totalBytes);

              // 1. Kirim header kolom dulu (wajib, aplikasi HP mengharapkan ini)
              const char* csvHeader = "sesi,mulai_lari,waktu_ms,breathPhase,step,spm,patternID;";
              pCharacteristic->setValue(csvHeader);
              pCharacteristic->notify();
              delay(50);

              String   buffer      = "";
              buffer.reserve(400);
              uint32_t sent        = 0;
              uint32_t lastPct     = 0;
              bool     transfer_ok = true;

              uint32_t sync_session_id     = 0;
              char     sync_mulai_lari[22] = "00/00/0000, 00.00.00";

              while (readFile.available() >= 1) {
                  if (pServer->getConnectedCount() == 0) {
                      transfer_ok = false;
                      Serial.println("[SYNC] HP disconnect saat transfer!");
                      break;
                  }

                  uint8_t firstByte;
                  readFile.read(&firstByte, 1);
                  sent += 1;

                  if (firstByte == 0xFF) {
                      // ── Baca header sesi (8 byte setelah magic) ──
                      if (readFile.available() < 8) break;  // file corrupt
                      uint8_t h[8];
                      readFile.read(h, 8);
                      sent += 8;
                      sync_session_id        = ((uint32_t)h[0]<<24)|((uint32_t)h[1]<<16)|((uint32_t)h[2]<<8)|h[3];
                      uint32_t unix_ts       = ((uint32_t)h[4]<<24)|((uint32_t)h[5]<<16)|((uint32_t)h[6]<<8)|h[7];

                      // Hitung tanggal WIB (sama seperti kode lama)
                      uint32_t t   = unix_ts + 7 * 3600UL;
                      uint32_t sec = t % 60; t /= 60;
                      uint32_t mn  = t % 60; t /= 60;
                      uint32_t hr  = t % 24; t /= 24;
                      uint32_t days = t, year = 1970;
                      while (true) {
                          bool leap = (year%4==0 && (year%100!=0||year%400==0));
                          uint32_t diy = leap ? 366 : 365;
                          if (days < diy) break;
                          days -= diy; year++;
                      }
                      bool leap = (year%4==0 && (year%100!=0||year%400==0));
                      const uint8_t dim[] = {31,28,31,30,31,30,31,31,30,31,30,31};
                      uint32_t month = 1;
                      for (int m = 0; m < 12; m++) {
                          uint32_t d = dim[m] + (m==1&&leap?1:0);
                          if (days < d) { month = m+1; break; }
                          days -= d;
                      }
                      snprintf(sync_mulai_lari, sizeof(sync_mulai_lari),
                              "%02u/%02u/%04u, %02u.%02u.%02u",
                              (unsigned)(days+1),(unsigned)month,(unsigned)year,
                              (unsigned)hr,(unsigned)mn,(unsigned)sec);
                      Serial.printf("[SYNC] Sesi #%u mulai: %s\n", sync_session_id, sync_mulai_lari);
                      continue;
                  }

                  // ── Baca record biasa (6 byte setelah byte pertama) ──
                  if (readFile.available() < 5) break;  // file corrupt
                  uint8_t rec[5];
                  readFile.read(rec, 5);
                  sent += 5;

                  uint32_t ts_ms = ((uint32_t)firstByte<<24)|((uint32_t)rec[0]<<16)|((uint32_t)rec[1]<<8)|rec[2];
                  uint8_t  flags = rec[3];
                  uint8_t  spm   = rec[4];

                  int      is_step = (flags >> 0) & 0x01;
                  int      bp_enc  = (flags >> 1) & 0x03;
                  int      pattern = (flags >> 3) & 0x01;
                  int8_t   breath  = (bp_enc == 1) ? 1 : (bp_enc == 2) ? -1 : 0;

                  char csvLine[56];
                  snprintf(csvLine, sizeof(csvLine), "%u,\"%s\",%lu,%d,%d,%d,%d;\n",
                          sync_session_id,
                          sync_mulai_lari,
                          (unsigned long)ts_ms,
                          (int)breath, is_step, (int)spm, pattern);

                  buffer += csvLine;
                  if (buffer.length() >= 350) {
                      pCharacteristic->setValue(buffer.c_str());
                      pCharacteristic->notify();
                      buffer = "";
                      delay(40);

                      uint32_t pct = (sent * 100) / totalBytes;
                      if (pct != lastPct && pct % 10 == 0) {
                          lastPct = pct;
                          display.clearDisplay();
                          display.setCursor(0, 0);  display.print("SYNCING...");
                          display.setCursor(0, 20); display.print(pct); display.print("%");
                          safeDisplayUpdate();
                          Serial.printf("[SYNC] Progress: %u%%\n", pct);
                      }
                  }
              }
              readFile.close();

              // Kirim sisa buffer
              if (transfer_ok && buffer.length() > 0) {
                  pCharacteristic->setValue(buffer.c_str());
                  pCharacteristic->notify();
                  delay(40);
              }

              // 2. Kirim EOF; (sesuai template teman)
              if (transfer_ok) {
                  delay(50);
                  pCharacteristic->setValue("EOF");
                  pCharacteristic->notify();
                  Serial.println("[SYNC] EOF; terkirim. Menunggu ACK dari HP...");
                  updateDisplay("SYNC DONE", "Tunggu konfirmasi\ndari HP...");

                  // ── TUNGGU ACK dari HP sebelum delete file & restart ──
                  // HP harus kirim "OK"/"DONE"/"ACK"/"SELESAI" setelah parse data.
                  // Kalau tidak ACK dalam 30 detik, file TIDAK dihapus → user bisa retry.
                  sync_completed_ack = false;
                  unsigned long ack_start = millis();
                  const unsigned long ACK_TIMEOUT_MS = 30000UL;

                  while (!sync_completed_ack && (millis() - ack_start < ACK_TIMEOUT_MS)) {
                      if (pServer->getConnectedCount() == 0) {
                          Serial.println("[SYNC] HP disconnect saat tunggu ACK!");
                          break;
                      }
                      delay(100);
                      yield();
                  }

                  if (sync_completed_ack) {
                      LittleFS.remove("/run.dat");
                      updateDisplay("SYNC SUKSES!", "Data tersimpan!\nTekan OK utk\nkembali ke AUDIO");
                      Serial.println("[SYNC] ACK diterima, file /run.dat dihapus");
                      Serial.println("[SYNC] Tunggu user pencet OK untuk kembali ke AUDIO mode...");

                      // ── TUNGGU USER PENCET OK (atau timeout 60s) ──
                      // BLE tetap aktif selama wait — HP masih bisa kirim TIME update dll.
                      // Bukan crash risk: BLE event-driven, idle tidak butuh apa-apa.
                      unsigned long ok_wait_start = millis();
                      const unsigned long OK_WAIT_TIMEOUT_MS = 60000UL;
                      bool user_confirmed = false;

                      while (millis() - ok_wait_start < OK_WAIT_TIMEOUT_MS) {
                          if (digitalRead(PIN_BTN_OK) == LOW) {
                              delay(50);  // debounce
                              if (digitalRead(PIN_BTN_OK) == LOW) {
                                  user_confirmed = true;
                                  // Tunggu user lepas tombol biar tidak ke-detect lagi setelah restart
                                  while (digitalRead(PIN_BTN_OK) == LOW) {
                                      delay(20);
                                      yield();
                                  }
                                  break;
                              }
                          }
                          delay(50);
                          yield();
                      }

                      if (user_confirmed) {
                          Serial.println("[SYNC] User pencet OK, lanjut restart ke AUDIO");
                          updateDisplay("OK!", "Kembali ke AUDIO\nmode...");
                      } else {
                          Serial.println("[SYNC] Timeout 60s tanpa OK — auto-restart ke AUDIO");
                          updateDisplay("AUTO-EXIT", "Timeout, kembali\nke AUDIO mode");
                      }
                  } else {
                      // Tidak ada konfirmasi — file dipertahankan supaya user bisa sync ulang
                      updateDisplay("SYNC GAGAL", "HP tdk konfirmasi\nCoba sync lagi");
                      Serial.println("[SYNC] TIMEOUT/disconnect tanpa ACK — file DIPERTAHANKAN");
                  }
              } else {
                  updateDisplay("SYNC GAGAL", "Koneksi putus");
              }

          } else {
              // Tidak ada data
              if (readFile) readFile.close();
              const char* csvHeader = "sesi,mulai_lari,waktu_ms,breathPhase,step,spm,patternID;";
              pCharacteristic->setValue(csvHeader);
              pCharacteristic->notify();
              delay(50);
              pCharacteristic->setValue("EOF;");
              pCharacteristic->notify();
              Serial.println("[SYNC] Tidak ada data, EOF; dikirim.");
              updateDisplay("SYNC", "Tidak ada data");
          }

          // ── Selesai sync: restart ke AUDIO mode (clean state, no BTDM mess) ──
          isSyncing = false;
          delay(2000);
          requestBootMode(BOOT_AUDIO, "Sync selesai");
          return;
      }
      return;
  }

  if (current_state != STATE_MAIN_MENU) {
      current_connected_status = a2dp_source.is_connected();
  }

  // ── DISCONNECT GUARD: STATE_MENU_PATTERN ──
  // Kalau earphone tiba-tiba mati saat user lagi pilih pola 3:2 / 2:1,
  // langsung balik ke main menu — jangan biarin user lanjut pilih pola
  // tanpa koneksi karena STATE_IDLE tanpa A2DP = no audio guidance.
  if (current_state == STATE_MENU_PATTERN && !current_connected_status) {
      updateDisplay("EARPHONE LEPAS!", "Kembali ke menu", false);
      delay(2000);
      current_state              = STATE_MAIN_MENU;
      main_menu_index            = 0;
      is_initial_connect_attempt = false;
      has_shown_not_found        = true;
      showMainMenu();
      return;
  }

  if (is_initial_connect_attempt && is_mac_configured && !current_connected_status) {
      if (millis() - startup_time > 20000) {
          is_initial_connect_attempt = false;
          if (!has_shown_not_found) {
             digitalWrite(PIN_LED, LOW);
             has_shown_not_found = true;
             current_state       = STATE_MAIN_MENU;
             main_menu_index     = 0;
             showMainMenu();
          }
      }
  }

  if (current_connected_status && is_initial_connect_attempt)
      is_initial_connect_attempt = false;

  if (current_state != STATE_SCANNING   && current_state != STATE_MENU_PATTERN &&
      current_state != STATE_PAIRING    && current_state != STATE_MAIN_MENU) {

      if (!current_connected_status && last_connected_status) {
        if (current_state != STATE_SCANNING && !is_shutting_down) {
            esp_timer_stop(audio_timer);
            is_playing_sound    = false;
            new_sound_requested = false;
            algo_state          = STATE_ANALYZING;
            stable_count        = 0;
            missed_step_count   = 0;
            sma_initialized     = false;
            // --- TAMBAHAN PENGAMAN ---
            // Stop recording dan simpan data ke flash saat earphone mati
            if (recording_active) {
                recording_active = false;
                flushLogBuffer();
                Serial.println("[CONN] Recording dihentikan karena earphone mati");
            }
            // -------------------------
            current_state       = STATE_IDLE;
            updateDisplay("DISCONNECTED", "Reconnecting...");
            digitalWrite(PIN_LED, LOW);
        }
      }
      if (current_connected_status && current_state == STATE_IDLE) {
          #ifdef TESTING_SPM
          {
              static unsigned long lastFakeStep = 0;
              unsigned long interval = 60000UL / TESTING_SPM;
              if (millis() - lastFakeStep >= interval) {
                  lastFakeStep = millis();
                  handleStepAnalysis();
              }
          }
          #else
          if (stepSensor.isStepDetected()) handleStepAnalysis();
          #endif
          updateMonitorDisplay();
          debugSerialPlotter();

      }
  }

  if (current_state != STATE_MAIN_MENU) {
      last_connected_status = current_connected_status;
  }

  if (trigger_hard_scan) startBluetoothScan();

  if (current_state == STATE_SCANNING) {
    unsigned long elapsed = millis() - scan_start_time;
    static unsigned long last_scan_disp = 0;

    if (millis() - last_scan_disp > 500) {
        last_scan_disp = millis();
        if (elapsed > 100 && elapsed < SCAN_DURATION) {
           display.clearDisplay();
           display.setCursor(0,0); display.print("SCANNING...");
           display.setCursor(0,20); display.print("Found: "); display.print(found_devices.size());
           safeDisplayUpdate();
        }
    }

    if (elapsed > SCAN_DURATION) {
      FoundDevice rescanOpt;
      rescanOpt.name          = "[ SCAN ULANG ]";
      rescanOpt.is_cmd_rescan = true;
      found_devices.push_back(rescanOpt);
      current_state = STATE_SELECTING;
      showSelectionScreen();
    }
  }
}