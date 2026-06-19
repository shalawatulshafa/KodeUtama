#ifndef BATTERY_MGR_H
#define BATTERY_MGR_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <freertos/semphr.h>
#include <driver/rtc_io.h>
#include <driver/gpio.h>

extern SemaphoreHandle_t wireMutex;

#define MAX17048_ADDR 0x36
//#define BATTERY_DUMMY   // ← chip sudah terpasang, dummy dimatikan
// ─────────────────────────────────────────────────────────────────────
// Tipe callback yang diisi oleh file .ino:
//   BatteryCriticalCb  → dipanggil saat voltage ≤ cutoff atau SOC = 0
//                        (biasanya isi dengan enterDeepSleep)
//   BatteryLowCb(bool) → dipanggil setiap status low-bat BERUBAH
//                        true  = baru masuk zona lemah
//                        false = kembali normal
// ─────────────────────────────────────────────────────────────────────
typedef void (*BatteryCriticalCb)();
typedef void (*BatteryLowCb)(bool isLow);

class BatteryMgr {
  private:
    // ── Sensor state ─────────────────────────────────────────────────
    float         voltage;
    int           level;
    unsigned long lastUpdate;

    // ── Pin hardware ─────────────────────────────────────────────────
    uint8_t pinLedRed;
    uint8_t pinMosfet;

    // ── Threshold ────────────────────────────────────────────────────
    int   criticalLevel;   // % SOC → masuk low-bat warning
    float cutoffVoltage;   // Volt  → paksa deep sleep

    // ── State tracking ───────────────────────────────────────────────
    bool isLowBatteryMode;

    // ── Callbacks ────────────────────────────────────────────────────
    BatteryCriticalCb cbCritical;
    BatteryLowCb      cbLow;

    // ── Baca register 16-bit dari MAX17048 via mutex ──────────────────
    void readRegister16(uint8_t reg, uint16_t &value) {
        if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            Wire.beginTransmission(MAX17048_ADDR);
            Wire.write(reg);
            Wire.endTransmission(false);
            Wire.requestFrom(MAX17048_ADDR, 2);
            if (Wire.available() == 2) {
                value = (Wire.read() << 8) | Wire.read();
            }
            xSemaphoreGive(wireMutex);
        }
    }

    // ── Evaluasi kondisi baterai, panggil callback jika perlu ─────────
    void evaluate() {
        if (level < 0) return;  // belum terbaca pertama kali, skip

        Serial.printf("[BAT] %d%% | %.3fV\n", level, voltage);

        // Kondisi 1: Cut-off — habis total, paksa sleep
        if (level == 0 || voltage <= cutoffVoltage) {
            Serial.println("[BAT] KRITIS: cut-off tercapai!");
            if (cbCritical) cbCritical();
            return;
        }

        // Kondisi 2: Low Battery
        if (level <= criticalLevel) {
            digitalWrite(pinLedRed, HIGH);
            if (!isLowBatteryMode) {
                isLowBatteryMode = true;
                Serial.println("[BAT] WARNING: Low Battery Mode aktif!");
                if (cbLow) cbLow(true);
            }
        }
        // Kondisi 3: Normal
        else {
            digitalWrite(pinLedRed, LOW);
            if (isLowBatteryMode) {
                isLowBatteryMode = false;
                Serial.println("[BAT] Baterai kembali normal.");
                if (cbLow) cbLow(false);
            }
        }
    }

  public:
    // ─────────────────────────────────────────────────────────────────
    // begin() — inisialisasi sensor + MOSFET peripheral + LED
    //
    // Parameter (semua punya default, bisa dipanggil tanpa argumen):
    //   ledRedPin   : pin LED indikator baterai lemah  (default 26)
    //   mosfetPin   : pin Gate Si2301 Active-LOW       (default 27)
    //   lowLevel    : % SOC ambang low-bat warning     (default 15)
    //   cutoffVolts : Volt ambang cut-off deep sleep   (default 3.2)
    // ─────────────────────────────────────────────────────────────────
    void begin(uint8_t ledRedPin   = 26,
               uint8_t mosfetPin   = 27,
               int     lowLevel    = 15,
               float   cutoffVolts = 3.2f) {

        voltage          = 0.0f;
        level            = -1;
        lastUpdate       = 0;
        isLowBatteryMode = false;
        cbCritical       = nullptr;
        cbLow            = nullptr;

        pinLedRed     = ledRedPin;
        pinMosfet     = mosfetPin;
        criticalLevel = lowLevel;
        cutoffVoltage = cutoffVolts;

        // LED indikator baterai lemah
        pinMode(pinLedRed, OUTPUT);
        digitalWrite(pinLedRed, LOW);

        // Buka gpio_hold dari siklus deep sleep sebelumnya
        gpio_hold_dis((gpio_num_t)pinMosfet);
        gpio_deep_sleep_hold_dis();

        // Nyalakan daya peripheral via MOSFET Si2301
        // (Active-LOW: LOW = ON, HIGH = OFF)
        pinMode(pinMosfet, OUTPUT);
        digitalWrite(pinMosfet, HIGH);

        Serial.println("[PWR] MOSFET ON — peripheral powered.");
    }

    // ─────────────────────────────────────────────────────────────────
    // Daftarkan callback dari file .ino
    // ─────────────────────────────────────────────────────────────────
    void onCritical(BatteryCriticalCb cb) { cbCritical = cb; }
    void onLow(BatteryLowCb cb)           { cbLow      = cb; }

    // ─────────────────────────────────────────────────────────────────
    // update() — baca sensor tiap 1 detik, lalu evaluasi kondisi
    // ─────────────────────────────────────────────────────────────────
    void update() {
    #ifdef BATTERY_DUMMY
    level   = 80;    // pura-pura 80%
    voltage = 3.8f;  // pura-pura 3.8V
    return;
    #endif
        if (millis() - lastUpdate < 1000) return;
        lastUpdate = millis();

        uint16_t rawVcell = 0;
        uint16_t rawSoc   = 0;

        readRegister16(0x02, rawVcell);  // VCELL register
        readRegister16(0x04, rawSoc);    // SOC register

        // Datasheet MAX17048: VCELL LSB = 78.125 uV, 12-bit efektif di 16-bit
        voltage = (rawVcell >> 4) * 0.00125f;

        // SOC: high byte = integer %, low byte = fractional/256
        float soc = (rawSoc >> 8) + ((rawSoc & 0xFF) / 256.0f);
        level = constrain((int)round(soc), 0, 100);

        evaluate();  // cek kondisi & trigger callback kalau perlu
    }

    // ─────────────────────────────────────────────────────────────────
    // powerOff() — putus daya peripheral + kunci pin untuk deep sleep
    // Wajib dipanggil di enterDeepSleep() sebelum esp_deep_sleep_start()
    // ─────────────────────────────────────────────────────────────────
    void powerOff() {
        digitalWrite(pinLedRed, LOW);
        digitalWrite(pinMosfet, HIGH);          // MOSFET OFF → peripheral mati
        gpio_hold_en((gpio_num_t)pinMosfet);    // Kunci HIGH selama CPU aktif
        gpio_deep_sleep_hold_en();              // Tahan kunci saat CPU mati total
        Serial.println("[PWR] MOSFET OFF — peripheral unpowered.");
    }

    // ── Getter ───────────────────────────────────────────────────────
    int   getLevel()   { return level;           }
    float getVoltage() { return voltage;         }
    bool  isLowBat()   { return isLowBatteryMode; }

    // ─────────────────────────────────────────────────────────────────
    // draw() — render ikon baterai + persentase ke OLED
    // ─────────────────────────────────────────────────────────────────
    void draw(Adafruit_SSD1306 &display) {
        int x = 112, y = 0, w = 14, h = 8;

        display.drawRect(x, y, w, h, SSD1306_WHITE);           // Body baterai
        display.fillRect(x + w, y + 2, 2, 4, SSD1306_WHITE);  // Terminal kecil

        if (level > 0) {
            int fillWidth = map(level, 0, 100, 0, w - 2);
            display.fillRect(x + 1, y + 1, fillWidth, h - 2, SSD1306_WHITE);
        }

        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(90, 0);
        if (level < 0) {
            display.print("--");
        } else {
            display.print(level);
            display.print("%");
        }
    }
};

#endif