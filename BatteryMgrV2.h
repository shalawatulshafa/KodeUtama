#ifndef BATTERY_MGR_H
#define BATTERY_MGR_H

// ---------------------------------------------------------------------------
// BatteryMgr V2 — MAX17048 Fuel Gauge Manager untuk Cheaststrap ESP32
// ---------------------------------------------------------------------------
// Fitur:
//   • Membaca VCELL & SOC dari MAX17048 via I2C (addr 0x36) dengan mutex
//   • Sanity check register (0xFFFF, 0x0000, range VCELL 2.0-4.5V, SOC ≤105%)
//   • SOC scaling: raw MAX17048 (max ~82% karena TP4056 cutoff) → UI 0-100%
//   • EMA filter 25% pada SOC untuk mengurangi jitter tampilan
//   • Debounce + histeresis untuk cutoff & low-battery (false trigger protection)
//   • Sensor-lost detection: 30x gagal baca → state UNKNOWN, grace 5 siklus reconnect
//   • Re-entrancy guard: callback tidak bisa dipanggil bersarang
//
// State Machine:
//   UNKNOWN     → sensor belum terbaca / lost
//   NORMAL      → SOC > criticalLevel (default 15%)
//   LOW_BATTERY → SOC ≤ criticalLevel (LED pin menyala, banner OLED aktif)
//   HardCutoff  → SOC raw ≤1% (3 siklus) ATAU tegangan ≤ cutoffVoltage (5 siklus)
//                  → cbCritical() dipanggil SEKALI → user wajib enterDeepSleep()
//
// Hardware:
//   • AP2112K-3.3 LDO → cutoffVoltage default 3.5V (margin 0.2V di atas dropout)
//   • Si2301 MOSFET P-ch (active-LOW): tidak digunakan, selalu OFF (HIGH)
//   • LED indikator: pin 26 (LOW=off, HIGH=low-battery/kritis)
//
// API:
//   begin(ledPin, mosfetPin, lowLevel%, cutoffV)  — inisialisasi
//   onCritical(cb)                                 — callback saat hardCutoff
//   onState(cb)                                    — callback saat state berubah
//   update()                                       — panggil tiap loop(), baca 1/detik
//   powerOff()                                     — panggil sebelum deep sleep
//   draw(display)                                  — render ikon baterai ke OLED
//   getLevel() / getVoltage() / getState()         — getter
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <freertos/semphr.h>
#include <driver/rtc_io.h>
#include <driver/gpio.h>
#include <math.h>

extern SemaphoreHandle_t wireMutex;

#define MAX17048_ADDR 0x36
#define BATTERY_DUMMY    // Uncomment untuk simulasi tanpa sensor MAX17048
//#define BATTERY_VERBOSE  // Uncomment untuk log detail SOC & voltase tiap detik

typedef void (*BatteryCriticalCb)();

enum class BatteryState : uint8_t {
    UNKNOWN = 0,   // sensor belum terbaca / lost / kritis (sebelum cbCritical)
    NORMAL,        // SOC di atas criticalLevel
    LOW_BATTERY    // SOC ≤ criticalLevel, LED menyala
};

typedef void (*BatteryStateCb)(BatteryState state);

class BatteryMgr {
  private:
    static constexpr uint16_t SOC_ZERO_THRESHOLD       = 3;
    static constexpr uint16_t VOLT_DEBOUNCE_THRESHOLD  = 5;
    static constexpr float    VOLT_HYST                = 0.05f;
    static constexpr uint8_t  RECONNECT_GRACE_CYCLES   = 5;
    static constexpr uint16_t SENSOR_LOST_THRESHOLD    = 30;

    // MAX17048 raw SOC yang dianggap "100%" oleh sistem/UI.
    // Dari pengukuran: TP4056 stop charge saat raw SOC sekitar 82%.
    static constexpr float    SOC_UI_FULL_RAW          = 82.0f;

    float    voltage;        // Volt aktual hasil konversi register VCELL
    float    filteredSoc;    // SOC untuk UI (scaled + EMA)
    float    rawSoc;         // SOC mentah dari MAX17048 (untuk proteksi)
    int      level;          // Persen untuk UI (hasil round(filteredSoc))
    uint32_t lastUpdate;

    uint8_t  pinLedRed;
    uint8_t  pinMosfet;

    int      criticalLevel;  // threshold warning LOW_BATTERY berbasis UI %
    float    cutoffVoltage;  // threshold undervoltage berbasis tegangan mentah

    bool     initialized;
    bool     criticalTriggered;
    bool     criticalNullptrWarned;
    bool     sensorLost;
    bool     powerOffDone;
    bool     mosfetPinValid;
    bool     rtcCapable;
    bool     inCallback;

    uint16_t lowVoltageCounter;
    uint16_t socZeroCounter;
    uint16_t consecutiveReadFails;
    uint8_t  reconnectGrace;

    BatteryState currentState;

    enum class SanityFailReason : uint8_t {
        NONE = 0,
        BUS_STUCK_HIGH,
        BUS_STUCK_LOW,
        VCELL_RANGE,
        SOC_RANGE
    };
    SanityFailReason lastSanityFail;

    BatteryCriticalCb cbCritical;
    BatteryStateCb    cbState;

    float scaleSocForUi(float socRawValue) const {
        float scaled = socRawValue * 100.0f / SOC_UI_FULL_RAW;
        return constrain(scaled, 0.0f, 100.0f);
    }

    bool readRegister16(uint8_t reg, uint16_t &value) {
        Wire.beginTransmission(MAX17048_ADDR);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) {
            while (Wire.available()) Wire.read();
            return false;
        }

        if (Wire.requestFrom((uint8_t)MAX17048_ADDR, (uint8_t)2) != 2) {
            while (Wire.available()) Wire.read();
            return false;
        }

        value = ((uint16_t)Wire.read() << 8) | Wire.read();
        return true;
    }

    bool readBatteryData(uint16_t &rawVcell, uint16_t &rawSocReg) {
        lastSanityFail = SanityFailReason::NONE;

        if (wireMutex == nullptr) {
            return false;
        }

        if (xSemaphoreTake(wireMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            return false;
        }

        bool okV = readRegister16(0x02, rawVcell);
        bool okS = okV && readRegister16(0x04, rawSocReg);

        xSemaphoreGive(wireMutex);

        if (!okV || !okS) {
            return false;
        }

        if (rawVcell == 0xFFFF || rawSocReg == 0xFFFF) {
            lastSanityFail = SanityFailReason::BUS_STUCK_HIGH;
            return false;
        }

        if (rawVcell == 0x0000 && rawSocReg == 0x0000) {
            lastSanityFail = SanityFailReason::BUS_STUCK_LOW;
            return false;
        }

        float vcellCheck = ((float)(rawVcell >> 4)) * 0.00125f;
        if (vcellCheck < 2.0f || vcellCheck > 4.5f) {
            lastSanityFail = SanityFailReason::VCELL_RANGE;
            return false;
        }

        float socCheck = (float)(rawSocReg >> 8) + ((rawSocReg & 0xFF) / 256.0f);
        if (socCheck > 105.0f) {
            lastSanityFail = SanityFailReason::SOC_RANGE;
            return false;
        }

        return true;
    }

    void safeInvokeState(BatteryState newState) {
        if (currentState == newState) return;
        currentState = newState;

        if (!cbState) return;
        if (inCallback) {
            Serial.println("[BAT] ERROR: Re-entrant cbState diblokir.");
            return;
        }

        inCallback = true;
        cbState(newState);
        inCallback = false;
    }

    void safeInvokeCritical() {
        if (!cbCritical) {
            if (!criticalNullptrWarned) {
                criticalNullptrWarned = true;
                Serial.println("[BAT] FATAL: hardCutoff terpenuhi tapi cbCritical nullptr!");
            }
            return;
        }

        if (inCallback) {
            Serial.println("[BAT] ERROR: Re-entrant cbCritical diblokir.");
            return;
        }

        inCallback = true;
        cbCritical();
        inCallback = false;
    }

    void handleSensorLost() {
        Serial.println("[BAT] ERR: Sensor tidak merespons >=30 detik!");
        level                 = -1;
        filteredSoc           = 0.0f;
        rawSoc                = 0.0f;
        voltage               = NAN;
        lowVoltageCounter     = 0;
        socZeroCounter        = 0;
        criticalTriggered     = false;
        criticalNullptrWarned = false;
        reconnectGrace        = 0;
        digitalWrite(pinLedRed, LOW);
        safeInvokeState(BatteryState::UNKNOWN);
    }

    void evaluate() {
        if (level < 0) return;

#ifdef BATTERY_VERBOSE
        if (!isnan(voltage)) {
            Serial.printf("[BAT] UI:%d%% | raw:%.2f%% | %.3fV\n",
                          level, rawSoc, voltage);
        }
#endif

        if (level <= criticalLevel) {
            digitalWrite(pinLedRed, HIGH);
            safeInvokeState(BatteryState::LOW_BATTERY);
        } else {
            digitalWrite(pinLedRed, LOW);
            safeInvokeState(BatteryState::NORMAL);
        }
    }

  public:
    BatteryMgr()
        : voltage(NAN),
          filteredSoc(0.0f),
          rawSoc(0.0f),
          level(-1),
          lastUpdate(0),
          pinLedRed(0),
          pinMosfet(0),
          criticalLevel(15),
          cutoffVoltage(3.2f),
          initialized(false),
          criticalTriggered(false),
          criticalNullptrWarned(false),
          sensorLost(false),
          powerOffDone(false),
          mosfetPinValid(false),
          rtcCapable(false),
          inCallback(false),
          lowVoltageCounter(0),
          socZeroCounter(0),
          consecutiveReadFails(0),
          reconnectGrace(0),
          currentState(BatteryState::UNKNOWN),
          lastSanityFail(SanityFailReason::NONE),
          cbCritical(nullptr),
          cbState(nullptr)
    {}

    void begin(uint8_t ledRedPin = 26,
               uint8_t mosfetPin = 27,
               int lowLevel = 15,
               float cutoffVolts = 3.2f) {

        pinLedRed      = ledRedPin;
        pinMosfet      = mosfetPin;
        mosfetPinValid = GPIO_IS_VALID_OUTPUT_GPIO(pinMosfet);
        rtcCapable     = mosfetPinValid &&
                         rtc_gpio_is_valid_gpio((gpio_num_t)pinMosfet);

        if (!mosfetPinValid) {
            Serial.printf("[PWR] ERROR: GPIO%u tidak valid untuk output.\n", pinMosfet);
        } else if (!rtcCapable) {
            Serial.printf("[PWR] WARN: GPIO%u tidak RTC-capable, hold tidak bertahan saat deep sleep.\n",
                          pinMosfet);
        }

        voltage               = NAN;
        filteredSoc           = 0.0f;
        rawSoc                = 0.0f;
        level                 = -1;
        lastUpdate            = 0;
        criticalTriggered     = false;
        criticalNullptrWarned = false;
        sensorLost            = false;
        powerOffDone          = false;
        lowVoltageCounter     = 0;
        socZeroCounter        = 0;
        consecutiveReadFails  = 0;
        reconnectGrace        = 0;
        currentState          = BatteryState::UNKNOWN;
        lastSanityFail        = SanityFailReason::NONE;
        cbCritical            = nullptr;
        cbState               = nullptr;
        inCallback            = false;

        criticalLevel = constrain(lowLevel, 1, 50);
        cutoffVoltage = constrain(cutoffVolts, 2.5f, 4.2f);

        pinMode(pinLedRed, OUTPUT);
        digitalWrite(pinLedRed, LOW);

        if (mosfetPinValid) {
            gpio_hold_dis((gpio_num_t)pinMosfet);
        }
        gpio_deep_sleep_hold_dis();

        if (mosfetPinValid) {
            pinMode(pinMosfet, OUTPUT);
            digitalWrite(pinMosfet, HIGH); // OFF / bypass sesuai hardware kamu
            Serial.println("[PWR] MOSFET OFF (bypass HW) — peripheral tetap dari AP2112.");
        } else {
            Serial.println("[PWR] MOSFET SKIP — pin tidak valid.");
        }

        initialized = true;
    }

    void onCritical(BatteryCriticalCb cb) {
        if (!cb) {
            Serial.println("[BAT] WARN: onCritical(nullptr) — auto shutdown tidak aktif.");
        }
        cbCritical = cb;
    }

    void onState(BatteryStateCb cb) {
        cbState = cb;
    }

    void update() {
#ifdef BATTERY_DUMMY
        if (!initialized) return;
        rawSoc                = 80.0f;
        voltage               = 3.8f;
        filteredSoc           = scaleSocForUi(rawSoc);
        level                 = (int)round(filteredSoc);
        sensorLost            = false;
        criticalTriggered     = false;
        criticalNullptrWarned = false;
        consecutiveReadFails  = 0;
        lowVoltageCounter     = 0;
        socZeroCounter        = 0;
        reconnectGrace        = 0;
        currentState          = BatteryState::NORMAL;
        lastSanityFail        = SanityFailReason::NONE;
        digitalWrite(pinLedRed, LOW);
        return;
#endif

        if (!initialized) {
            Serial.println("[BAT] ERROR: update() dipanggil sebelum begin()!");
            return;
        }

        uint32_t now = millis();
        if ((uint32_t)(now - lastUpdate) < 1000UL) return;

        if ((uint32_t)(now - lastUpdate) > 5000UL) {
            lastUpdate = now;
        } else {
            lastUpdate += 1000UL;
        }

        uint16_t rawVcellReg = 0;
        uint16_t rawSocReg   = 0;

        if (!readBatteryData(rawVcellReg, rawSocReg)) {
            if (consecutiveReadFails < UINT16_MAX) consecutiveReadFails++;

            if (consecutiveReadFails <= SENSOR_LOST_THRESHOLD ||
                consecutiveReadFails % 60 == 0) {
                switch (lastSanityFail) {
                    case SanityFailReason::BUS_STUCK_HIGH:
                        Serial.printf("[BAT] WARN: Register 0xFFFF (%u×)\n", consecutiveReadFails);
                        break;
                    case SanityFailReason::BUS_STUCK_LOW:
                        Serial.printf("[BAT] WARN: Register 0x0000 (%u×)\n", consecutiveReadFails);
                        break;
                    case SanityFailReason::VCELL_RANGE:
                        Serial.printf("[BAT] WARN: VCELL di luar range (raw=0x%04X) (%u×)\n",
                                      rawVcellReg, consecutiveReadFails);
                        break;
                    case SanityFailReason::SOC_RANGE:
                        Serial.printf("[BAT] WARN: SOC >105%% (raw=0x%04X) (%u×)\n",
                                      rawSocReg, consecutiveReadFails);
                        break;
                    default:
                        Serial.printf("[BAT] WARN: Gagal baca MAX17048 (%u×)\n",
                                      consecutiveReadFails);
                        break;
                }
            }

            if (consecutiveReadFails >= SENSOR_LOST_THRESHOLD && !sensorLost) {
                sensorLost = true;
                handleSensorLost();
            }
            return;
        }

        consecutiveReadFails = 0;
        lastSanityFail       = SanityFailReason::NONE;

        voltage = constrain(((float)(rawVcellReg >> 4)) * 0.00125f, 2.0f, 4.5f);
        rawSoc  = constrain(
            (float)(rawSocReg >> 8) + ((rawSocReg & 0xFF) / 256.0f),
            0.0f, 100.0f
        );

        float uiSoc  = scaleSocForUi(rawSoc);
        int   newLevel = (int)round(uiSoc);

        bool reconnecting = sensorLost;
        if (reconnecting) {
            level             = -1;
            lowVoltageCounter = 0;
            socZeroCounter    = 0;

            bool reconnectCritical = (rawSoc <= 1.0f) ||
                                     (voltage <= cutoffVoltage - 0.1f);

            if (reconnectCritical) {
                reconnectGrace = 0;
                socZeroCounter = SOC_ZERO_THRESHOLD - 1;
                Serial.println("[BAT] Sensor reconnect — evaluasi hardCutoff langsung.");
            } else {
                reconnectGrace = RECONNECT_GRACE_CYCLES;
                Serial.printf("[BAT] Sensor reconnect — grace aktif (%u siklus).\n",
                              RECONNECT_GRACE_CYCLES);
            }
        }

        bool skipShutdownLogic = (reconnectGrace > 0);
        if (skipShutdownLogic) {
            reconnectGrace--;
            Serial.printf("[BAT] Grace: %u siklus tersisa | UI:%d%% | raw:%.2f%% | %.3fV\n",
                          reconnectGrace, newLevel, rawSoc, voltage);
        }

        if (!skipShutdownLogic) {
            if (voltage <= cutoffVoltage) {
                if (lowVoltageCounter < VOLT_DEBOUNCE_THRESHOLD)
                    lowVoltageCounter++;
            } else if (voltage >= cutoffVoltage + VOLT_HYST) {
                lowVoltageCounter = 0;
            }

            if (rawSoc <= 1.0f) {
                if (socZeroCounter < SOC_ZERO_THRESHOLD)
                    socZeroCounter++;
            } else {
                socZeroCounter = 0;
            }

            if (criticalTriggered &&
                lowVoltageCounter == 0 &&
                socZeroCounter == 0 &&
                rawSoc > 5.0f) {
                criticalTriggered     = false;
                criticalNullptrWarned = false;
                Serial.println("[BAT] Kondisi kritis hilang — flag reset.");
            }

            bool rawSocCritical = (socZeroCounter >= SOC_ZERO_THRESHOLD);
            bool hardCutoff     = rawSocCritical ||
                                  (lowVoltageCounter >= VOLT_DEBOUNCE_THRESHOLD &&
                                   rawSoc <= 5.0f);

            if (hardCutoff) {
                digitalWrite(pinLedRed, HIGH);  // LED menyala sebagai indikator kritis

                if (reconnecting) sensorLost = false;

                level       = max(0, newLevel);
                filteredSoc = (float)level;

                if (!criticalTriggered) {
                    criticalTriggered = true;

                    if (rawSocCritical) {
                        Serial.printf("[BAT] KRITIS: SOC raw <=1.0%% selama %u siklus\n",
                                      socZeroCounter);
                    }
                    if (lowVoltageCounter >= VOLT_DEBOUNCE_THRESHOLD &&
                        rawSoc <= 5.0f) {
                        Serial.printf("[BAT] KRITIS: %.3fV selama %u siklus (raw SOC %.2f%%)\n",
                                      voltage, lowVoltageCounter, rawSoc);
                    }

                    safeInvokeState(BatteryState::UNKNOWN);
                    Serial.println("[BAT] KRITIS: memanggil cbCritical()...");

                    safeInvokeCritical();  // HANYA dipanggil sekali saat pertama kali hardCutoff
                }
                return;
            }
        }

        if (level < 0 || isnan(filteredSoc)) {
            filteredSoc = (float)newLevel;
        } else {
            filteredSoc = filteredSoc * 0.75f + (float)newLevel * 0.25f;
        }
        level = (int)round(filteredSoc);

        if (reconnecting) sensorLost = false;

        evaluate();
    }

    void powerOff() {
        if (!initialized) {
            Serial.println("[PWR] ERROR: powerOff() dipanggil sebelum begin()!");
            return;
        }

        if (powerOffDone) return;
        powerOffDone = true;

        digitalWrite(pinLedRed, LOW);

        if (!mosfetPinValid) {
            Serial.printf("[PWR] WARN: GPIO%u tidak valid — powerOff di-skip.\n", pinMosfet);
            return;
        }

        pinMode(pinMosfet, OUTPUT);
        digitalWrite(pinMosfet, HIGH);

        if (rtcCapable) {
            gpio_hold_en((gpio_num_t)pinMosfet);
            gpio_deep_sleep_hold_en();
            Serial.println("[PWR] MOSFET OFF + RTC hold aktif.");
        } else {
            Serial.printf("[PWR] WARN: GPIO%u tidak RTC-capable — hold deep sleep tidak bertahan.\n",
                          pinMosfet);
        }
    }

    int          getLevel()             const { return level; }
    float        getVoltage()           const { return voltage; }
    float        getRawSoc()            const { return rawSoc; }
    uint16_t     getLowVoltageCounter() const { return lowVoltageCounter; }
    uint16_t     getSocZeroCounter()    const { return socZeroCounter; }
    bool         isSensorLost()         const { return sensorLost; }
    bool         isStabilizing()        const { return reconnectGrace > 0; }
    bool         isCriticalTriggered()  const { return criticalTriggered; }
    bool         isReady()              const { return initialized; }
    bool         isRtcCapable()         const { return rtcCapable; }
    BatteryState getState()             const { return currentState; }

    void draw(Adafruit_SSD1306 &display) {
        const int x = 112, y = 0, w = 14, h = 8;
        int shownLevel = (level < 0) ? -1 : constrain(level, 0, 100);

        display.drawRect(x, y, w, h, SSD1306_WHITE);
        display.fillRect(x + w, y + 2, 2, 4, SSD1306_WHITE);

        if (shownLevel > 0) {
            int fillWidth = max(1,
                (int)round((float)shownLevel / 100.0f * (float)(w - 2))
            );
            display.fillRect(x + 1, y + 1, fillWidth, h - 2, SSD1306_WHITE);
        }

        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(90, 0);

        if (shownLevel < 0) {
            display.print("--");
        } else {
            display.print(shownLevel);
            display.print("%");
        }
    }
};

#endif // BATTERY_MGR_H
