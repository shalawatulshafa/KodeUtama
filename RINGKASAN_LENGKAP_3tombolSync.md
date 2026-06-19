# Ringkasan Lengkap & Dokumen Handoff — Firmware 3tombolSync

> **CARA PAKAI DOKUMEN INI (untuk AI lain):**
> Tempelkan seluruh isi file ini sebagai konteks awal. Dokumen ini menjelaskan tujuan proyek, arsitektur, cara kerja tiap sub-sistem, seluruh bug yang pernah ditemukan + akar masalah + solusinya, dan daftar pekerjaan yang masih tersisa. Setelah menempel, AI lain bisa langsung melanjutkan tanpa perlu menebak konteks.
>
> **Aturan penting yang WAJIB dipatuhi AI mana pun yang menyentuh proyek ini:**
> 1. `BatteryMgr.h` adalah **file milik anggota tim lain — JANGAN diubah**.
> 2. `StepSensor.cpp/.h` adalah adaptasi dari kode teman (`sensor_lari.ino`) — algoritmanya jangan diutak-atik, hanya boleh perbaiki bug porting.
> 3. `TESTING_USB_ONLY` harus **DINONAKTIFKAN** di produksi.
> 4. Brownout detector harus **tetap aktif** (jangan `WRITE_PERI_REG` untuk mematikannya).
> 5. **Prioritas user = STABILITAS DEMO di atas estetika kode.** Jangan refactor besar; lakukan perubahan minimal yang aman.
> 6. File utama produksi = **`3tombolSync.ino`**. `adabatrai.ino` hanya referensi (varian baterai), jangan diubah kecuali diminta.

---

## 0. IDENTITAS PROYEK

- **Nama:** 3tombolSync — *breath-step coupling coach* (alat pelatih sinkronisasi napas-langkah untuk lari).
- **Platform:** ESP32 (dual-core, FreeRTOS), Arduino framework.
- **Fungsi inti:** Memutar panduan napas audio (HIRUP / BUANG) lewat **Bluetooth A2DP** ke earphone, **disinkronkan dengan irama langkah (cadence)** pelari, lalu **merekam data sesi** ke flash dan **mengirimnya ke aplikasi HP (Flutter) via BLE**.
- **Pola napas didukung:** **3:2** (3 langkah HIRUP, 2 langkah BUANG) dan **2:1** (2 langkah HIRUP, 1 langkah BUANG).
- **Konteks:** Tugas Akhir (skripsi) mahasiswa. Butuh keandalan saat demo.

### Daftar file penting
| File | Peran |
|---|---|
| `3tombolSync.ino` | **Firmware produksi utama** (~2580 baris) |
| `StepSensor.cpp` / `StepSensor.h` | Deteksi langkah (MPU6050) — adaptasi dari kode teman |
| `BatteryMgr.h` | Manajemen baterai MAX17048 — **MILIK TIM, jangan ubah** |
| `BreathSensor.h` | Sensor napas |
| `adabatrai.ino` | Varian referensi (punya fitur kirim baterai ke HP) |
| `sensor_lari.ino` | **Kode ASLI deteksi langkah dari teman** (pakai library Adafruit_MPU6050) |
| `sensor_lari_last.ino` | Varian kode teman (manual Wire, ada komentar kompensasi 132ms) |
| `Laporan_2.7_2.8_Panduan_Antarmuka.md` | Naskah laporan TA bagian 2.7 & 2.8 |
| `flowchart_gen.py` + `flowcharts/` | 6 flowchart hitam-putih untuk laporan |

---

## A. CARA KERJA KODE (PENJELASAN MENYELURUH)

### A.1 Konsep besar: kenapa proyek ini sulit
Dua radio Bluetooth dipakai bergantian:
- **A2DP (Bluetooth Classic)** → *streaming audio* panduan napas (library `BluetoothA2DPSource` by pschatzmann, codec SBC).
- **BLE (Bluetooth Low Energy)** → *sinkronisasi data* ke aplikasi HP Flutter.

ESP32 **tidak stabil jika A2DP & BLE hidup bersamaan dalam satu boot**. Seluruh arsitektur dibangun di atas satu prinsip:

> **Single-Mode-Per-Boot** — dalam satu kali boot, ESP32 hanya menjalankan SATU mode radio. Untuk pindah mode, perangkat **restart** (`ESP.restart()`) dengan `boot_mode` disimpan di NVS — BUKAN mematikan/menyalakan radio secara live.

State penting harus bertahan melewati restart, lewat:
- **RTC memory** (bertahan saat deep sleep)
- **NVS / Preferences** (bertahan saat power-off / restart) — menyimpan `boot_mode`, `session_id`, waktu.

### A.2 Alur boot (`setup()`)
1. Baca `reset_reason` dari ESP32.
2. Tentukan `booted_from_crash` — **hanya `true` untuk crash**: `ESP_RST_PANIC` (4), `ESP_RST_INT_WDT`, `ESP_RST_TASK_WDT`, `ESP_RST_WDT`. Restart sengaja (pindah mode) TIDAK dihitung crash.
   - Kode reset: 1=POWERON, 4=PANIC, 5/6/7=WDT, 8=DEEPSLEEP, 9=BROWNOUT.
3. Baca `boot_mode` dari NVS → tentukan mode (A2DP guiding / BLE sync / deep sleep).
4. Jika `booted_from_crash == true` → tampilkan layar **RECONNECT / "Pilih Scan"** dan **batalkan `is_mac_configured`** (paksa re-pair) sebagai pengaman.
5. Inisialisasi peripheral (OLED, I2C, sensor) → masuk mode sesuai `boot_mode`.

> **PENTING:** Gejala "layar berkedip → RECONNECT / Pilih Scan" = perangkat **baru saja CRASH** (`reset_reason=4`), BUKAN fitur normal. Layar berkedip = reboot.

### A.3 Task FreeRTOS (dua core)
- **loopTask** (Core 1, prio 1): logika utama — state machine panduan, penjadwalan audio, logging, BLE sync.
- **SensorTask** (prio 2): baca sensor terus-menerus.
  ```cpp
  xTaskCreatePinnedToCore(SensorTask, "SensorTask", 16384, NULL, 2, &SensorTaskHandle, 0);
  ```
  **Saat ini di-pin ke Core 0** (argumen terakhir = `0`).
- **esp_timer** (prio ~22): timer presisi untuk memicu audio cue.
- **BT controller task**: prio tinggi, dikelola sistem.

Bus I2C (OLED + MPU6050 berbagi bus) dilindungi **`wireMutex`** agar transaksi tidak tabrakan antar-task.

### A.4 Deteksi langkah (StepSensor.cpp/.h)
Sumber: **MPU6050** (±4G, DLPF 10Hz). Pipeline per sampel (laju **100Hz**):
1. Baca ax, ay, az via I2C (dengan `wireMutex`). Jika baca gagal/parsial → **buang sampel** (jangan inject 0g).
2. Magnitudo: `raw_mag = sqrt(ax²+ay²+az²)`.
3. **Band-Pass Filter IIR orde-4** → saring frekuensi langkah, buang gravitasi (DC) & getaran tinggi. (group delay ~132ms.)
4. **Deteksi peak** jendela 3-sampel: `curr > prev && curr > next`.
5. **Ambang adaptif**: rata-rata 5 puncak terakhir × `THRESHOLD_FACTOR (0.40)`, dibatasi `THRESHOLD_MIN (0.12g)` … `THRESHOLD_MAX (2.5g)`.
6. **Validasi**: lewat *dynamic mask* (refractory `0.40×periode`, 230–600ms) DAN `raw_mag < IMPACT_LIMIT_G (4.5g)`.
7. Valid → `total_steps++`, hitung **SPM** (EMA, `SPM_ALPHA=0.2`), set `stepTriggered=true`.
8. **Decay saat diam** (>650ms): ambang & SPM turun perlahan.

### A.5 Penjadwalan audio (jantung sinkronisasi) — 3tombolSync.ino
Cue harus sampai di telinga **tepat sebelum** langkah target, mengompensasi latensi sistem. Rumus:
```
time_to_trigger = N × T_avg − (T_LEAD_MS + L_SYS)
```
- `T_avg` = rata-rata periode langkah (SMA/EMA).
- `N` = `sched_steps_ahead` (1 atau 2 langkah ke depan, adaptif).
- `T_LEAD_MS = 170` → lead di telinga (spek).
- `L_SYS_1 = 168` (1-langkah: ~18ms buffer firmware + ~150ms SBC); `L_SYS_2 = 158` (2-langkah, −10ms karena error prediksi ter-amplifikasi ×2).

**Adaptif N dengan histeresis** (cegah flapping):
```cpp
long comp1 = T_LEAD_MS + L_SYS_1;            // 338ms
if      (T_avg < comp1 - 18) sched_steps_ahead = 2;  // ~>187 SPM → 2 langkah
else if (T_avg > comp1 - 8)  sched_steps_ahead = 1;  // ~<183 SPM → 1 langkah
```
Timer di-`arm` **paling awal** (sebelum file I/O) agar tulis file yang lambat tidak menggeser jadwal audio.

### A.6 State machine panduan
- **ANALYZING** → amati irama. `stable_count` naik tiap langkah teratur (toleransi ~20%). `stable_count > 5` → **GUIDING**.
- **GUIDING** → audio cue + perekaman aktif. Tiap langkah meleset, `missed_step_count++`.
  - `≤ 3` → re-arm timer (toleransi sesaat).
  - `> 3` → **PAUSE**: `stable_count=0`, balik ANALYZING, `recording_active=false`.
- Ritme stabil lagi → kembali GUIDING + **RESUME** perekaman (lihat B.2).

### A.7 Perekaman (format BINER) & sync ke HP
- **GUIDING pertama** (`guiding_ever_started`): naikkan `session_id` (NVS), tulis **SESSION header 9-byte** ke `/run.dat` = `0xFF magic + session_id(4B) + unix(4B)`. Set `recording_active=true`, catat `recording_start_ms`.
- **Tiap langkah GUIDING**: tulis **record 6-byte** = `ts_ms(4B) + flags/breath_phase(1B) + spm(1B)`.
- **Saat sync (BLE)**: kirim header CSV → konversi biner→CSV → kirim **EOF** → tunggu **ACK** (`OK`/`DONE`/`ACK`/`SELESAI`/`OKE`) ≤30 detik. Tanpa ACK → **file TIDAK dihapus** (data aman).

### A.8 Baterai (BatteryMgr.h — JANGAN UBAH)
- IC: **MAX17048** fuel gauge via I2C.
- `onCritical` fired saat `level==0 || voltage<=cutoffVoltage(3.2V)`. (Punya 2 guard: tolak v<2.5V + konfirmasi 2× berturut.)
- `onLow` fired saat `level <= 15%`.
- Jika baca I2C gagal, register tinggal 0 → bisa **false-trigger** (lihat B.3).

---

## B. SEMUA PERUBAHAN & PERBAIKAN (RIWAYAT LENGKAP)

### B.1 Crash Bluetooth (riwayat panjang — SUDAH TERATASI) ✅
| # | Gejala | Akar masalah | Solusi sekarang |
|---|---|---|---|
| 1 | Crash saat transisi A2DP↔BLE | Controller BTDM tak boleh ganti mode live | **Single-Mode-Per-Boot**: `ESP.restart()` + `boot_mode` di NVS; waktu via RTC memory + counter NVS |
| 2 | *use-after-free* saat mau sleep | `a2dp_source.end()` membebaskan resource yang masih dipakai | **Jangan panggil `end()`** — restart ke `BOOT_DEEP_SLEEP` untuk tidur bersih |
| 3 | **Stack canary watchpoint (ipc0)** = stack overflow | Init BT controller berebut Core 0 dengan SensorTask | Pernah dicoba: pindah SensorTask ke Core 1. **Status: file = Core 0 lagi (fix tidak aktif).** Jika ipc0 muncul lagi, ubah arg terakhir `0`→`1` di baris `xTaskCreatePinnedToCore(SensorTask,...)`. |

*Fingerprint crash ipc0:* register dump berisi pola `0xa5a5a5a5`, backtrace semua di `0x4009xxxx` (IRAM/system), ada `|<-CORRUPTED`.

### B.2 Perbaikan RESUME perekaman (FIX UTAMA — AKTIF) ✅
**Masalah:** Lari 30 menit, audio terdengar penuh, tapi data terekam **hanya ~5 menit.**
**Akar masalah:** `recording_active` hanya di-set `true` sekali (di blok `if (!guiding_ever_started)`). Begitu panduan PAUSE sekali (>3 langkah meleset), `recording_active=false` **permanen** walau panduan jalan lagi. Audio tetap bunyi karena `audioTimerCallback` tidak cek `recording_active`.
**Perbaikan (3tombolSync.ino, ~baris 658):**
```cpp
} else if (!recording_active && !is_pattern_switching) {
  // RESUME: panduan sempat PAUSE lalu ritme stabil lagi.
  // Lanjut di sesi SAMA: JANGAN tulis header baru, JANGAN reset recording_start_ms.
  recording_active = true;
  logging_enabled  = true;
  Serial.println("RECORDING RESUMED");
}
```
*Riwayat:* fix ini sempat dicurigai bikin "data gagal terkirim" → di-revert untuk tes → ternyata penyebab gagal kirim = **mouse Bluetooth laptop** (interferensi BLE), bukan kode → fix dikembalikan & **aktif**.

### B.3 "Segera charge" palsu di 3.7–3.8V (DIDIAGNOSIS — belum diperbaiki) ⏳
**Gejala:** Muncul "LOW BATTERY / Segera charge!" padahal tegangan masih 3.7–3.8V (sehat).
**Akar masalah:** `onLow` fired saat `level ≤ 15%`. Jika baca I2C MAX17048 misread → `level=0` → ≤15% → alarm palsu. `onLow` **belum punya guard tegangan**.
**Usulan fix (belum diterapkan):**
```cpp
battery.onLow([](bool isLow){
  if (isLow && battery.getVoltage() > 3.45f) return;  // tegangan sehat → abaikan
  if (isLow) updateDisplay("LOW BATTERY", "Segera charge!", false);
});
```

### B.4 Langkah "hantu" saat MPU6050 diam (SUDAH DIPERBAIKI) ✅
**Gejala:** Terhitung langkah padahal sensor diam / SPM fluktuatif.
**Akar masalah + perbaikan yang sudah dilakukan:**
1. **Inject 0g ke filter** — saat baca I2C gagal, ax/ay/az tetap 0 → `raw_mag=0` (mustahil fisik) → band-pass *ringing* → puncak palsu.
   → **FIX:** di `StepSensor.cpp::update()`, jika `Wire.available() != 6` → `xSemaphoreGive(wireMutex); return;` (buang sampel, jangan proses).
2. **Sampling rate salah** — `SAMPLE_PERIOD_US=6250` (160Hz) padahal koefisien BPF didesain untuk 100Hz → passband bergeser.
   → **FIX:** `StepSensor.h` → `SAMPLE_PERIOD_US = 10000UL` (100Hz, sesuai kode asli teman).
3. `THRESHOLD_MIN=0.12g` agak sensitif saat diam → **belum diubah** (algoritma teman, dibiarkan; cukup fix #1 & #2 yang mengurangi phantom step).

### B.5 Perbandingan kode langkah ASLI (teman) vs StepSensor (porting)
Sumber asli: `sensor_lari.ino` (pakai library `Adafruit_MPU6050`, baca `mpu.getEvent`).
StepSensor.cpp = adaptasi: **baca MPU6050 manual via `Wire`** (karena library Adafruit tidak bisa dipakai di setup user) + dibungkus class + `wireMutex` untuk multi-task.
- **Cara baca manual = AMAN & ekuivalen** dengan library: konversi `AcX/8192.0` (±4G) sama hasilnya dengan `a.acceleration.x/9.80665`. Register init (0x6B wake, 0x1C range, 0x1A DLPF) identik dengan yang library lakukan.
- Algoritma BPF/peak/threshold/mask/hysteresis/SPM = **identik kata-per-kata** dengan kode asli.
- Satu-satunya bug porting = `SAMPLE_PERIOD_US` (sudah diperbaiki di B.4).
- Kode asli punya komentar "kompensasi delay 132ms" tapi `actual_step_time` tak pernah dipakai → tidak berpengaruh, tak perlu di-port.

### B.6 Fitur kirim baterai ke HP (BARU DI-PORT — AKTIF) ✅
**Konteks:** `adabatrai.ino` bisa kirim level baterai ke aplikasi HP; `3tombolSync.ino` semula tidak. Fitur ini **di-port ke `3tombolSync.ino`** (tanpa mengubah `adabatrai.ino`). Tiga perubahan:
1. Deklarasi global: `BLECharacteristic *pBattCharacteristic = nullptr;`
2. Di `initBLEForSync()`: buat **Battery Service standar BLE (UUID 0x180F)** + Characteristic (**0x2A19**, READ+NOTIFY) + `BLE2902` descriptor; isi level awal; daftarkan UUID 0x180F ke advertising.
3. Di `loop()` mode `STATE_SYNC`: `notify()` level baterai ke HP **tiap 5 detik** selama sesi sync.
> UUID 0x180F adalah **Battery Service standar Bluetooth**, langsung dikenali Flutter tanpa kode khusus.

### B.7 Dokumen & aset laporan TA
- `Laporan_2.7_2.8_Panduan_Antarmuka.md` — naskah Sub-Sistem Panduan (2.7) & Antarmuka Lokal (2.8), gaya akademis mengikuti format 2.5: tabel perbandingan desain↔implementasi, cuplikan kode, tabel verifikasi, riwayat iterasi.
- `flowchart_gen.py` + `flowcharts/` — 6 flowchart **hitam-putih**: `fc1_analisis_penjadwalan.png` (Gbr 2.7-1), `fc1b_langkah_meleset.png`, `fc2_pemicu_cue.png`, `fc3_render_nada.png`, `fc4_state_machine.png` (Gbr 2.8-1), `fc5_input_tombol.png` (Gbr 2.8-2).
- `B400_extracted.txt` — ekstraksi teks 58 halaman B400.pdf (via pypdf).

### B.8 Catatan teknis lain yang masih perlu diperhatikan
- **Fitur baterai-ke-HP** sudah di-port (B.6).
- `THRESHOLD_MIN=0.12g` dibiarkan (lihat B.4 #3).

---

## C. CATATAN OPERASIONAL UNTUK DEMO

1. **Saat sync ke HP**, matikan/lepas perangkat Bluetooth lain (mouse, headset) di laptop & HP — radio BT berebut bisa memutus koneksi BLE di tengah (gejala: data terkirim + "EOF terkirim" tapi `HP Terputus` tanpa ACK = masalah koneksi HP, BUKAN kode).
2. **Sync waktu (jam) DULU sebelum lari.** Data bertanggal `01/01/1970` = direkam sebelum waktu valid (`time_is_valid=false`).
3. **Layar berkedip → "RECONNECT/Pilih Scan"** = perangkat baru CRASH. Cek serial: `reset_reason=4` (PANIC) / `5,6,7` (WDT).
4. **Produksi:** `TESTING_USB_ONLY` DINONAKTIFKAN; brownout detector AKTIF; `BatteryMgr.h` jangan diubah.

---

## D. STATUS RINGKAS SEMUA PERBAIKAN

| Item | Status |
|---|---|
| Crash A2DP↔BLE (single-mode-per-boot) | ✅ Teratasi |
| use-after-free saat sleep (restart-to-sleep) | ✅ Teratasi |
| RESUME perekaman (data tak hilang setelah pause) | ✅ Aktif |
| Sync gagal karena mouse BT | ✅ Terbukti eksternal, bukan kode |
| Phantom step — inject 0g I2C | ✅ Diperbaiki (skip sampel invalid) |
| Phantom step — sampling 160Hz→100Hz | ✅ Diperbaiki (`SAMPLE_PERIOD_US=10000`) |
| Fitur baterai-ke-HP (BLE 0x180F/0x2A19) | ✅ Di-port ke 3tombolSync.ino |
| SensorTask → Core 1 (anti crash ipc0) | ⚠️ Tidak aktif (file = Core 0) |
| Guard tegangan `onLow` ("Segera charge" palsu) | ⏳ Belum diterapkan (usulan di B.3) |
| `THRESHOLD_MIN` 0.12→0.30g (opsional) | ⏳ Belum diterapkan (algoritma teman) |

---

## E. PEKERJAAN TERSISA (untuk dilanjutkan AI lain / sesi berikutnya)

1. **(Opsional, aman)** Terapkan guard tegangan `onLow` di B.3 agar "Segera charge" tidak muncul palsu saat baterai masih 3.7–3.8V.
2. **(Kondisional)** Jika crash `ipc0` (stack canary) muncul lagi → pindah SensorTask ke Core 1: ubah argumen terakhir `xTaskCreatePinnedToCore(SensorTask, ..., 0)` menjadi `1`.
3. **(Laporan TA)** User masih perlu: isi data timing nyata di tabel verifikasi, gambar/sisipkan flowchart, tambah foto perangkat.
4. **(Opsional)** Pertimbangkan naikkan `THRESHOLD_MIN` jika phantom step masih muncul setelah fix B.4.

> Semua perubahan harus **minimal & aman** — prioritas user adalah stabilitas demo. Konfirmasi dulu sebelum mengubah `StepSensor.*` (kode teman) atau menyentuh `BatteryMgr.h` (jangan).
