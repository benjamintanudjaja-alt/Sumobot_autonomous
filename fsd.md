# Functional Specification Document (FSD)
## Autonomous Sumobot - ESP32 & VL53L1X

---

### 1. Overview
Dokumen spesifikasi fungsional ini menjelaskan konfigurasi perangkat keras, pemetaan pin (pinout), spesifikasi & konfigurasi sensor jarak VL53L1X Time-of-Flight (ToF), parameter tuning, serta logika pengambilan keputusan (*decision engine*) untuk robot Sumobot Otonom berbasis ESP32.

---

### 2. Hardware Pinout

#### 2.1. Motor Driver (DRV8833 Dual H-Bridge)
Penggerak motor menggunakan modul driver DRV8833 yang dikontrol menggunakan sinyal PWM (ESP32 LEDC peripheral).

| ESP32 GPIO | Nama Pin Driver | Fungsi | Keterangan |
| :--- | :--- | :--- | :--- |
| **GPIO 16** | `PIN_INM11` | Motor Kiri - IN1 | Forward PWM (Kiri) |
| **GPIO 17** | `PIN_INM12` | Motor Kiri - IN2 | Reverse PWM (Kiri) |
| **GPIO 27** | `PIN_INM21` | Motor Kanan - IN1 | Forward PWM (Kanan) |
| **GPIO 26** | `PIN_INM22` | Motor Kanan - IN2 | Reverse PWM (Kanan) |

*Catatan Motor Direction:*
- `INVERT_LEFT_MOTOR`: `false`
- `INVERT_RIGHT_MOTOR`: `true` (arah motor kanan dibalik secara software sesuai pemasangan mekanis)

#### 2.2. I2C Bus Komunikasi Sensor
Komunikasi dengan ketiga sensor VL53L1X menggunakan shared I2C bus (Bus 0 pada ESP32).

| ESP32 GPIO | Sinyal I2C | Deskripsi |
| :--- | :--- | :--- |
| **GPIO 21** | `I2C0_SDA` | I2C Data Line (Frekuensi: 400 kHz / Fast Mode) |
| **GPIO 22** | `I2C0_SCL` | I2C Clock Line |

#### 2.3. XSHUT Pins (Kontrol Power / Reset Sensor VL53L1X)
Digunakan untuk sekuens inisialisasi dan pengalamatan I2C individual saat boot.

| Sensor | Posisi Sensor | GPIO ESP32 | Default I2C Addr | Runtime I2C Addr |
| :--- | :--- | :--- | :--- | :--- |
| Sensor 0 | **Kiri (`SENSOR_LEFT`)** | **GPIO 25** | `0x29` | `0x2A` |
| Sensor 1 | **Tengah (`SENSOR_CENTER`)** | **GPIO 33** | `0x29` | `0x2B` |
| Sensor 2 | **Kanan (`SENSOR_RIGHT`)** | **GPIO 32** | `0x29` | `0x2C` |

---

### 3. Sensor Specification & Configuration

#### 3.1. Sensor yang Digunakan
- **Model:** VL53L1X Time-of-Flight (ToF) Laser Ranging Sensor (Pololu Library `pololu/VL53L1X @ ^1.3.1`).
- **Jumlah:** 3 unit (Kiri, Depan Tengah, Kanan).

#### 3.2. Prosedur Inisialisasi Alamat I2C (Sequential Bootstrapping)
Karena semua modul VL53L1X memiliki alamat I2C default pabrik yang sama (`0x29`), urutan aktivasi dilakukan bertahap:
1. Seluruh pin XSHUT diset ke `OUTPUT LOW` (menahan semua sensor dalam mode shutdown).
2. Tunda sejenak (10 ms).
3. Untuk setiap sensor satu per satu:
   - Lepas status reset dengan mengubah XSHUT menjadi `INPUT` (membiarkan pull-up eksternal mengaktifkan sensor).
   - Tunggu 10 ms hingga sensor siap.
   - Panggil `sensor.init()` di alamat default `0x29`.
   - Ubah alamat sensor ke alamat dinamis (`0x2A`, `0x2B`, atau `0x2C`) via `sensor.setAddress()`.

#### 3.3. Pengaturan Ranging & Optical Filter
- **Distance Mode:** `VL53L1X::Short`
  - Mengoptimalkan ketahanan terhadap *ambient light* dan meningkatkan akurasi jarak pendek untuk arena sumobot.
- **Measurement Timing Budget:** `20000 µs` (20 ms).
  - Waktu alokasi pembacaan pulsa laser per pengukuran.
- **Continuous Sampling:** Interval `20 ms` via `sensor.startContinuous(20)`.
- **Field of View (ROI) Configuration:**
  - `setROISize(16, 8)`: Membatasi zona pembacaan SPAD menjadi 16 horizontal x 8 vertikal (50% dari tinggi sensor).
  - `setROICenter(60)`: Menggeser pusat SPAD ke baris bawah sensor optik.
  - *Tujuan:* Lensa sensor membalik bayangan (*inverted optics*). Memilih SPAD bagian bawah berarti membatasi FoV fisik hanya untuk area 50% ke atas, menghindari pantulan palsu dari lantai dohyo/garis arena.

---

### 4. Requirements & Tuning Variables

#### 4.1. Jarak Deteksi Musuh (Thresholds)
| Parameter | Nilai Default | Satuan | Deskripsi |
| :--- | :--- | :--- | :--- |
| `ENEMY_MIN_DIST_MM` | `1` | mm | Batas jarak minimum valid (mengabaikan pembacaan 0 mm/error). |
| `ENEMY_MAX_DIST_MM` | `400` | mm | Jarak maksimum musuh dianggap terdeteksi di arena (40 cm). |

> Nilai di luar rentang `[1 - 400] mm` atau saat sensor mengalami timeout dianggap sebagai **tidak terdeteksi** (`dist = 0`).

#### 4.2. Pengaturan Kecepatan Motor (PWM Duty Cycle: 0 - 255)
| Parameter | Nilai Default | Deskripsi |
| :--- | :--- | :--- |
| `SPEED_ATTACK_FULL` | `55` | Kecepatan maju lurus penuh saat musuh berada tepat di depan. |
| `SPEED_ATTACK_OUTER` | `50` | Kecepatan roda luar saat manuver serang + belok melengkung. |
| `SPEED_ATTACK_INNER` | `45` | Kecepatan roda dalam saat manuver serang + belok melengkung. |
| `SPEED_TURN_IN_PLACE` | `90` | Kecepatan saat berputar di tempat (kedua roda berputar berlawanan arah). |

#### 4.3. PWM Driver Settings
- **PWM Frequency:** `20,000 Hz` (20 kHz) — frekuensi ultrasonik, tidak menimbulkan dengung berisik (*whining sound*) pada motor.
- **PWM Resolution:** `8-bit` (Range duty cycle: 0 – 255).
- **LEDC Channels:** Channel 0 (IN11), Channel 1 (IN12), Channel 2 (IN21), Channel 3 (IN22).

#### 4.4. Timing Requirements
- `SENSOR_READ_INTERVAL_MS`: `25 ms` (interval pengambilan sampel sensor secara non-blocking; harus $\ge$ timing budget 20 ms).
- **Serial Debug Interval:** `100 ms` (menghindari bottleneck transmisi UART pada kecepatan `921600 baud`).

---

### 5. Autonomous Decision Logic (State Machine)

Pengambilan keputusan didasarkan pada kombinasi deteksi ketiga sensor:
- $L = \text{Sensor Kiri terdeteksi } (1 \le dist_L \le 400\text{ mm})$
- $C = \text{Sensor Tengah terdeteksi } (1 \le dist_C \le 400\text{ mm})$
- $R = \text{Sensor Kanan terdeteksi } (1 \le dist_R \le 400\text{ mm})$

| Kondisi Sensor ($L, C, R$) | State Robot | Aksi Motor Kiri | Aksi Motor Kanan | Deskripsi Gerakan |
| :---: | :--- | :---: | :---: | :--- |
| **0, 0, 0** | `STATE_IDLE` | `0` | `0` | **Diam / Stop** (tidak bergerak sampai musuh terdeteksi). |
| **0, 1, 0** | `STATE_ATTACK_FULL` | `+55` | `+55` | **Maju lurus** menyerang musuh di depan. |
| **1, 1, 1** | `STATE_ATTACK_FULL` | `+55` | `+55` | **Maju lurus** mendobrak (musuh dekat & lebar). |
| **1, 0, 1** | `STATE_ATTACK_FULL` | `+55` | `+55` | **Maju lurus** (musuh simetris di kedua sisi). |
| **1, 1, 0** | `STATE_ATTACK_LEFT` | `+45` | `+50` | **Maju melengkung ke kiri** (musuh agak condong ke kiri). |
| **0, 1, 1** | `STATE_ATTACK_RIGHT` | `+50` | `+45` | **Maju melengkung ke kanan** (musuh agak condong ke kanan). |
| **1, 0, 0** | `STATE_TURN_LEFT` | `-90` | `+90` | **Putar di tempat ke kiri** mencari/mengunci posisi musuh. |
| **0, 0, 1** | `STATE_TURN_RIGHT` | `+90` | `-90` | **Putar di tempat ke kanan** mencari/mengunci posisi musuh. |
