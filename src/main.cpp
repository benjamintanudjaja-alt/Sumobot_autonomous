#include <Arduino.h>
#include <Wire.h>
#include <VL53L1X.h>

// =============================================================================
// TUNING VARIABLES - Ubah nilai di sini untuk tuning perilaku robot
// =============================================================================

// --- Sensor ---
const uint16_t ENEMY_MAX_DIST_MM        = 400;  // Jarak maksimum musuh dianggap terdeteksi (mm)
const uint16_t ENEMY_MIN_DIST_MM        = 1;    // Jarak minimum valid (abaikan 0 mm)

// --- Kecepatan Motor (0 - 255) ---
const int SPEED_ATTACK_FULL             = 55;  // Kecepatan maju lurus saat musuh tepat di depan
const int SPEED_ATTACK_OUTER            = 50;  // Kecepatan roda luar saat maju + belok
const int SPEED_ATTACK_INNER            = 45;  // Kecepatan roda dalam saat maju + belok
const int SPEED_TURN_IN_PLACE           = 90;  // Kecepatan belok di tempat (roda berlawanan)

// --- Timing ---
const unsigned long SENSOR_READ_INTERVAL_MS = 25; // Interval baca sensor (ms), minimum = timing budget

// =============================================================================
// PIN CONFIGURATION
// =============================================================================

// Motor Driver DRV8833
#define PIN_INM11  16   // Motor Kiri  IN1
#define PIN_INM12  17   // Motor Kiri  IN2
#define PIN_INM21  27   // Motor Kanan IN1
#define PIN_INM22  26   // Motor Kanan IN2

// Invert motor direction jika wiring terbalik secara mekanis
#define INVERT_LEFT_MOTOR   false
#define INVERT_RIGHT_MOTOR  true

// I2C Bus 0 (hanya Bus 0 yang digunakan, Bus 1 tidak dipasang)
#define I2C0_SDA  21
#define I2C0_SCL  22

// XSHUT pins: masing-masing mengontrol 1 sensor
// Posisi: Pos0 = Kiri, Pos1 = Tengah, Pos2 = Kanan
#define SENSOR_LEFT    0
#define SENSOR_CENTER  1
#define SENSOR_RIGHT   2

const uint8_t SENSOR_COUNT              = 3;
const uint8_t xshutPins[SENSOR_COUNT]   = {25, 33, 32};
const uint8_t sensorAddresses[SENSOR_COUNT] = {0x2A, 0x2B, 0x2C};

// =============================================================================
// PWM CONFIGURATION
// =============================================================================
const int PWM_FREQ       = 20000;  // 20 kHz ultrasonic (senyap)
const int PWM_RESOLUTION = 8;      // 8-bit (0-255)

// LEDC Channels (ESP32 Arduino Core v2.x)
const int CH_M11 = 0;
const int CH_M12 = 1;
const int CH_M21 = 2;
const int CH_M22 = 3;

// =============================================================================
// SENSOR STATE
// =============================================================================
TwoWire I2CBus0 = TwoWire(0);
VL53L1X sensors[SENSOR_COUNT];
bool sensorConnected[SENSOR_COUNT] = {false, false, false};

// Valid distance reading per sensor (0 = tidak terdeteksi / tidak valid)
uint16_t dist[SENSOR_COUNT] = {0, 0, 0};

// =============================================================================
// MOTOR DRIVER HELPERS
// =============================================================================
void writePWM(int pin, int channel, int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin, duty);
#else
    ledcWrite(channel, duty);
#endif
}

void initMotorPWM() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(PIN_INM11, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_INM12, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_INM21, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PIN_INM22, PWM_FREQ, PWM_RESOLUTION);
#else
    ledcSetup(CH_M11, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_INM11, CH_M11);
    ledcSetup(CH_M12, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_INM12, CH_M12);
    ledcSetup(CH_M21, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_INM21, CH_M21);
    ledcSetup(CH_M22, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_INM22, CH_M22);
#endif
}

// speed: -255 (mundur) hingga +255 (maju)
void setLeftMotor(int speed) {
    if (INVERT_LEFT_MOTOR) speed = -speed;
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        writePWM(PIN_INM11, CH_M11, speed);
        writePWM(PIN_INM12, CH_M12, 0);
    } else if (speed < 0) {
        writePWM(PIN_INM11, CH_M11, 0);
        writePWM(PIN_INM12, CH_M12, -speed);
    } else {
        writePWM(PIN_INM11, CH_M11, 0);
        writePWM(PIN_INM12, CH_M12, 0);
    }
}

void setRightMotor(int speed) {
    if (INVERT_RIGHT_MOTOR) speed = -speed;
    speed = constrain(speed, -255, 255);
    if (speed > 0) {
        writePWM(PIN_INM21, CH_M21, speed);
        writePWM(PIN_INM22, CH_M22, 0);
    } else if (speed < 0) {
        writePWM(PIN_INM21, CH_M21, 0);
        writePWM(PIN_INM22, CH_M22, -speed);
    } else {
        writePWM(PIN_INM21, CH_M21, 0);
        writePWM(PIN_INM22, CH_M22, 0);
    }
}

void setMotors(int left, int right) {
    setLeftMotor(left);
    setRightMotor(right);
}

void stopMotors() {
    setMotors(0, 0);
}

// =============================================================================
// SENSOR INITIALIZATION
// =============================================================================
bool initSensor(VL53L1X &sensor, uint8_t address, const char *label) {
    sensor.setBus(&I2CBus0);
    sensor.setTimeout(500);
    if (!sensor.init()) {
        Serial.printf("[WARN] Sensor '%s' tidak terdeteksi, dilewati.\n", label);
        return false;
    }
    sensor.setAddress(address);
    sensor.setDistanceMode(VL53L1X::Short);
    sensor.setMeasurementTimingBudget(20000); // 20 ms minimum

    // ROI: Baca 50% atas (fisik). Lensa membalik, jadi SPAD bawah = fisik atas.
    // Ukuran 16x8, center SPAD 60 (baris 0-7 dari bawah grid)
    sensor.setROISize(16, 8);
    sensor.setROICenter(60);

    sensor.startContinuous(20);
    Serial.printf("[OK]  Sensor '%s' siap di alamat 0x%02X\n", label, address);
    return true;
}

// =============================================================================
// SENSOR READING
// =============================================================================
const char* sensorLabel[SENSOR_COUNT] = {"KIRI ", "TNGAH", "KANAN"};

void readSensors() {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        if (!sensorConnected[i]) {
            dist[i] = 0;
            continue;
        }
        uint16_t raw = sensors[i].read(false); // non-blocking
        if (sensors[i].timeoutOccurred() || raw < ENEMY_MIN_DIST_MM || raw > ENEMY_MAX_DIST_MM) {
            dist[i] = 0; // Tidak valid / di luar range 1-400mm
        } else {
            dist[i] = raw;
        }
    }
}

// =============================================================================
// AUTONOMOUS DECISION ENGINE
// =============================================================================

// Enum untuk state robot
typedef enum {
    STATE_IDLE,          // Tidak ada musuh (1-400mm) - DIAM / tidak bergerak
    STATE_ATTACK_FULL,   // Musuh di depan - maju lurus
    STATE_ATTACK_LEFT,   // Musuh kiri+tengah - maju + belok kiri
    STATE_ATTACK_RIGHT,  // Musuh tengah+kanan - maju + belok kanan
    STATE_TURN_LEFT,     // Hanya sensor kiri - belok kiri di tempat
    STATE_TURN_RIGHT,    // Hanya sensor kanan - belok kanan di tempat
} RobotState;

RobotState lastState = STATE_IDLE;

RobotState decideState() {
    bool L = (dist[SENSOR_LEFT]   >= ENEMY_MIN_DIST_MM && dist[SENSOR_LEFT]   <= ENEMY_MAX_DIST_MM);
    bool C = (dist[SENSOR_CENTER] >= ENEMY_MIN_DIST_MM && dist[SENSOR_CENTER] <= ENEMY_MAX_DIST_MM);
    bool R = (dist[SENSOR_RIGHT]  >= ENEMY_MIN_DIST_MM && dist[SENSOR_RIGHT]  <= ENEMY_MAX_DIST_MM);

    // Prioritas: jika terdeteksi musuh
    if (C && L && R)   return STATE_ATTACK_FULL;   // Semua terdeteksi -> maju lurus
    if (C && !L && !R) return STATE_ATTACK_FULL;  // Hanya tengah -> maju lurus
    if (C && L && !R)  return STATE_ATTACK_LEFT;   // Tengah + kiri -> belok kiri sedikit
    if (C && !L && R)  return STATE_ATTACK_RIGHT;  // Tengah + kanan -> belok kanan sedikit
    if (L && !C && !R) return STATE_TURN_LEFT;    // Hanya kiri -> putar kiri di tempat
    if (R && !C && !L) return STATE_TURN_RIGHT;   // Hanya kanan -> putar kanan di tempat
    if (L && R && !C)  return STATE_ATTACK_FULL;  // Kiri & kanan -> maju lurus

    // Default: Tidak ada musuh terdeteksi di range 1-400mm -> robot DIAM
    return STATE_IDLE;
}

void executeState(RobotState state) {
    switch (state) {
        case STATE_ATTACK_FULL:
            // Maju lurus penuh
            setMotors(SPEED_ATTACK_FULL, SPEED_ATTACK_FULL);
            break;

        case STATE_ATTACK_LEFT:
            // Maju + belok kiri: roda kanan lebih kencang
            setMotors(SPEED_ATTACK_INNER, SPEED_ATTACK_OUTER);
            break;

        case STATE_ATTACK_RIGHT:
            // Maju + belok kanan: roda kiri lebih kencang
            setMotors(SPEED_ATTACK_OUTER, SPEED_ATTACK_INNER);
            break;

        case STATE_TURN_LEFT:
            // Belok kiri di tempat: kiri mundur, kanan maju
            setMotors(-SPEED_TURN_IN_PLACE, SPEED_TURN_IN_PLACE);
            break;

        case STATE_TURN_RIGHT:
            // Belok kanan di tempat: kiri maju, kanan mundur
            setMotors(SPEED_TURN_IN_PLACE, -SPEED_TURN_IN_PLACE);
            break;

        case STATE_IDLE:
        default:    
            // Tidak ada musuh: bot tidak bergerak sama sekali
            stopMotors();
            break;
    }
}

const char* stateName(RobotState s) {
    switch (s) {
        case STATE_ATTACK_FULL:  return "ATTACK LURUS ";
        case STATE_ATTACK_LEFT:  return "ATTACK KIRI  ";
        case STATE_ATTACK_RIGHT: return "ATTACK KANAN ";
        case STATE_TURN_LEFT:    return "PUTAR KIRI   ";
        case STATE_TURN_RIGHT:   return "PUTAR KANAN  ";
        case STATE_IDLE:         return "IDLE (DIAM)  ";
        default:                 return "UNKNOWN      ";
    }
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    Serial.begin(921600);
    while (!Serial) delay(10);

    Serial.println("============================================");
    Serial.println("   SUMOBOT AUTONOMOUS - VL53L1X Edition   ");
    Serial.println("============================================");

    // 1. Init Motor PWM
    initMotorPWM();
    stopMotors();
    Serial.println("[OK] Motor PWM diinisialisasi.");

    // 2. Init I2C
    I2CBus0.begin(I2C0_SDA, I2C0_SCL);
    I2CBus0.setClock(400000);
    Serial.println("[OK] I2C Bus 0 siap (SDA=21, SCL=22).");

    // 3. Tahan semua sensor dalam reset
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        pinMode(xshutPins[i], OUTPUT);
        digitalWrite(xshutPins[i], LOW);
    }
    delay(10);

    // 4. Init sensor satu per satu
    Serial.println("Inisialisasi sensor VL53L1X...");
    uint8_t connected = 0;
    const char* labels[SENSOR_COUNT] = {"Kiri (Pos0)", "Tengah (Pos1)", "Kanan (Pos2)"};
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
        pinMode(xshutPins[i], INPUT); // Lepaskan XSHUT -> sensor menyala
        delay(10);
        sensorConnected[i] = initSensor(sensors[i], sensorAddresses[i], labels[i]);
        if (sensorConnected[i]) connected++;
    }

    Serial.printf("\n%u dari %u sensor aktif.\n", connected, SENSOR_COUNT);
    if (connected == 0) {
        Serial.println("[ERROR] Tidak ada sensor yang terdeteksi! Periksa wiring.");
        while (1) delay(1000);
    }

    Serial.println("============================================");
    Serial.printf("%-14s | %-7s | %-7s | %-7s | %-14s\n",
                  "State", "KIRI", "TENGAH", "KANAN", "Motor L/R");
    Serial.println("------------------------------------------------------------");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    static unsigned long lastRead = 0;
    static unsigned long lastPrint = 0;
    unsigned long now = millis();

    // Baca sensor sesuai interval
    if (now - lastRead >= SENSOR_READ_INTERVAL_MS) {
        lastRead = now;

        readSensors();

        RobotState state = decideState();
        executeState(state);

        // Debug serial setiap 100ms agar tidak banjir tapi tetap informatif
        if (now - lastPrint >= 100) {
            lastPrint = now;

            // Format jarak: tampilkan angka atau "-" jika tidak terdeteksi
            char dL[8], dC[8], dR[8];
            dist[SENSOR_LEFT]   ? snprintf(dL, sizeof(dL), "%4u mm", dist[SENSOR_LEFT])   : snprintf(dL, sizeof(dL), "  -    ");
            dist[SENSOR_CENTER] ? snprintf(dC, sizeof(dC), "%4u mm", dist[SENSOR_CENTER]) : snprintf(dC, sizeof(dC), "  -    ");
            dist[SENSOR_RIGHT]  ? snprintf(dR, sizeof(dR), "%4u mm", dist[SENSOR_RIGHT])  : snprintf(dR, sizeof(dR), "  -    ");

            // Hitung output motor dari state saat ini untuk display
            int showL = 0, showR = 0;
            switch (state) {
                case STATE_ATTACK_FULL:  showL =  SPEED_ATTACK_FULL;  showR =  SPEED_ATTACK_FULL;  break;
                case STATE_ATTACK_LEFT:  showL =  SPEED_ATTACK_INNER; showR =  SPEED_ATTACK_OUTER; break;
                case STATE_ATTACK_RIGHT: showL =  SPEED_ATTACK_OUTER; showR =  SPEED_ATTACK_INNER; break;
                case STATE_TURN_LEFT:    showL = -SPEED_TURN_IN_PLACE; showR =  SPEED_TURN_IN_PLACE; break;
                case STATE_TURN_RIGHT:   showL =  SPEED_TURN_IN_PLACE; showR = -SPEED_TURN_IN_PLACE; break;
                case STATE_IDLE:
                default:                 showL = 0;                    showR = 0;                    break;
            }

            Serial.printf("%-14s | %s | %s | %s | L=%4d R=%4d\n",
                          stateName(state), dL, dC, dR, showL, showR);

            lastState = state;
        }
    }
}