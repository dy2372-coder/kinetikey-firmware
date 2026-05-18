#include "mbed.h"
#include <cmath>
#include <cstring>
using namespace std::chrono;

// ─── Peripherals ─────────────────────────────────────────────────────────────
static I2C        i2c(PB_11, PB_10);
static DigitalOut led1(LED1);
static DigitalOut led2(LED2);
static DigitalIn  user_btn(BUTTON1);
static InterruptIn int1_pin(PD_11, PullDown);

// ─── LSM6DSL register map ────────────────────────────────────────────────────
#define LSM6DSL_ADDR    (0x6A << 1)
#define WHO_AM_I        0x0F
#define CTRL1_XL        0x10
#define CTRL2_G         0x11
#define CTRL3_C         0x12
#define DRDY_PULSE_CFG  0x0B
#define INT1_CTRL       0x0D
#define STATUS_REG      0x1E
#define OUTX_L_G        0x22
#define OUTX_L_XL       0x28

// ─── Sensor scaling ───────────────────────────────────────────────────────────
// CTRL1_XL = 0x48 → 104 Hz, ±4 g → 0.122 mg/LSB
constexpr float ACCEL_SENS    = 0.000122f;
// CTRL2_G  = 0x40 → 104 Hz, ±250 dps → 8.75 mdps/LSB
constexpr float GYRO_SENS_DPS = 0.00875f;

// ─── Tuning constants ─────────────────────────────────────────────────────────
constexpr int   CAPTURE_LEN         = 312;    // ~3 s at 104 Hz
constexpr float MOTION_START_G      = 0.20f;
constexpr float GESTURE_PEAK_G      = 0.22f;
constexpr int   COOLDOWN_MS         = 1200;
constexpr int   GESTURE_TIMEOUT_MS  = 12000;
constexpr int   PIN_LENGTH          = 3;
constexpr int   TEMPLATE_LEN        = 52;   // CAPTURE_LEN / TEMPLATE_STRIDE
constexpr int   TEMPLATE_STRIDE     = 6;    // downsample factor: 312 / 6 = 52 points
constexpr float DTW_REJECT_THRESH   = 25.0f;// reject if best DTW dist > this (print to tune)
// Shake = rapid left-right (Y-axis oscillation)
constexpr float SHAKE_THRESH_G      = 0.25f;  // minimum Y amplitude per direction
constexpr int   SHAKE_MIN_REVERSALS = 3;       // direction changes in first ~0.75 s
constexpr int   SHAKE_CHECK_SAMPLES = 80;      // samples examined for shake early-exit

// ─── Types ───────────────────────────────────────────────────────────────────
struct Sample {
    float ax, ay, az;   // g
    float gx, gy, gz;   // dps
};

// Digit mapping: TRIANGLE=1, SQUARE=2, CIRCLE=3
enum GestureID : uint8_t {
    GESTURE_NONE     = 0,
    GESTURE_TRIANGLE = 1,
    GESTURE_SQUARE   = 2,
    GESTURE_CIRCLE   = 3,
    GESTURE_ERASE    = 4,  // shake left-right → backspace
};

enum State : uint8_t {
    STATE_IDLE,
    STATE_CALIBRATE,
    STATE_RECORD_WAIT,
    STATE_RECORD_COMPLETE,
    STATE_UNLOCK_WAIT,
    STATE_UNLOCK_SUCCESS,
    STATE_UNLOCK_FAIL,
};

static const char* gesture_name(GestureID g) {
    switch (g) {
        case GESTURE_TRIANGLE: return "TRIANGLE";
        case GESTURE_SQUARE:   return "SQUARE";
        case GESTURE_CIRCLE:   return "CIRCLE";
        case GESTURE_ERASE:    return "ERASE";
        default:               return "NONE";
    }
}

static void print_pin(const char* label, const GestureID* pin, int len) {
    printf("%s: ", label);
    for (int i = 0; i < PIN_LENGTH; i++) {
        if (i < len) printf("[%d]", (int)pin[i]);
        else         printf("[_]");
    }
    printf("\r\n");
}

// ─── ISR-shared state (volatile) ─────────────────────────────────────────────
volatile bool  data_ready    = false;
volatile bool  capturing     = false;
volatile bool  capture_done  = false;
volatile int   capture_count = 0;
volatile float live_ax = 0.0f, live_ay = 0.0f, live_az = 0.0f;

static Sample    capture_buf[CAPTURE_LEN];
static GestureID recorded_pin[PIN_LENGTH] = {};
static bool      pin_valid               = false;

// ─── ISR ─────────────────────────────────────────────────────────────────────
static void isr_drdy() { data_ready = true; }

// ─── I2C helpers ─────────────────────────────────────────────────────────────
static bool write_reg(uint8_t reg, uint8_t val) {
    char buf[2] = {(char)reg, (char)val};
    return (i2c.write(LSM6DSL_ADDR, buf, 2) == 0);
}

static bool read_reg(uint8_t reg, uint8_t &val) {
    char r = (char)reg;
    if (i2c.write(LSM6DSL_ADDR, &r, 1, true) != 0) return false;
    char v = 0;
    if (i2c.read(LSM6DSL_ADDR, &v, 1) != 0) return false;
    val = (uint8_t)v;
    return true;
}

static bool read_int16(uint8_t reg_low, int16_t &val) {
    uint8_t lo = 0, hi = 0;
    if (!read_reg(reg_low,     lo)) return false;
    if (!read_reg(reg_low + 1, hi)) return false;
    val = (int16_t)((uint16_t)(hi << 8) | lo);
    return true;
}

// ─── Sensor init ─────────────────────────────────────────────────────────────
static bool init_sensor() {
    uint8_t who = 0;
    if (!read_reg(WHO_AM_I, who) || who != 0x6A) {
        printf("LSM6DSL not found (WHO_AM_I=0x%02X)\r\n", who);
        return false;
    }
    printf("LSM6DSL OK (WHO_AM_I=0x6A)\r\n");

    if (!write_reg(CTRL3_C,        0x44)) printf("[WARN] CTRL3_C\r\n");
    if (!write_reg(CTRL1_XL,       0x48)) printf("[WARN] CTRL1_XL\r\n");
    if (!write_reg(CTRL2_G,        0x40)) printf("[WARN] CTRL2_G\r\n");
    if (!write_reg(INT1_CTRL,      0x03)) printf("[WARN] INT1_CTRL\r\n");
    if (!write_reg(DRDY_PULSE_CFG, 0x80)) printf("[WARN] DRDY_PULSE_CFG\r\n");

    ThisThread::sleep_for(100ms);

    uint8_t dummy;
    read_reg(STATUS_REG, dummy);
    int16_t tmp;
    for (int i = 0; i < 3; i++) read_int16(OUTX_L_XL + i * 2, tmp);
    for (int i = 0; i < 3; i++) read_int16(OUTX_L_G  + i * 2, tmp);

    int1_pin.rise(&isr_drdy);
    return true;
}

// ─── Acquisition thread (HIGH priority) ──────────────────────────────────────
void task_acquire() {
    while (true) {
        if (data_ready) {
            data_ready = false;

            int16_t ra[3], rg[3];
            for (int i = 0; i < 3; i++) {
                read_int16(OUTX_L_XL + i * 2, ra[i]);
                read_int16(OUTX_L_G  + i * 2, rg[i]);
            }

            float ax = ra[0] * ACCEL_SENS, ay = ra[1] * ACCEL_SENS, az = ra[2] * ACCEL_SENS;
            float gx = rg[0] * GYRO_SENS_DPS, gy = rg[1] * GYRO_SENS_DPS, gz = rg[2] * GYRO_SENS_DPS;

            live_ax = ax; live_ay = ay; live_az = az;

            if (capturing) {
                int cnt = capture_count;
                if (cnt < CAPTURE_LEN) {
                    capture_buf[cnt] = {ax, ay, az, gx, gy, gz};
                    capture_count = cnt + 1;
                    if (cnt + 1 >= CAPTURE_LEN) {
                        capturing = false;
                        __DMB();
                        capture_done = true;
                    }
                }
            }
        }
        ThisThread::sleep_for(1ms);
    }
}

// ─── Shake detection ──────────────────────────────────────────────────────────
// True if first len samples have enough rapid Y-axis reversals.
static bool is_shake(const Sample* buf, int len) {
    float sum_y = 0.0f;
    for (int i = 0; i < len; i++) sum_y += buf[i].ay;
    float mean_y = sum_y / len;

    int  reversals = 0;
    bool prev_pos  = false;
    bool have      = false;

    for (int i = 0; i < len; i++) {
        float dy = buf[i].ay - mean_y;
        if (fabsf(dy) < SHAKE_THRESH_G) continue;
        bool pos = (dy > 0.0f);
        if (have && pos != prev_pos) reversals++;
        prev_pos = pos;
        have     = true;
    }
    return reversals >= SHAKE_MIN_REVERSALS;
}

// ─── DTW shape recognition ────────────────────────────────────────────────────
// 52-point [accel_mag, gyro_mag] per shape, normalised per channel.
// Record templates in CALIBRATE mode; classify picks lowest DTW distance.

// [shape_idx][point][channel]  shape: 0=triangle 1=square 2=circle
static float shape_templates[3][TEMPLATE_LEN][2];
static bool  templates_valid = false;

static void extract_features(const Sample* buf, int blen, float dst[][2]) {
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    for (int i = 0; i < blen; i++) {
        sum_ax += buf[i].ax; sum_ay += buf[i].ay; sum_az += buf[i].az;
    }
    float mean_ax = sum_ax / blen, mean_ay = sum_ay / blen, mean_az = sum_az / blen;

    float max_d = 0, max_g = 0;
    for (int t = 0; t < TEMPLATE_LEN; t++) {
        int start = t * TEMPLATE_STRIDE;
        int end   = start + TEMPLATE_STRIDE;
        if (end > blen) end = blen;
        float sd = 0, sg = 0;
        for (int i = start; i < end; i++) {
            float dx = buf[i].ax - mean_ax, dy = buf[i].ay - mean_ay, dz = buf[i].az - mean_az;
            sd += sqrtf(dx*dx + dy*dy + dz*dz);
            sg += sqrtf(buf[i].gx*buf[i].gx + buf[i].gy*buf[i].gy + buf[i].gz*buf[i].gz);
        }
        int n = end - start;
        dst[t][0] = sd / n;
        dst[t][1] = sg / n;
        if (dst[t][0] > max_d) max_d = dst[t][0];
        if (dst[t][1] > max_g) max_g = dst[t][1];
    }
    // normalise per channel — amplitude differences shouldn't affect matching
    if (max_d < 0.01f) max_d = 0.01f;
    if (max_g < 1.0f)  max_g = 1.0f;
    for (int t = 0; t < TEMPLATE_LEN; t++) {
        dst[t][0] /= max_d;
        dst[t][1] /= max_g;
    }
}

static void store_template(int shape_idx) {
    extract_features(capture_buf, CAPTURE_LEN, shape_templates[shape_idx]);
}

// O(N) memory DTW, returns total path cost.
static float dtw_distance(const float query[][2], const float templ[][2]) {
    static float prev[TEMPLATE_LEN + 1];
    static float curr[TEMPLATE_LEN + 1];

    for (int j = 0; j <= TEMPLATE_LEN; j++) prev[j] = 1e9f;
    prev[0] = 0.0f;

    for (int i = 1; i <= TEMPLATE_LEN; i++) {
        curr[0] = 1e9f;
        for (int j = 1; j <= TEMPLATE_LEN; j++) {
            float d0   = query[i-1][0] - templ[j-1][0];
            float d1   = query[i-1][1] - templ[j-1][1];
            float cost = sqrtf(d0*d0 + d1*d1);
            float best = prev[j];
            if (curr[j-1] < best) best = curr[j-1];
            if (prev[j-1] < best) best = prev[j-1];
            curr[j] = cost + best;
        }
        for (int j = 0; j <= TEMPLATE_LEN; j++) prev[j] = curr[j];
    }
    return prev[TEMPLATE_LEN];
}

static GestureID classify(const Sample* buf, int len) {
    // Reject motion-free captures
    float sum_dyn = 0, max_dyn = 0;
    float mean_ax = 0, mean_ay = 0, mean_az = 0;
    for (int i = 0; i < len; i++) {
        mean_ax += buf[i].ax; mean_ay += buf[i].ay; mean_az += buf[i].az;
    }
    mean_ax /= len; mean_ay /= len; mean_az /= len;
    for (int i = 0; i < len; i++) {
        float dx = buf[i].ax - mean_ax, dy = buf[i].ay - mean_ay, dz = buf[i].az - mean_az;
        float d = sqrtf(dx*dx + dy*dy + dz*dz);
        sum_dyn += d;
        if (d > max_dyn) max_dyn = d;
    }
    if (max_dyn < GESTURE_PEAK_G && sum_dyn / len < 0.08f) {
        printf("[DTW] too weak: max=%.2f\r\n", max_dyn);
        return GESTURE_NONE;
    }

    if (!templates_valid) {
        printf("[DTW] not calibrated — hold button 3 s from IDLE\r\n");
        return GESTURE_NONE;
    }

    static float query[TEMPLATE_LEN][2];
    extract_features(buf, len, query);

    float dist[3];
    for (int t = 0; t < 3; t++) dist[t] = dtw_distance(query, shape_templates[t]);

    int best = 0;
    if (dist[1] < dist[best]) best = 1;
    if (dist[2] < dist[best]) best = 2;
    int second = (best == 0) ? 1 : 0;
    if (dist[(best == 2) ? 1 : 2] < dist[second]) second = (best == 2) ? 1 : 2;

    static const char* names[3] = {"TRIANGLE", "SQUARE", "CIRCLE"};
    printf("[DTW] tri=%.1f sq=%.1f circ=%.1f → %s (margin=%.1f)\r\n",
           dist[0], dist[1], dist[2], names[best], dist[second] - dist[best]);

    if (dist[best] > DTW_REJECT_THRESH) {
        printf("[DTW] rejected (dist=%.1f > thr=%.1f)\r\n", dist[best], DTW_REJECT_THRESH);
        return GESTURE_NONE;
    }
    return (GestureID)(best + 1); // GESTURE_TRIANGLE=1 GESTURE_SQUARE=2 GESTURE_CIRCLE=3
}

// ─── Wait for motion, then capture gesture or detect shake ───────────────────
static GestureID wait_and_capture(int timeout_ms) {
    int elapsed = 0;
    while (elapsed < timeout_ms) {
        float ax = live_ax, ay = live_ay, az = live_az;
        float mag = sqrtf(ax*ax + ay*ay + az*az);
        if (fabsf(mag - 1.0f) > MOTION_START_G) break;
        ThisThread::sleep_for(10ms);
        elapsed += 10;
    }
    if (elapsed >= timeout_ms) return GESTURE_NONE;

    capture_count = 0;
    capture_done  = false;
    capturing     = true;

    bool shake_tested = false;
    int  waited       = 0;

    while (!capture_done && waited < 5000) {
        ThisThread::sleep_for(10ms);
        waited += 10;

        // early shake check at ~0.75 s so we don't wait for the full capture
        if (!shake_tested && capture_count >= SHAKE_CHECK_SAMPLES) {
            shake_tested = true;
            if (is_shake(capture_buf, SHAKE_CHECK_SAMPLES)) {
                capturing = false;
                return GESTURE_ERASE;
            }
        }
    }

    capturing = false;
    if (!capture_done) return GESTURE_NONE;

    return classify(capture_buf, CAPTURE_LEN);
}

// ─── LED helpers ─────────────────────────────────────────────────────────────
static void led_flash(DigitalOut &led, int n, int on_ms, int off_ms) {
    for (int i = 0; i < n; i++) {
        led = 1; ThisThread::sleep_for(milliseconds(on_ms));
        led = 0; if (i < n - 1) ThisThread::sleep_for(milliseconds(off_ms));
    }
}

static void led_celebrate() {
    for (int i = 0; i < 6; i++) {
        led1 = 1; led2 = 0; ThisThread::sleep_for(110ms);
        led1 = 0; led2 = 1; ThisThread::sleep_for(110ms);
    }
    led1 = 0; led2 = 0;
}

static void led_fail_pattern() {
    led2 = 0;
    led_flash(led1, 3, 350, 150);
}

// ─── Main ─────────────────────────────────────────────────────────────────────
int main() {
    static BufferedSerial pc(USBTX, USBRX, 115200);
    i2c.frequency(400000);

    printf("\r\n=== KinetiKey v11 — DTW Shape PIN Lock ===\r\n");
    printf("Shapes: TRIANGLE=1  SQUARE=2  CIRCLE=3\r\n");
    printf("Shake left-right = erase last digit\r\n");
    printf("HOLD 3 s = calibrate | HOLD 1.5 s = record PIN | SHORT = unlock\r\n\r\n");

    if (!init_sensor()) {
        while (true) {
            led1 = 1; led2 = 1; ThisThread::sleep_for(150ms);
            led1 = 0; led2 = 0; ThisThread::sleep_for(150ms);
        }
    }

    Thread acq_thread(osPriorityHigh, 4096);
    acq_thread.start(task_acquire);
    ThisThread::sleep_for(200ms);

    if (!templates_valid)
        printf("[!] No calibration — hold button 3 s to calibrate first.\r\n\r\n");

    State state       = STATE_IDLE;
    int   gesture_idx = 0;  // digit index (0-3)
    int   rec_fails   = 0;  // consecutive failed detections in RECORD mode
    int   unl_fails   = 0;  // consecutive failed detections in UNLOCK mode

    while (true) {
        switch (state) {

        // ── IDLE ──────────────────────────────────────────────────────────────
        case STATE_IDLE: {
            led1 = 0; led2 = 0;
            printf("\r\n[IDLE] Waiting for button...\r\n");
            if (!pin_valid) printf("  (No PIN stored — long-press to record)\r\n");

            int  blink_acc = 0, held_ms = 0;
            bool got_press = false;
            while (!got_press) {
                if (user_btn.read() == 0) {
                    led1 = 0; held_ms = 0;
                    while (user_btn.read() == 0) { ThisThread::sleep_for(10ms); held_ms += 10; }
                    got_press = true;
                } else {
                    ThisThread::sleep_for(10ms);
                    blink_acc += 10;
                    if (blink_acc >= 500) { led1 = !led1; blink_acc = 0; }
                }
            }
            led1 = 0;

            if (held_ms >= 3000) {
                state = STATE_CALIBRATE;
            } else if (held_ms >= 1500) {
                if (!templates_valid) {
                    printf("[ERROR] Calibrate first — hold button 3 s.\r\n");
                    led_flash(led1, 3, 200, 100);
                } else {
                    gesture_idx = 0;
                    rec_fails   = 0;
                    memset(recorded_pin, 0, sizeof(recorded_pin));
                    state = STATE_RECORD_WAIT;
                }
            } else {
                if (!pin_valid) {
                    printf("[ERROR] No PIN stored — long-press to record.\r\n");
                    led_flash(led1, 3, 200, 100);
                } else {
                    gesture_idx = 0;
                    unl_fails   = 0;
                    state = STATE_UNLOCK_WAIT;
                }
            }
            break;
        }

        // ── CALIBRATE ─────────────────────────────────────────────────────────
        case STATE_CALIBRATE: {
            static const char* cal_names[3] = {
                "TRIANGLE — 3 strokes, pause at each corner",
                "SQUARE   — 4 strokes, pause at each corner",
                "CIRCLE   — one smooth continuous loop"
            };
            printf("\r\n[CAL] Calibration mode.\r\n");
            printf("  You will record each shape once.\r\n");
            printf("  Press the button when ready, then draw immediately.\r\n\r\n");

            for (int i = 0; i < 4; i++) {
                led1 = 1; led2 = 1; ThisThread::sleep_for(80ms);
                led1 = 0; led2 = 0; ThisThread::sleep_for(80ms);
            }

            for (int s = 0; s < 3; s++) {
                bool recorded = false;
                while (!recorded) {
                    printf("[CAL %d/3] Draw: %s\r\n", s + 1, cal_names[s]);
                    printf("          Press button when ready...\r\n");

                    while (user_btn.read() != 0) {
                        led1 = !led1; led2 = 0;
                        ThisThread::sleep_for(300ms);
                    }
                    while (user_btn.read() == 0) ThisThread::sleep_for(10ms); // wait release
                    led1 = 1; led2 = 0;
                    printf("[CAL] Go — draw now!\r\n");

                    GestureID g = wait_and_capture(GESTURE_TIMEOUT_MS);

                    if (capture_done && g != GESTURE_ERASE) {
                        store_template(s);
                        printf("[CAL] Template %d stored.\r\n\r\n", s + 1);
                        led1 = 0;
                        led_flash(led2, 3, 120, 80);
                        ThisThread::sleep_for(600ms);
                        recorded = true;
                    } else if (g == GESTURE_ERASE) {
                        printf("[CAL] Shake detected — draw the shape, not shake. Retry.\r\n");
                        led_flash(led1, 2, 200, 100);
                    } else {
                        printf("[CAL] No motion / timeout — try again.\r\n");
                        led_flash(led1, 2, 200, 100);
                    }
                }
            }

            templates_valid = true;
            printf("[CAL] Done! All 3 templates stored.\r\n");
            printf("      Hold 1.5 s to record PIN, short press to unlock.\r\n\r\n");
            led_celebrate();
            state = STATE_IDLE;
            break;
        }

        // ── RECORD_WAIT ───────────────────────────────────────────────────────
        case STATE_RECORD_WAIT: {
            printf("\r\n[RECORD] Digit %d/%d — draw shape (or shake to erase)\r\n",
                   gesture_idx + 1, PIN_LENGTH);
            print_pin("  PIN so far", recorded_pin, gesture_idx);
            led1 = 1; led2 = 0;

            GestureID g = wait_and_capture(GESTURE_TIMEOUT_MS);

            if (g == GESTURE_ERASE) {
                if (gesture_idx > 0) {
                    gesture_idx--;
                    recorded_pin[gesture_idx] = GESTURE_NONE;
                    printf("[RECORD] Erased. ");
                    print_pin("PIN", recorded_pin, gesture_idx);
                } else {
                    printf("[RECORD] Nothing to erase.\r\n");
                }
            } else if (g == GESTURE_TRIANGLE || g == GESTURE_SQUARE || g == GESTURE_CIRCLE) {
                rec_fails = 0;
                recorded_pin[gesture_idx] = g;
                gesture_idx++;
                printf("[RECORD] Digit %d: %s\r\n", gesture_idx, gesture_name(g));
                print_pin("  PIN", recorded_pin, gesture_idx);
                led2 = 1; ThisThread::sleep_for(300ms); led2 = 0;
                ThisThread::sleep_for(milliseconds(COOLDOWN_MS));
                if (gesture_idx >= PIN_LENGTH) {
                    state = STATE_RECORD_COMPLETE;
                }

            } else {
                // retry up to 5 times before giving up
                rec_fails++;
                if (rec_fails >= 5) {
                    printf("[RECORD] 5 failed attempts — returning to IDLE.\r\n");
                    led_fail_pattern();
                    state = STATE_IDLE;
                    gesture_idx = 0;
                    rec_fails = 0;
                } else {
                    printf("[RECORD] No shape (%d/5) — try again.\r\n", rec_fails);
                    led_flash(led2, 1, 80, 0);
                }
            }
            break;
        }

        // ── RECORD_COMPLETE ───────────────────────────────────────────────────
        case STATE_RECORD_COMPLETE:
            pin_valid = true;
            printf("\r\n[RECORD COMPLETE] PIN saved: ");
            for (int i = 0; i < PIN_LENGTH; i++) printf("%d", (int)recorded_pin[i]);
            printf("\r\n");
            led_celebrate();
            state = STATE_IDLE;
            gesture_idx = 0;
            break;

        // ── UNLOCK_WAIT ───────────────────────────────────────────────────────
        case STATE_UNLOCK_WAIT: {
            static GestureID attempt[PIN_LENGTH] = {};

            if (gesture_idx == 0) memset(attempt, 0, sizeof(attempt));

            printf("\r\n[UNLOCK] Digit %d/%d — draw shape (or shake to erase)\r\n",
                   gesture_idx + 1, PIN_LENGTH);
            print_pin("  Attempt", attempt, gesture_idx);
            led1 = 0; led2 = 1;

            GestureID g = wait_and_capture(GESTURE_TIMEOUT_MS);

            if (g == GESTURE_ERASE) {
                if (gesture_idx > 0) {
                    gesture_idx--;
                    attempt[gesture_idx] = GESTURE_NONE;
                    printf("[UNLOCK] Erased. ");
                    print_pin("Attempt", attempt, gesture_idx);
                } else {
                    printf("[UNLOCK] Nothing to erase.\r\n");
                }

            } else if (g == GESTURE_TRIANGLE || g == GESTURE_SQUARE || g == GESTURE_CIRCLE) {
                unl_fails = 0;
                attempt[gesture_idx] = g;
                gesture_idx++;
                printf("[UNLOCK] Digit %d: %s\r\n", gesture_idx, gesture_name(g));
                print_pin("  Attempt", attempt, gesture_idx);
                led1 = 1; ThisThread::sleep_for(200ms); led1 = 0;
                ThisThread::sleep_for(milliseconds(COOLDOWN_MS));

                if (gesture_idx >= PIN_LENGTH) {
                    bool match = (memcmp(attempt, recorded_pin, PIN_LENGTH * sizeof(GestureID)) == 0);
                    state = match ? STATE_UNLOCK_SUCCESS : STATE_UNLOCK_FAIL;
                    gesture_idx = 0;
                }

            } else {
                unl_fails++;
                if (unl_fails >= 5) {
                    printf("[UNLOCK] 5 failed attempts — aborting.\r\n");
                    state = STATE_UNLOCK_FAIL;
                    gesture_idx = 0;
                    unl_fails = 0;
                } else {
                    printf("[UNLOCK] No shape (%d/5) — try again.\r\n", unl_fails);
                    led_flash(led1, 1, 80, 0);
                }
            }
            break;
        }

        // ── UNLOCK_SUCCESS ────────────────────────────────────────────────────
        case STATE_UNLOCK_SUCCESS:
            printf("\r\n*** UNLOCKED! PIN correct. ***\r\n");
            led_celebrate();
            led1 = 1; led2 = 1;
            ThisThread::sleep_for(2s);
            led1 = 0; led2 = 0;
            state = STATE_IDLE;
            break;

        // ── UNLOCK_FAIL ───────────────────────────────────────────────────────
        case STATE_UNLOCK_FAIL:
            printf("\r\n[ACCESS DENIED]\r\n");
            led_fail_pattern();
            state = STATE_IDLE;
            gesture_idx = 0;
            break;
        }
    }
}
