// =============================================================================
// HEADING ALIGN — hold heading at 0 deg, two-stage turn
// =============================================================================
//
// Packet reading is heading_hold.ino's, unchanged. What's new is the turn:
//
//   COARSE (|err| >= 7.5)  PD on encoder ticks. Gets close, fast.
//   FINE   (|err| <  7.5)  Fixed duty, spin to tick target, SHORT-BRAKE.
//                          This is what lands 1-5 deg accurately.
//
// Camera gives the error. Encoders execute. Camera re-checks. Repeat.
// Both stages are closed on encoders — the camera is only ever consulted
// between turns, never during one.
//
// The old version used PD for everything, with an APPROACH_BAND hack to
// suppress the duty floor on small errors. That's gone: small errors now go
// to a controller that was actually built for them.
//
// Move the robot by hand; it corrects itself back to 0.
//
// =============================================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/pulse_cnt.h"

// ── Identity ───────────────────────────────────────────────────────────────
// Must match the ArUco marker AND be in the ID list typed into initiation.py.
#define THIS_ROBOT_ID   1

// ── Network (must match udp.py) ────────────────────────────────────────────
#define WIFI_SSID       "RobotWifi"
#define WIFI_PASSWORD   "12345678"
#define UDP_PORT        5005

// ── Pin Definitions ────────────────────────────────────────────────────────
#define LEFT_ENC_A    1
#define LEFT_ENC_B    2
#define RIGHT_ENC_A   21
#define RIGHT_ENC_B   47

#define PWMA_PIN   4
#define AIN1_PIN   6
#define AIN2_PIN   5
#define STBY_PIN   7
#define BIN1_PIN   8
#define BIN2_PIN   9
#define PWMB_PIN   10

#define BATTERY_PIN 18

// ── PWM Config ─────────────────────────────────────────────────────────────
#define PWM_FREQ        20000
#define PWM_RESOLUTION  8
#define DUTY_ABS_MAX    255

// ═══════════════════════════════════════════════════════════════════════════
//  TUNABLES
// ═══════════════════════════════════════════════════════════════════════════

#define TARGET_HEADING_DEG  0.0f

// ── THE HANDOFF ────────────────────────────────────────────────────────────
// At or above this, PD. Below, fixed-duty + brake.
float HANDOFF_DEG = 7.5f;

// ── COARSE (PD) ────────────────────────────────────────────────────────────
float KP = 2.0f;
float KD = 0.25f;
float MIN_TURN_DUTY = 80.0f;
float MAX_TURN_DUTY = 120.0f;

// Loose on purpose. PD only has to get close — fine cleans up. Tightening
// this just makes PD hunt at its duty floor.
float COARSE_DEADBAND_DEG = 2.0f;
float STOPPED_RATE_DPS    = 5.0f;
#define COARSE_SETTLE_MS     150
#define COARSE_TIMEOUT_MS    6000

// ── FINE (fixed duty + brake) ──────────────────────────────────────────────
// Small-angle accuracy knob. Tune this on the bench sketch.
float FINE_DUTY = 80.0f;
float FINE_MIN_PULSE_DEG = 0.4f;
unsigned long BRAKE_MS       = 150;
unsigned long FINE_SETTLE_MS = 250;
#define MAX_FINE_PULSES   8
#define FINE_TIMEOUT_MS   3000

// ── Tolerances ─────────────────────────────────────────────────────────────
// Camera says we're done inside this.
float DEADBAND_DEG = 1.0f;

// Encoder-side tolerance for the fine cleanup within one turn.
float FINE_TOLERANCE_DEG = 1.0f;

// Don't act on camera errors smaller than this.
float MIN_CORRECTION_DEG = 1.0f;

// ── Geometry ───────────────────────────────────────────────────────────────
#define TICKS_PER_REV      1400.0f
#define WHEEL_DIAMETER_M   0.065f
#define TRACK_WIDTH_M      0.125f
float TICKS_PER_DEG = (TICKS_PER_REV * TRACK_WIDTH_M)
                      / (360.0f * WHEEL_DIAMETER_M);

// ── Freshness / safety ─────────────────────────────────────────────────────
#define PACKET_TIMEOUT_MS    500
#define FRESH_PACKETS_NEEDED    3
#define FRESH_PACKET_TIMEOUT_MS 500
#define VISION_PIPELINE_MS      120

// Pause after each correction so you can watch it land. 0 for continuous.
unsigned long OBSERVE_PAUSE_MS = 2000;

// ── Direction ──────────────────────────────────────────────────────────────
// Camera theta is CCW-positive. Positive error = turn CCW.
// If it turns the wrong way, flip this.
bool INVERT_SPIN = false;

// ── Timing ─────────────────────────────────────────────────────────────────
#define CONTROL_INTERVAL_MS  10
#define FINE_SAMPLE_MS        2
#define PRINT_INTERVAL_MS   500

// ── Battery compensation ───────────────────────────────────────────────────
#define R1              100000.0f
#define R2               47000.0f
#define DIVIDER_RATIO   (R2 / (R1 + R2))
#define ADC_RESOLUTION  4095.0f
#define ADC_VREF            3.3f
#define BATT_NOMINAL_V      7.2f
bool USE_BATT_COMP = false;

// ── Encoder / PCNT ─────────────────────────────────────────────────────────
#define PCNT_HIGH_LIMIT   32767
#define PCNT_LOW_LIMIT   -32768
#define GLITCH_FILTER_NS  12500

// ═══════════════════════════════════════════════════════════════════════════

static pcnt_unit_handle_t leftUnit = NULL, rightUnit = NULL;
static WiFiUDP udp;
static uint8_t rxBuf[512];

// ── Link / vision state ────────────────────────────────────────────────────
static bool          serverRun      = false;
static bool          haveHeading    = false;
static float         currentHeading = 0.0f;    // radians, CCW+
static uint32_t      lastPacketNum  = 0;
static unsigned long lastPacketMs   = 0;
static unsigned long lastHeadingMs  = 0;

// =============================================================================
// ENCODER SETUP
// =============================================================================

static pcnt_unit_handle_t setupEncoder(int pinA, int pinB, bool invertDirection) {
    pcnt_unit_config_t unitCfg = {
        .low_limit = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
        .intr_priority = 0,
        .flags = {.accum_count = false}
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unitCfg, &unit));

    pcnt_glitch_filter_config_t filterCfg = {.max_glitch_ns = GLITCH_FILTER_NS};
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filterCfg));

    pcnt_channel_edge_action_t onRise = invertDirection
        ? PCNT_CHANNEL_EDGE_ACTION_DECREASE : PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    pcnt_channel_edge_action_t onFall = invertDirection
        ? PCNT_CHANNEL_EDGE_ACTION_INCREASE : PCNT_CHANNEL_EDGE_ACTION_DECREASE;

    pcnt_chan_config_t ch0Cfg = {.edge_gpio_num = pinA, .level_gpio_num = pinB, .flags = {}};
    pcnt_channel_handle_t ch0 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &ch0Cfg, &ch0));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch0, onRise, onFall));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch0,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    pcnt_chan_config_t ch1Cfg = {.edge_gpio_num = pinB, .level_gpio_num = pinA, .flags = {}};
    pcnt_channel_handle_t ch1 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &ch1Cfg, &ch1));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch1, onFall, onRise));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch1,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));
    return unit;
}

static int16_t readTicks(pcnt_unit_handle_t unit) {
    int raw = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(unit, &raw));
    return (int16_t)raw;
}

static void clearEncoders() {
    ESP_ERROR_CHECK(pcnt_unit_clear_count(leftUnit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(rightUnit));
}

// =============================================================================
// MOTOR CONTROL — drive, brake, coast
// =============================================================================

static void enableDriver() { digitalWrite(STBY_PIN, HIGH); }

static void setLeftMotor(float duty) {
    duty = constrain(duty, -(float)DUTY_ABS_MAX, (float)DUTY_ABS_MAX);
    if (duty >= 0) { digitalWrite(AIN1_PIN, HIGH); digitalWrite(AIN2_PIN, LOW); }
    else           { digitalWrite(AIN1_PIN, LOW);  digitalWrite(AIN2_PIN, HIGH); }
    ledcWrite(PWMA_PIN, (int)fabsf(duty));
}

static void setRightMotor(float duty) {
    duty = constrain(duty, -(float)DUTY_ABS_MAX, (float)DUTY_ABS_MAX);
    if (duty >= 0) { digitalWrite(BIN1_PIN, HIGH); digitalWrite(BIN2_PIN, LOW); }
    else           { digitalWrite(BIN1_PIN, LOW);  digitalWrite(BIN2_PIN, HIGH); }
    ledcWrite(PWMB_PIN, (int)fabsf(duty));
}

// SHORT BRAKE: both IN pins HIGH shorts the windings -> stops dead.
// STBY stays HIGH — pulling it low is a coast, not a brake.
// This is why the fine stage is repeatable: coast distance varies with
// friction, which is the thing that ruins small turns.
static void brakeMotors() {
    digitalWrite(STBY_PIN, HIGH);
    digitalWrite(AIN1_PIN, HIGH); digitalWrite(AIN2_PIN, HIGH);
    digitalWrite(BIN1_PIN, HIGH); digitalWrite(BIN2_PIN, HIGH);
    ledcWrite(PWMA_PIN, DUTY_ABS_MAX);
    ledcWrite(PWMB_PIN, DUTY_ABS_MAX);
}

static void allStop() {
    setLeftMotor(0);
    setRightMotor(0);
    digitalWrite(STBY_PIN, LOW);
}

// Positive duty = CCW (subject to INVERT_SPIN).
static void spinInPlace(float duty) {
    if (INVERT_SPIN) duty = -duty;
    setLeftMotor(-duty);
    setRightMotor(duty);
}

// =============================================================================
// BATTERY
// =============================================================================

static float readBatteryVoltage() {
    long sum = 0;
    const int N = 8;
    for (int i = 0; i < N; i++) sum += analogRead(BATTERY_PIN);
    float pinVolts = ((sum / (float)N) / ADC_RESOLUTION) * ADC_VREF;
    return pinVolts / DIVIDER_RATIO;
}

static float voltageCompensation() {
    if (!USE_BATT_COMP) return 1.0f;
    float battV = readBatteryVoltage();
    if (battV < 1.0f) return 1.0f;
    return constrain(BATT_NOMINAL_V / battV, 0.5f, 2.0f);
}

// =============================================================================
// ANGLE / ROTATION HELPERS
// =============================================================================

// Wrap to [-180, 180]. Without this an error of 350 would send the robot the
// long way round instead of turning -10.
static float wrapDeg(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

// Rotation from the wheel DIFFERENCE — cancels residual forward drift.
// Sign: right forward + left reverse = CCW = positive.
static float rotationTicks(int16_t l, int16_t r) {
    return ((float)r - (float)l) / 2.0f;
}

static float rotationDegrees(int16_t l, int16_t r) {
    return rotationTicks(l, r) / TICKS_PER_DEG;
}

static float measuredDegrees() {
    return rotationDegrees(readTicks(leftUnit), readTicks(rightUnit));
}

// =============================================================================
// PACKET PARSING  —  "!cIB" + "!Bfff", big-endian, unpadded
// =============================================================================
// Unchanged from heading_hold.ino.

static uint32_t be32(const uint8_t* p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

// Wire floats are IEEE754 big-endian; the ESP32 is little-endian, so bytes
// must be reassembled before reinterpreting as float.
static float beFloat(const uint8_t* p) {
    uint32_t bits = be32(p);
    float f;
    memcpy(&f, &bits, 4);
    return f;
}

#define HDR_SIZE          6      // struct.calcsize("!cIB")
#define ROBOT_ENTRY_SIZE  13     // struct.calcsize("!Bfff")

// Drain ALL waiting packets, keeping only the newest. UDP queues, so reading
// one per loop would fall progressively further behind.
static void pollPackets() {
    while (udp.parsePacket() > 0) {
        int len = udp.read(rxBuf, sizeof(rxBuf));
        if (len < HDR_SIZE) continue;

        uint8_t  cmd     = rxBuf[0];
        uint32_t pktNum  = be32(&rxBuf[1]);
        uint8_t  nRobots = rxBuf[5];

        if (len < HDR_SIZE + (int)nRobots * ROBOT_ENTRY_SIZE) continue;

        serverRun     = (cmd == 'R');
        lastPacketNum = pktNum;
        lastPacketMs  = millis();

        for (uint8_t i = 0; i < nRobots; i++) {
            const uint8_t* e = &rxBuf[HDR_SIZE + i * ROBOT_ENTRY_SIZE];
            if (e[0] != THIS_ROBOT_ID) continue;
            currentHeading = beFloat(&e[9]);    // theta: id(1) + x(4) + y(4)
            haveHeading    = true;
            lastHeadingMs  = millis();
            break;
        }
    }
}

static bool linkAlive() {
    return (millis() - lastPacketMs) < PACKET_TIMEOUT_MS;
}

// Wait until we've seen FRESH_PACKETS_NEEDED packets newer than sinceNum.
// Returns false on STOP, link loss, or timeout.
static bool waitForFreshPackets(uint32_t sinceNum, unsigned long timeoutMs) {
    unsigned long t0 = millis();
    while (millis() - t0 < timeoutMs) {
        pollPackets();
        if (!serverRun || !linkAlive()) return false;
        if (lastPacketNum >= sinceNum + FRESH_PACKETS_NEEDED) return true;
        delay(2);
    }
    return false;
}

// Abort a move on STOP or link loss. tracker.py sends 'S' on collision and on
// lost-robot, so this is checked continuously, not just between turns.
static bool mustAbort() {
    pollPackets();
    bool dead = (!serverRun || !linkAlive());
    if (dead) {
        Serial.printf("  [abort] run=%d age=%lu wifi=%s rssi=%d\n",
                      serverRun ? 1 : 0,
                      millis() - lastPacketMs,
                      WiFi.status() == WL_CONNECTED ? "up" : "DOWN",
                      WiFi.RSSI());
    }
    return dead;
}

// =============================================================================
// COARSE STAGE — PD on encoder ticks
// =============================================================================
//
// Encoders are NOT cleared here — the caller owns the count so coarse and fine
// legs accumulate into one total. Returns absolute degrees turned since clear.

static float runCoarse(float targetDeg) {
    enableDriver();

    unsigned long startTime   = millis();
    unsigned long lastLoop    = startTime;
    unsigned long inBandSince = 0;

    float lastDeg = measuredDegrees();
    float rateDps = 0.0f;

    while (true) {
        unsigned long now = millis();

        if (mustAbort()) {
            allStop();
            Serial.printf("       [coarse] EXIT=abort after %lu ms\n", now - startTime);
            break;
        }

        if (now - startTime > COARSE_TIMEOUT_MS) {
            allStop();
            Serial.printf("       [coarse] EXIT=timeout after %lu ms\n", now - startTime);
            break;
        }

        if (now - lastLoop < CONTROL_INTERVAL_MS) continue;
        float dtSec = (now - lastLoop) / 1000.0f;
        lastLoop = now;
        if (dtSec <= 0.0f) continue;

        float turnedDeg = measuredDegrees();

        // Filtered turn rate — the D signal. From encoders, so zero camera lag:
        // we decelerate before the camera could even tell us to.
        float rawRate = (turnedDeg - lastDeg) / dtSec;
        rateDps = 0.7f * rateDps + 0.3f * rawRate;
        lastDeg = turnedDeg;

        float error  = targetDeg - turnedDeg;
        float output = KP * error - KD * rateDps;

        // Arrival: in band AND stopped, held briefly. Position alone would
        // "succeed" while coasting through target at speed.
        bool inBand = (fabsf(error) <= COARSE_DEADBAND_DEG)
                      && (fabsf(rateDps) <= STOPPED_RATE_DPS);
        if (inBand) {
            if (inBandSince == 0) inBandSince = now;
            if (now - inBandSince >= COARSE_SETTLE_MS) {
                Serial.printf("       [coarse] EXIT=deadband after %lu ms\n",
                              now - startTime);
                break;
            }
            spinInPlace(0);
            continue;
        } else {
            inBandSince = 0;
        }

        float mag = fabsf(output) * voltageCompensation();
        float dir = (output >= 0) ? 1.0f : -1.0f;

        // Floor applies unconditionally: this stage only runs for errors
        // >= HANDOFF_DEG now, so it's never fighting a small correction.
        // That's what the handoff bought us — no approach-band hack needed.
        if (mag < MIN_TURN_DUTY) mag = MIN_TURN_DUTY;
        if (mag > MAX_TURN_DUTY) mag = MAX_TURN_DUTY;

        spinInPlace(dir * mag);
    }

    allStop();
    delay(FINE_SETTLE_MS);          // let it stop rocking before measuring
    return measuredDegrees();
}

// =============================================================================
// FINE STAGE — fixed duty, spin to tick target, SHORT-BRAKE
// =============================================================================
//
// No PD, no duty floor, no settle loop. One measured shove, stopped dead.
//
// Why this beats PD small: PD's smallest possible command is MIN_TURN_DUTY,
// which for a 2 deg error is a shove that overshoots, and then it hunts.
//
// deltaDeg is RELATIVE. Encoders are not cleared — the target is computed from
// wherever we currently are, so error from earlier legs is never discarded.
// Returns new absolute measured degrees.

static float runFinePulse(float deltaDeg) {
    float startDeg  = measuredDegrees();
    float targetDeg = startDeg + deltaDeg;
    float sign      = (deltaDeg >= 0) ? 1.0f : -1.0f;

    enableDriver();
    unsigned long t0 = millis();

    while (millis() - t0 < FINE_TIMEOUT_MS) {
        float now = measuredDegrees();

        // Directional cross-check — stop when we reach or pass the target in
        // the direction we're travelling.
        if (sign > 0 ? (now >= targetDeg) : (now <= targetDeg)) break;

        if (mustAbort()) { allStop(); return measuredDegrees(); }

        spinInPlace(sign * FINE_DUTY * voltageCompensation());
        delay(FINE_SAMPLE_MS);
    }

    brakeMotors();
    delay(BRAKE_MS);
    allStop();
    delay(FINE_SETTLE_MS);

    return measuredDegrees();
}

// =============================================================================
// THE TURN — coarse if big, then fine cleanup
// =============================================================================
//
// Camera said how far. Encoders do all the work from here; the camera is not
// consulted again until this returns.

static float executeTurn(float targetDeg) {
    clearEncoders();                // one shared count for the whole turn
    float turned;

    // ── Stage 1: COARSE ────────────────────────────────────────────────────
    if (fabsf(targetDeg) >= HANDOFF_DEG) {
        turned = runCoarse(targetDeg);
        Serial.printf("       [coarse] %+.2f  (residual %+.2f)\n",
                      turned, targetDeg - turned);
    } else {
        turned = measuredDegrees();
    }

    // ── Stage 2: FINE cleanup ──────────────────────────────────────────────
    // Each pulse re-reads encoders, so an overshoot is simply answered by a
    // pulse the other way.
    int pulses = 0;
    while (pulses < MAX_FINE_PULSES) {
        if (!serverRun || !linkAlive()) break;

        float residual = targetDeg - turned;
        if (fabsf(residual) <= FINE_TOLERANCE_DEG) break;
        if (fabsf(residual) < FINE_MIN_PULSE_DEG)  break;

        pulses++;
        turned = runFinePulse(residual);
        Serial.printf("       [fine %d] %+.2f  (residual %+.2f)\n",
                      pulses, turned, targetDeg - turned);
    }

    if (pulses >= MAX_FINE_PULSES) {
        Serial.println("       [fine] hit pulse cap — FINE_DUTY likely too high");
    }

    return turned;
}

// =============================================================================
// STATUS
// =============================================================================

static void printStatus() {
    Serial.println("\n---- STATUS ----");
    Serial.printf("  WiFi          : %s  (%s)\n",
                  WiFi.status() == WL_CONNECTED ? "connected" : "DOWN",
                  WiFi.localIP().toString().c_str());
    Serial.printf("  Link          : %s (last pkt %lu ms ago)\n",
                  linkAlive() ? "alive" : "TIMED OUT", millis() - lastPacketMs);
    Serial.printf("  Server cmd    : %s\n", serverRun ? "RUN" : "STOP");
    Serial.printf("  Robot ID      : %d\n", THIS_ROBOT_ID);
    Serial.printf("  Packet #      : %lu\n", (unsigned long)lastPacketNum);
    if (haveHeading) {
        Serial.printf("  Heading       : %.2f deg (seen %lu ms ago)\n",
                      currentHeading * 180.0f / PI, millis() - lastHeadingMs);
    } else {
        Serial.println("  Heading       : NOT SEEN (marker not detected?)");
    }
    Serial.println("---- SETTINGS ----");
    Serial.printf("  handoff %.1f deg\n", HANDOFF_DEG);
    Serial.printf("  coarse: KP=%.3f KD=%.4f  duty %.0f-%.0f  deadband %.1f\n",
                  KP, KD, MIN_TURN_DUTY, MAX_TURN_DUTY, COARSE_DEADBAND_DEG);
    Serial.printf("  fine:   duty %.0f  tol %.2f  brake %lums  cap %d\n",
                  FINE_DUTY, FINE_TOLERANCE_DEG, BRAKE_MS, MAX_FINE_PULSES);
    Serial.printf("  TICKS_PER_DEG = %.4f\n", TICKS_PER_DEG);
    Serial.printf("  camera deadband %.1f  min correction %.1f\n",
                  DEADBAND_DEG, MIN_CORRECTION_DEG);
    Serial.printf("  observe pause %lu ms\n", OBSERVE_PAUSE_MS);
    Serial.printf("  spin: %s   batt comp: %s (%.2f V)\n",
                  INVERT_SPIN ? "INVERTED" : "normal",
                  USE_BATT_COMP ? "on" : "off", readBatteryVoltage());
    Serial.println("------------------\n");
}

// =============================================================================
// WIFI
// =============================================================================

static void connectWiFi() {
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(400);
        Serial.print(".");
        if (millis() - t0 > 20000) {
            Serial.println("\n[WiFi] FAILED — retrying...");
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
            t0 = millis();
        }
    }
    Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());

    udp.begin(UDP_PORT);
    Serial.printf("[UDP] Listening on port %d\n", UDP_PORT);
}

// =============================================================================
// SETUP / LOOP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(800);

    Serial.println("\n=== BUILD " __DATE__ " " __TIME__ " ===");

    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    pinMode(BIN1_PIN, OUTPUT);
    pinMode(BIN2_PIN, OUTPUT);
    pinMode(STBY_PIN, OUTPUT);
    pinMode(BATTERY_PIN, INPUT);

    // ESP32 core v3.x pin-based LEDC API.
    ledcAttach(PWMA_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWMB_PIN, PWM_FREQ, PWM_RESOLUTION);

    allStop();

    leftUnit  = setupEncoder(LEFT_ENC_A,  LEFT_ENC_B,  true);
    rightUnit = setupEncoder(RIGHT_ENC_A, RIGHT_ENC_B, false);

    Serial.println("\n\n=== HEADING ALIGN (two-stage) ===");
    Serial.printf("Robot ID %d, holding %.0f deg +/- %.1f deg\n",
                  THIS_ROBOT_ID, TARGET_HEADING_DEG, DEADBAND_DEG);
    Serial.printf("Handoff at %.1f deg: PD above, fine pulses below\n\n", HANDOFF_DEG);

    connectWiFi();
    printStatus();
}

void loop() {
    pollPackets();

    static unsigned long lastPrint = 0;
    bool doPrint = (millis() - lastPrint > PRINT_INTERVAL_MS);

    // ── Safety gates ───────────────────────────────────────────────────────
    if (!linkAlive()) {
        allStop();
        if (doPrint) {
            lastPrint = millis();
            Serial.printf("[wait] no packets (%lu ms) wifi=%s rssi=%d\n",
                          millis() - lastPacketMs,
                          WiFi.status() == WL_CONNECTED ? "up" : "DOWN",
                          WiFi.RSSI());
        }
        delay(20);
        return;
    }
    if (!serverRun) {
        allStop();
        if (doPrint) {
            lastPrint = millis();
            Serial.println("[stop] server says STOP");
        }
        delay(20);
        return;
    }
    if (!haveHeading) {
        allStop();
        if (doPrint) {
            lastPrint = millis();
            Serial.printf("[wait] link OK (pkt %lu) but marker %d not in packets\n",
                          (unsigned long)lastPacketNum, THIS_ROBOT_ID);
        }
        delay(20);
        return;
    }

    // ── Heading error from the camera ──────────────────────────────────────
    float headingDeg = currentHeading * 180.0f / PI;
    float error = wrapDeg(TARGET_HEADING_DEG - headingDeg);

    // ── Landed ─────────────────────────────────────────────────────────────
    if (fabsf(error) <= DEADBAND_DEG) {
        allStop();
        if (doPrint) {
            lastPrint = millis();
            Serial.printf("[hold] hdg=%7.2f  err=%6.2f\n", headingDeg, error);
        }
        delay(10);
        return;
    }

    // ── Too small to act on ────────────────────────────────────────────────
    if (fabsf(error) < MIN_CORRECTION_DEG) {
        allStop();
        delay(10);
        return;
    }

    // ── Correct ────────────────────────────────────────────────────────────
    uint32_t pktBefore = lastPacketNum;
    Serial.printf("[turn] hdg=%7.2f  err=%6.2f -> %s\n",
                  headingDeg, error,
                  fabsf(error) >= HANDOFF_DEG ? "coarse + fine" : "fine only");

    float actual = executeTurn(error);
    Serial.printf("       encoders turned %.2f (asked %.2f)  ticks L=%d R=%d\n",
                  actual, error, readTicks(leftUnit), readTicks(rightUnit));

    // ── Wait out the camera pipeline ───────────────────────────────────────
    // Every packet in flight was captured before or during the turn. Acting on
    // one would over-correct. Wait for a frame captured AFTER we stopped.
    unsigned long t0 = millis();
    while (millis() - t0 < VISION_PIPELINE_MS) {
        pollPackets();
        if (!serverRun || !linkAlive()) { allStop(); return; }
        delay(2);
    }

    // ── Observation pause ──────────────────────────────────────────────────
    // Keeps polling so a STOP still lands instantly.
    if (OBSERVE_PAUSE_MS > 0) {
        allStop();
        unsigned long tp = millis();
        while (millis() - tp < OBSERVE_PAUSE_MS) {
            pollPackets();
            if (!serverRun || !linkAlive()) { allStop(); return; }
            delay(2);
        }
        if (haveHeading) {
            float restDeg = currentHeading * 180.0f / PI;
            Serial.printf("       camera says %7.2f  (err %6.2f)\n",
                          restDeg, wrapDeg(TARGET_HEADING_DEG - restDeg));
        }
    }

    // ── Require genuinely fresh packets before deciding again ──────────────
    if (!waitForFreshPackets(pktBefore, FRESH_PACKET_TIMEOUT_MS)) {
        allStop();
    }
}
