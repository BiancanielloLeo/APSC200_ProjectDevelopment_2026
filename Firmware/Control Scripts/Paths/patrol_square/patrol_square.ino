// =============================================================================
// PATROL SQUARE — walk a square: (0.75,0) -> (0,0.75) -> (-0.75,0) -> (0,-0.75)
// =============================================================================
//
// Built on heading_align.ino's two-stage turn (coarse PD + fine pulse) and
// pi_tune_test.ino's feedforward + PI velocity loop. Packet reading is
// unchanged from both.
//
// Instead of homing on (0,0), the robot visits four waypoints on the corners
// of a square, in order, looping forever:
//
//     (0.75, 0) -> (0, 0.75) -> (-0.75, 0) -> (0, -0.75) -> back to start
//
// CYCLE (one call to approachWaypoint(), driven from loop()):
//
//   1. Read camera (x, y, theta). If already within POSITION_TOLERANCE_M of
//      the CURRENT waypoint, stop, advance to the next waypoint — done.
//   2. TURN   — point at the current waypoint. Same two-stage turn as
//               heading_align.ino, closed on encoders; camera only supplies
//               the target angle.
//   3. DRIVE  — straight line for the camera-measured distance to the
//               waypoint, dead-reckoned entirely on encoders using per-wheel
//               feedforward + shared PI velocity control (from
//               pi_tune_test.ino). Camera is not consulted during the drive.
//   4. Stop, wait for the vision pipeline + an observe pause, then let a
//      fresh camera packet arrive.
//
// loop() calls this repeatedly. Step 1 on the NEXT call is the verification:
// if the drive undershot/overshot, it just becomes the start of another leg
// toward the same waypoint. Once the robot lands inside tolerance, the
// waypoint index advances and it heads for the next corner.
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

// ── The square (world-frame corners, visited in order, looping) ────────────
struct Waypoint { float x, y; };
static const Waypoint WAYPOINTS[] = {
    { 0.75f,  0.00f },
    { 0.00f,  0.75f },
    {-0.75f,  0.00f },
    { 0.00f, -0.75f },
};
static const int NUM_WAYPOINTS = sizeof(WAYPOINTS) / sizeof(WAYPOINTS[0]);
static int currentWaypoint = 0;   // index of the corner we're currently driving to

// ── Where "there" means ────────────────────────────────────────────────────
// Camera says we've reached the current waypoint inside this radius.
float POSITION_TOLERANCE_M = 0.20f;

// Don't bother driving a leg shorter than this (camera noise floor).
float MIN_DRIVE_M = 0.03f;

// Clamp any single drive leg to this. A bad camera read (misdetected marker,
// jump to a stale position) should never turn into a long blind drive.
float DRIVE_SAFETY_CAP_M = 3.0f;

// ── THE HANDOFF (turn stages, unchanged from heading_align.ino) ────────────
// At or above this, PD. Below, fixed-duty + brake.
float HANDOFF_DEG = 7.5f;

// ── COARSE (PD) ────────────────────────────────────────────────────────────
float KP = 2.0f;
float KD = 0.25f;
float MIN_TURN_DUTY = 80.0f;
float MAX_TURN_DUTY = 120.0f;
float COARSE_DEADBAND_DEG = 2.0f;
float STOPPED_RATE_DPS    = 5.0f;
#define COARSE_SETTLE_MS     150
#define COARSE_TIMEOUT_MS    6000

// ── FINE (fixed duty + brake) ──────────────────────────────────────────────
float FINE_DUTY = 80.0f;
float FINE_MIN_PULSE_DEG = 0.4f;
unsigned long BRAKE_MS       = 150;
unsigned long FINE_SETTLE_MS = 250;
#define MAX_FINE_PULSES   8
#define FINE_TIMEOUT_MS   3000

// Don't act on camera-derived turn errors smaller than this.
float MIN_CORRECTION_DEG = 1.0f;

// ── DRIVE — feedforward + PI velocity loop (from pi_tune_test.ino) ─────────
// Per-wheel feedforward: duty = Kv * |RPM| + Ks. Shared PI on top.
// These are the fitted defaults from the tuning rig; re-run 'ff' on that
// sketch if the floor changes and update these.
float DRIVE_KV = 1.0f;
float DRIVE_KS = 46.0f;
float DRIVE_KP = 2.5f;
float DRIVE_KI = 1.0f;

float CRUISE_RPM   = 70.0f;     // constant cruise speed, hard stop at target
float DRIVE_MAX_DUTY = 178.0f;  // 70% of 255 — same cap as the tuning rig
#define DRIVE_LOOP_HZ   100
#define DRIVE_DT        (1.0f / DRIVE_LOOP_HZ)
#define DRIVE_LOOP_US   (1000000 / DRIVE_LOOP_HZ)
#define RPM_FILTER_ALPHA 0.3f
#define RPM_GLITCH_CEIL  200.0f   // motor no-load spec max; anything above is a bus glitch

// ── Tolerances ─────────────────────────────────────────────────────────────
// Camera says we're pointed correctly inside this (used by executeTurn's caller).
float DEADBAND_DEG = 1.0f;
float FINE_TOLERANCE_DEG = 1.0f;

// ── Geometry ───────────────────────────────────────────────────────────────
#define TICKS_PER_REV      1400.0f
#define WHEEL_DIAMETER_M   0.065f
#define TRACK_WIDTH_M      0.125f
float TICKS_PER_DEG = (TICKS_PER_REV * TRACK_WIDTH_M)
                      / (360.0f * WHEEL_DIAMETER_M);
float TICKS_PER_METER = TICKS_PER_REV / (PI * WHEEL_DIAMETER_M);

// ── Freshness / safety ─────────────────────────────────────────────────────
#define PACKET_TIMEOUT_MS    500
#define FRESH_PACKETS_NEEDED    3
#define FRESH_PACKET_TIMEOUT_MS 500
#define VISION_PIPELINE_MS      120

// Pause after each cycle so you can watch it land. 0 for continuous.
unsigned long OBSERVE_PAUSE_MS = 1000;

// ── Direction ──────────────────────────────────────────────────────────────
// Camera theta is CCW-positive, 0 deg = +X axis. Positive turn error = CCW.
// If turning goes the wrong way, flip this. If the drive goes backwards
// instead of forwards, the wheels are swapped/inverted — check DRIVE_FORWARD_SIGN.
bool INVERT_SPIN = false;
float DRIVE_FORWARD_SIGN = 1.0f;

// ── Timing ─────────────────────────────────────────────────────────────────
#define CONTROL_INTERVAL_MS  10
#define FINE_SAMPLE_MS        2
#define PRINT_INTERVAL_MS   500

// ── Battery compensation (turn stages only — the PI drive loop compensates
//    for battery sag on its own via the integral term) ─────────────────────
#define R1              100000.0f
#define R2               47000.0f
#define DIVIDER_RATIO   (R2 / (R1 + R2))
#define ADC_RESOLUTION  4095.0f
#define ADC_VREF            3.3f
#define BATT_NOMINAL_V      7.2f
bool USE_BATT_COMP = true;

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
static float         currentHeading = 0.0f;    // radians, CCW+, 0 = +X axis
static float         currentX       = 0.0f;    // metres, world frame
static float         currentY       = 0.0f;    // metres, world frame
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

static float wrapDeg(float d) {
    while (d >  180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

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
// Unchanged from heading_hold.ino, extended to also keep x, y.

static uint32_t be32(const uint8_t* p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16)
         | ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

static float beFloat(const uint8_t* p) {
    uint32_t bits = be32(p);
    float f;
    memcpy(&f, &bits, 4);
    return f;
}

#define HDR_SIZE          6      // struct.calcsize("!cIB")
#define ROBOT_ENTRY_SIZE  13     // struct.calcsize("!Bfff")  ->  id, x, y, theta

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
            currentX       = beFloat(&e[1]);    // id(1) + x(4)
            currentY       = beFloat(&e[5]);    // + y(4)
            currentHeading = beFloat(&e[9]);    // + theta(4)
            haveHeading    = true;
            lastHeadingMs  = millis();
            break;
        }
    }
}

static bool linkAlive() {
    return (millis() - lastPacketMs) < PACKET_TIMEOUT_MS;
}

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

static bool mustAbort() {
    pollPackets();
    return (!serverRun || !linkAlive());
}

// =============================================================================
// COARSE STAGE — PD on encoder ticks (unchanged from heading_align.ino)
// =============================================================================

static float runCoarse(float targetDeg) {
    enableDriver();

    unsigned long startTime   = millis();
    unsigned long lastLoop    = startTime;
    unsigned long inBandSince = 0;

    float lastDeg = measuredDegrees();
    float rateDps = 0.0f;

    while (true) {
        unsigned long now = millis();

        if (mustAbort()) { allStop(); break; }

        if (now - startTime > COARSE_TIMEOUT_MS) {
            allStop();
            Serial.println("       [coarse] timeout");
            break;
        }

        if (now - lastLoop < CONTROL_INTERVAL_MS) continue;
        float dtSec = (now - lastLoop) / 1000.0f;
        lastLoop = now;
        if (dtSec <= 0.0f) continue;

        float turnedDeg = measuredDegrees();

        float rawRate = (turnedDeg - lastDeg) / dtSec;
        rateDps = 0.7f * rateDps + 0.3f * rawRate;
        lastDeg = turnedDeg;

        float error  = targetDeg - turnedDeg;
        float output = KP * error - KD * rateDps;

        bool inBand = (fabsf(error) <= COARSE_DEADBAND_DEG)
                      && (fabsf(rateDps) <= STOPPED_RATE_DPS);
        if (inBand) {
            if (inBandSince == 0) inBandSince = now;
            if (now - inBandSince >= COARSE_SETTLE_MS) break;
            spinInPlace(0);
            continue;
        } else {
            inBandSince = 0;
        }

        float mag = fabsf(output) * voltageCompensation();
        float dir = (output >= 0) ? 1.0f : -1.0f;

        if (mag < MIN_TURN_DUTY) mag = MIN_TURN_DUTY;
        if (mag > MAX_TURN_DUTY) mag = MAX_TURN_DUTY;

        spinInPlace(dir * mag);
    }

    allStop();
    delay(FINE_SETTLE_MS);
    return measuredDegrees();
}

// =============================================================================
// FINE STAGE — fixed duty, spin to tick target, SHORT-BRAKE (unchanged)
// =============================================================================

static float runFinePulse(float deltaDeg) {
    float startDeg  = measuredDegrees();
    float targetDeg = startDeg + deltaDeg;
    float sign      = (deltaDeg >= 0) ? 1.0f : -1.0f;

    enableDriver();
    unsigned long t0 = millis();

    while (millis() - t0 < FINE_TIMEOUT_MS) {
        float now = measuredDegrees();

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
// THE TURN — coarse if big, then fine cleanup (unchanged)
// =============================================================================

static float executeTurn(float targetDeg) {
    clearEncoders();
    float turned;

    if (fabsf(targetDeg) >= HANDOFF_DEG) {
        turned = runCoarse(targetDeg);
        Serial.printf("       [coarse] %+.2f  (residual %+.2f)\n",
                      turned, targetDeg - turned);
    } else {
        turned = measuredDegrees();
    }

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
// DRIVE — feedforward + PI velocity loop, straight line, hard stop
// =============================================================================
// Both wheels get the same RPM setpoint. Kv/Ks are per-wheel in principle
// (see pi_tune_test.ino) but start out shared here since that's what was
// handed over; split them if the robot curves during the drive.

struct DriveWheel {
    float Kv, Ks;
    float setpoint;
    float rpm;
    float integral;
    int16_t lastTicks;
};

static DriveWheel driveL, driveR;

// Manual forward declarations. Arduino's auto-prototype generator inserts its
// own prototypes at the very top of the file, before DriveWheel is defined,
// which fails to compile. Declaring these here (after the struct) gives the
// IDE something that already looks like a prototype, so it skips generating
// its own broken one. Same trick used in pi_tune_test.ino for fitAndApply.
static void  driveResetWheel(DriveWheel &w);
static void  driveUpdateRPM(DriveWheel &w, int16_t rawTicks);
static float driveControlLaw(DriveWheel &w);

static void driveResetWheel(DriveWheel &w) {
    w.setpoint = 0.0f;
    w.rpm = 0.0f;
    w.integral = 0.0f;
    w.lastTicks = 0;
}

// One 10 ms sample: update the filtered RPM estimate from a fresh raw tick
// count. Non-physical deltas (bus glitches) are rejected and lastTicks is
// resynced so the next sample starts clean.
static void driveUpdateRPM(DriveWheel &w, int16_t rawTicks) {
    int16_t delta = rawTicks - w.lastTicks;
    w.lastTicks = rawTicks;
    float rawRpm = ((float)delta / TICKS_PER_REV) * (60.0f / DRIVE_DT);

    if (fabsf(rawRpm) > RPM_GLITCH_CEIL) return;
    w.rpm += RPM_FILTER_ALPHA * (rawRpm - w.rpm);
}

// FF + PI control law -> signed duty, clamped to DRIVE_MAX_DUTY with
// conditional-integration anti-windup.
static float driveControlLaw(DriveWheel &w) {
    if (fabsf(w.setpoint) < 0.5f) {
        w.integral = 0.0f;
        return 0.0f;
    }

    float err = w.setpoint - w.rpm;
    float dir = (w.setpoint >= 0) ? 1.0f : -1.0f;
    float ff  = dir * (w.Kv * fabsf(w.setpoint) + w.Ks);

    float newIntegral = w.integral + err * DRIVE_DT;
    if (DRIVE_KI > 0.0001f) {
        float iLimit = DRIVE_MAX_DUTY / DRIVE_KI;
        newIntegral = constrain(newIntegral, -iLimit, iLimit);
    } else {
        newIntegral = 0.0f;
    }

    float unclamped = ff + DRIVE_KP * err + DRIVE_KI * newIntegral;
    bool satHigh = (unclamped >  DRIVE_MAX_DUTY) && (err > 0);
    bool satLow  = (unclamped < -DRIVE_MAX_DUTY) && (err < 0);
    if (!satHigh && !satLow) w.integral = newIntegral;

    float out = ff + DRIVE_KP * err + DRIVE_KI * w.integral;
    return constrain(out, -DRIVE_MAX_DUTY, DRIVE_MAX_DUTY);
}

// Drives straight for distanceM, dead-reckoned on encoders. Cruises at
// CRUISE_RPM, hard-stops (brake) the instant the tick target is reached.
// Returns the measured distance actually covered.
static float driveStraightMeters(float distanceM) {
    if (distanceM > DRIVE_SAFETY_CAP_M) {
        Serial.printf("       [drive] %.2f m requested, capped to %.2f m "
                      "(suspect camera reading)\n", distanceM, DRIVE_SAFETY_CAP_M);
        distanceM = DRIVE_SAFETY_CAP_M;
    }
    if (distanceM < MIN_DRIVE_M) {
        Serial.printf("       [drive] %.3f m too short to bother\n", distanceM);
        return 0.0f;
    }

    clearEncoders();
    driveResetWheel(driveL);
    driveResetWheel(driveR);
    enableDriver();

    float targetTicks = distanceM * TICKS_PER_METER;
    float sign = DRIVE_FORWARD_SIGN;

    // Generous timeout: assume worst case a fraction of cruise speed, plus
    // margin, so a stall/away-from-target reading still gets caught.
    float minExpectedSpeedMps = (CRUISE_RPM / 60.0f) * (PI * WHEEL_DIAMETER_M) * 0.3f;
    if (minExpectedSpeedMps < 0.02f) minExpectedSpeedMps = 0.02f;
    unsigned long timeoutMs = (unsigned long)((distanceM / minExpectedSpeedMps) * 1000.0f) + 3000UL;

    unsigned long t0 = millis();
    unsigned long lastLoopUs_local = micros();

    Serial.printf("       [drive] target %.3f m (%.0f ticks) @ %.0f RPM, timeout %lu ms\n",
                  distanceM, targetTicks, CRUISE_RPM, timeoutMs);

    while (true) {
        if (millis() - t0 > timeoutMs) {
            Serial.println("       [drive] timeout");
            break;
        }
        if (mustAbort()) { allStop(); return 0.0f; }

        uint32_t nowUs = micros();
        if ((uint32_t)(nowUs - lastLoopUs_local) < DRIVE_LOOP_US) continue;
        lastLoopUs_local += DRIVE_LOOP_US;

        int16_t lRaw = readTicks(leftUnit);
        int16_t rRaw = readTicks(rightUnit);

        float traveledTicks = sign * ((float)(lRaw + rRaw) / 2.0f);
        if (traveledTicks >= targetTicks) break;

        driveL.setpoint = sign * CRUISE_RPM;
        driveR.setpoint = sign * CRUISE_RPM;

        driveUpdateRPM(driveL, lRaw);
        driveUpdateRPM(driveR, rRaw);

        float dutyL = driveControlLaw(driveL);
        float dutyR = driveControlLaw(driveR);

        setLeftMotor(dutyL);
        setRightMotor(dutyR);
    }

    brakeMotors();
    delay(BRAKE_MS);
    allStop();
    delay(FINE_SETTLE_MS);

    int16_t lFinal = readTicks(leftUnit);
    int16_t rFinal = readTicks(rightUnit);
    float actualM = (((float)(lFinal + rFinal) / 2.0f) / TICKS_PER_METER) * sign;
    Serial.printf("       [drive] covered %.3f m (asked %.3f m)\n", actualM, distanceM);
    return actualM;
}

// =============================================================================
// APPROACH CYCLE — drive toward the current waypoint
// =============================================================================
// Called once per loop() iteration once the safety gates pass. Either:
//   - we're already at the current corner -> advance the index, hold, ret true
//   - we're not -> turn at the corner, drive one leg toward it, return false
// (Verification of the drive happens on the NEXT call, once a fresh camera
// packet has arrived — see the pipeline/settle wait in loop().)

static bool approachWaypoint() {
    float x = currentX, y = currentY;
    float tx = WAYPOINTS[currentWaypoint].x;
    float ty = WAYPOINTS[currentWaypoint].y;

    float dx = tx - x, dy = ty - y;
    float distToWp = sqrtf(dx * dx + dy * dy);

    Serial.printf("[pos] (%.3f, %.3f)  -> wp %d (%.2f, %.2f)  dist=%.3f m  hdg=%.2f deg\n",
                  x, y, currentWaypoint, tx, ty, distToWp, currentHeading * 180.0f / PI);

    if (distToWp <= POSITION_TOLERANCE_M) {
        allStop();
        int reached = currentWaypoint;
        currentWaypoint = (currentWaypoint + 1) % NUM_WAYPOINTS;
        Serial.printf("[reached] wp %d (%.2f, %.2f) — next is wp %d (%.2f, %.2f)\n",
                      reached, tx, ty, currentWaypoint,
                      WAYPOINTS[currentWaypoint].x, WAYPOINTS[currentWaypoint].y);
        return true;
    }

    // ── Stage 1: turn to point at the waypoint ──────────────────────────────
    float desiredHeadingDeg = atan2f(dy, dx) * 180.0f / PI;
    float headingNowDeg     = currentHeading * 180.0f / PI;
    float aimErr = wrapDeg(desiredHeadingDeg - headingNowDeg);

    if (fabsf(aimErr) >= MIN_CORRECTION_DEG) {
        Serial.printf("[turn->wp] want %.2f deg, err %.2f\n", desiredHeadingDeg, aimErr);
        executeTurn(aimErr);
    }

    // ── Stage 2: drive straight for the camera-measured distance ───────────
    driveStraightMeters(distToWp);

    allStop();
    return false;
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
    if (haveHeading) {
        Serial.printf("  Pose          : (%.3f, %.3f) m  %.2f deg (seen %lu ms ago)\n",
                      currentX, currentY, currentHeading * 180.0f / PI,
                      millis() - lastHeadingMs);
    } else {
        Serial.println("  Pose          : NOT SEEN (marker not detected?)");
    }
    Serial.println("---- SETTINGS ----");
    Serial.printf("  goal: wp %d/%d (%.2f, %.2f) +/- %.2f m\n",
                  currentWaypoint, NUM_WAYPOINTS,
                  WAYPOINTS[currentWaypoint].x, WAYPOINTS[currentWaypoint].y,
                  POSITION_TOLERANCE_M);
    Serial.printf("  turn: handoff %.1f  coarse KP=%.2f KD=%.3f  fine duty %.0f\n",
                  HANDOFF_DEG, KP, KD, FINE_DUTY);
    Serial.printf("  drive: Kv=%.3f Ks=%.1f Kp=%.2f Ki=%.2f  cruise %.0f RPM  cap %.1f m\n",
                  DRIVE_KV, DRIVE_KS, DRIVE_KP, DRIVE_KI, CRUISE_RPM, DRIVE_SAFETY_CAP_M);
    Serial.printf("  TICKS_PER_DEG = %.4f   TICKS_PER_METER = %.1f\n",
                  TICKS_PER_DEG, TICKS_PER_METER);
    Serial.printf("  spin: %s   drive sign: %+.0f   batt comp: %s (%.2f V)\n",
                  INVERT_SPIN ? "INVERTED" : "normal", DRIVE_FORWARD_SIGN,
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

    driveL.Kv = DRIVE_KV; driveL.Ks = DRIVE_KS;
    driveR.Kv = DRIVE_KV; driveR.Ks = DRIVE_KS;
    driveResetWheel(driveL);
    driveResetWheel(driveR);

    Serial.println("\n\n=== PATROL SQUARE (turn - drive, corner to corner) ===");
    Serial.printf("Robot ID %d, %d waypoints, tolerance +/- %.2f m\n",
                  THIS_ROBOT_ID, NUM_WAYPOINTS, POSITION_TOLERANCE_M);
    for (int i = 0; i < NUM_WAYPOINTS; i++) {
        Serial.printf("   wp %d: (%.2f, %.2f)\n", i, WAYPOINTS[i].x, WAYPOINTS[i].y);
    }

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
            Serial.printf("[wait] no packets (%lu ms) — is initiation.py running?\n",
                          millis() - lastPacketMs);
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

    // ── Already at the current corner? Advance to the next and continue. ────
    float wdx = WAYPOINTS[currentWaypoint].x - currentX;
    float wdy = WAYPOINTS[currentWaypoint].y - currentY;
    float distNow = sqrtf(wdx * wdx + wdy * wdy);
    if (distNow <= POSITION_TOLERANCE_M) {
        allStop();
        int reached = currentWaypoint;
        currentWaypoint = (currentWaypoint + 1) % NUM_WAYPOINTS;
        if (doPrint) {
            lastPrint = millis();
            Serial.printf("[reached] wp %d (%.2f, %.2f) at (%.3f, %.3f) — next wp %d\n",
                          reached, WAYPOINTS[reached].x, WAYPOINTS[reached].y,
                          currentX, currentY, currentWaypoint);
        }
        delay(10);
        return;
    }

    // ── Run one full turn-drive cycle toward the current waypoint ──────────
    uint32_t pktBefore = lastPacketNum;
    approachWaypoint();

    // ── Wait out the camera pipeline before trusting the next reading ──────
    unsigned long t0 = millis();
    while (millis() - t0 < VISION_PIPELINE_MS) {
        pollPackets();
        if (!serverRun || !linkAlive()) { allStop(); return; }
        delay(2);
    }

    // ── Observation pause ────────────────────────────────────────────────
    if (OBSERVE_PAUSE_MS > 0) {
        allStop();
        unsigned long tp = millis();
        while (millis() - tp < OBSERVE_PAUSE_MS) {
            pollPackets();
            if (!serverRun || !linkAlive()) { allStop(); return; }
            delay(2);
        }
        if (haveHeading) {
            float d = sqrtf(currentX * currentX + currentY * currentY);
            Serial.printf("       camera says (%.3f, %.3f)  dist=%.3f m  hdg=%.2f\n",
                          currentX, currentY, d, currentHeading * 180.0f / PI);
        }
    }

    // ── Require genuinely fresh packets before deciding again ──────────────
    if (!waitForFreshPackets(pktBefore, FRESH_PACKET_TIMEOUT_MS)) {
        allStop();
    }
}
