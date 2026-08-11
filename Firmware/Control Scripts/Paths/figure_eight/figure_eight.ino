// =============================================================================
// FIGURE EIGHT — pure pursuit path following
// =============================================================================
//
// Built on approach_origin.ino. The drive control stack is UNCHANGED:
// per-wheel feedforward + PI velocity loop, encoder handling, packet parsing
// and safety gates are all exactly as validated there.
//
// What is replaced is the discrete state machine (turn / drive / turn). Instead
// of stopping between movements, the robot now runs one continuous controller:
//
//   Every camera packet (~20 Hz):
//     1. Advance along the path to the nearest point AHEAD of us.
//     2. Look further ahead by LOOKAHEAD_M to pick a goal point.
//     3. Transform that goal into the robot frame.
//     4. Curvature kappa = 2*y_r / Ld^2.
//     5. Split into left/right wheel speeds and hand to the existing PI loop.
//
// At CRUISE_MPS = 0.15 the ~150 ms vision latency is about 2 cm of stale
// position, which is inside the tracking error anyway — so the camera pose is
// used directly with no odometry fusion.
//
// TUNING (in rough order of what to try first):
//   LOOKAHEAD_M   too small -> weaves/oscillates;  too large -> cuts corners
//   CRUISE_MPS    lower it if the robot cannot keep up on the tight crossing
//   LOBE_RADIUS_M size of each circle; keep 2*r inside the camera frame
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

// ── Where "there" means ────────────────────────────────────────────────────
// Camera says we're home inside this radius.
float POSITION_TOLERANCE_M = 0.10f;

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
float DEADBAND_DEG = 1.0f;
float FINE_TOLERANCE_DEG = 1.0f;

// ── PURE PURSUIT / PATH ────────────────────────────────────────────────────

// Radius of each lobe of the figure-eight, metres. The full shape spans
// 4*r HORIZONTALLY and 2*r vertically. At r=0.40 that is 1.6 m wide by
// 0.8 m tall. Keep 4*r inside the frame width with margin for the robot.
float LOBE_RADIUS_M = 0.40f;

// Spacing between generated waypoints. Smaller = smoother goal selection but
// more points. 0.05 m gives ~50 points per lobe at r=0.40.
float PATH_POINT_SPACING_M = 0.05f;

// THE tuning knob. Distance ahead along the path to aim at.
// Too small: robot weaves and can oscillate. Too large: it cuts the corners
// and may skip the crossing entirely. Start 0.15, adjust in 0.02 steps.
float LOOKAHEAD_M = 0.15f;

// Forward travel speed, m/s. 0.15 m/s is about 44 RPM at the wheel.
float CRUISE_MPS = 0.15f;

// How many points ahead to consider when re-locating ourselves on the path.
// Must be large enough to keep up if we fall behind, small enough that we
// cannot jump across the crossing to the other lobe.
#define PATH_SEARCH_WINDOW  12

// Curvature limit, 1/m. Caps how hard it will try to turn; without this a
// large lateral error near the crossing commands a wheel reversal.
float MAX_CURVATURE = 6.0f;

// Used when the goal ends up behind the robot (e.g. after being picked up and
// put down facing the wrong way) — spin toward it instead of arcing.
float TURN_IN_PLACE_RPM = 35.0f;

// If the robot is further than this from its tracked path point, it has lost
// the path (picked up, shoved, or bad camera data). Re-acquire from scratch.
float PATH_LOST_DIST_M = 0.45f;

// Status print interval while following, ms.
#define PURSUIT_PRINT_MS  500

// ── Geometry ───────────────────────────────────────────────────────────────
#define TICKS_PER_REV      1400.0f
#define WHEEL_DIAMETER_M   0.065f
#define TRACK_WIDTH_M      0.125f
float TICKS_PER_DEG = (TICKS_PER_REV * TRACK_WIDTH_M)
                      / (360.0f * WHEEL_DIAMETER_M);
float TICKS_PER_METER = TICKS_PER_REV / (PI * WHEEL_DIAMETER_M);

// ── Freshness / safety ─────────────────────────────────────────────────────
// Raised from 500. The observed gaps during continuous running are 520-960 ms,
// and at 20 Hz a 500 ms window is only 10 packets. In the discrete sketch a
// gap just aborted one move; here it stops the controller mid-curve, so a
// slightly longer tolerance is worth it. Still short enough to be a real
// safety stop if the laptop goes away.
#define PACKET_TIMEOUT_MS    1200
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

// =============================================================================
// DRIVE — feedforward + PI velocity loop, straight line, hard stop
// =============================================================================
// UNCHANGED from approach_origin.ino. Pure pursuit only changes what the
// setpoints ARE — the loop that achieves them is exactly as validated.
// Kv/Ks are shared across wheels; split them if the robot curves on a
// straight section of the path.

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

// =============================================================================
// FIGURE-EIGHT PATH
// =============================================================================
//
// Two tangent circles that cross at the origin, side by side:
//   Right lobe: centre (+r, 0)
//   Left  lobe: centre (-r, 0)
// Spans 4*r horizontally and 2*r vertically — wide rather than tall, to suit
// a camera frame that is wider than it is deep.
//
// Generated once at boot into PATH[] so the geometry can be changed by editing
// LOBE_RADIUS_M / PATH_POINT_SPACING_M without recomputing anything by hand.

#define PATH_MAX_POINTS  256

static float pathX[PATH_MAX_POINTS];
static float pathY[PATH_MAX_POINTS];
static int   pathCount = 0;

// The waypoint we are currently tracking from. Never allowed to move
// backwards — this is what stops the robot jumping lobes at the crossing,
// where two parts of the path are physically on top of each other.
static int   pathIndex = 0;
static int   lapCount  = 0;

static void buildFigureEight() {
    pathCount = 0;

    float circumference = 2.0f * PI * LOBE_RADIUS_M;
    int perLobe = (int)(circumference / PATH_POINT_SPACING_M);
    if (perLobe < 8) perLobe = 8;
    if (perLobe > (PATH_MAX_POINTS / 2) - 1) perLobe = (PATH_MAX_POINTS / 2) - 1;

    // Right lobe: centre (+r, 0).
    for (int i = 0; i < perLobe; i++) {
        float t = (2.0f * PI * i) / perLobe;
        pathX[pathCount] = LOBE_RADIUS_M - LOBE_RADIUS_M * cosf(t);
        pathY[pathCount] = LOBE_RADIUS_M * sinf(t);
        pathCount++;
    }

    // Left lobe: centre (-r, 0), traversed in the OPPOSITE rotational
    // direction (note +sinf, matching the right lobe rather than negating).
    //
    // This is what makes it a figure-eight instead of two circles joined
    // back-to-back. With the sign negated, the path tangent flipped ~187 deg
    // at each crossing, so the robot had to stop and spin on the spot before
    // it could continue. Traversing the second lobe this way keeps the tangent
    // continuous — max heading change at the crossing is now 7.2 deg, the same
    // as every other point on the path, so it drives straight through.
    for (int i = 0; i < perLobe; i++) {
        float t = (2.0f * PI * i) / perLobe;
        pathX[pathCount] = -LOBE_RADIUS_M + LOBE_RADIUS_M * cosf(t);
        pathY[pathCount] = LOBE_RADIUS_M * sinf(t);
        pathCount++;
    }

    Serial.printf("[path] figure-eight: %d points, r=%.2f m, spacing %.3f m\n",
                  pathCount, LOBE_RADIUS_M, PATH_POINT_SPACING_M);
    Serial.printf("[path] spans %.2f m wide x %.2f m tall\n",
                  4.0f * LOBE_RADIUS_M, 2.0f * LOBE_RADIUS_M);
}

// Distance from the robot to a given path point.
static float distToPoint(int idx) {
    float dx = pathX[idx] - currentX;
    float dy = pathY[idx] - currentY;
    return sqrtf(dx * dx + dy * dy);
}

// Advance pathIndex to the nearest point WITHIN A FORWARD WINDOW.
//
// Searching the whole path would be wrong here: at the crossing the nearest
// point may belong to the other lobe, and the robot would loop one circle
// forever. Searching only ahead makes progress monotonic.
static void advancePathIndex() {
    int   bestIdx  = pathIndex;
    float bestDist = distToPoint(pathIndex);

    for (int k = 1; k <= PATH_SEARCH_WINDOW; k++) {
        int idx = (pathIndex + k) % pathCount;
        float d = distToPoint(idx);
        if (d < bestDist) {
            bestDist = d;
            bestIdx  = idx;
        }
    }

    if (bestIdx != pathIndex) {
        // Detect wrap so we can count laps.
        if (bestIdx < pathIndex) lapCount++;
        pathIndex = bestIdx;
    }
}

// Walk forward from pathIndex until we have accumulated LOOKAHEAD_M of path
// length. That point is the pure-pursuit goal.
static int findGoalPoint() {
    float accumulated = 0.0f;
    int   idx  = pathIndex;
    int   prev = idx;

    for (int k = 1; k <= pathCount; k++) {
        idx = (pathIndex + k) % pathCount;
        float dx = pathX[idx] - pathX[prev];
        float dy = pathY[idx] - pathY[prev];
        accumulated += sqrtf(dx * dx + dy * dy);
        prev = idx;
        if (accumulated >= LOOKAHEAD_M) break;
    }
    return idx;
}

// =============================================================================
// PURE PURSUIT
// =============================================================================
//
// Standard formulation. Transform the goal into the robot frame, take the
// lateral offset y_r, and the arc joining robot to goal has curvature
//
//     kappa = 2 * y_r / Ld^2
//
// Differential drive converts curvature to a wheel speed split directly:
// omega = v * kappa, then v_left/right = v -/+ omega * track / 2.

static float lastCurvature = 0.0f;

static void pursuitStep() {
    advancePathIndex();
    int goalIdx = findGoalPoint();

    float dx = pathX[goalIdx] - currentX;
    float dy = pathY[goalIdx] - currentY;

    // World -> robot frame. currentHeading is CCW-positive radians.
    float c = cosf(currentHeading);
    float sn = sinf(currentHeading);
    float xr =  c * dx + sn * dy;
    float yr = -sn * dx + c * dy;

    float ld = sqrtf(dx * dx + dy * dy);
    if (ld < 0.02f) ld = 0.02f;              // guard divide-by-zero

    float kappa = 2.0f * yr / (ld * ld);
    kappa = constrain(kappa, -MAX_CURVATURE, MAX_CURVATURE);
    lastCurvature = kappa;

    // If the goal is behind us (xr < 0) the arc solution is degenerate.
    // Spin toward it in place rather than driving a nonsense curve.
    if (xr < 0.0f && fabsf(yr) > 0.05f) {
        float spinRpm = TURN_IN_PLACE_RPM * ((yr > 0) ? 1.0f : -1.0f);
        driveL.setpoint = -spinRpm;
        driveR.setpoint =  spinRpm;
    } else {
        float v = CRUISE_MPS;
        float omega = v * kappa;                     // rad/s
        float vL = v - omega * (TRACK_WIDTH_M / 2.0f);
        float vR = v + omega * (TRACK_WIDTH_M / 2.0f);

        // m/s -> RPM at the wheel.
        float toRpm = 60.0f / (PI * WHEEL_DIAMETER_M);
        driveL.setpoint = vL * toRpm;
        driveR.setpoint = vR * toRpm;
    }

    int16_t lRaw = readTicks(leftUnit);
    int16_t rRaw = readTicks(rightUnit);
    driveUpdateRPM(driveL, lRaw);
    driveUpdateRPM(driveR, rRaw);

    // allStop() (called from pursuitStop() on every safety gate) pulls STBY
    // low, which puts the driver in standby. Without re-enabling it here the
    // PWM is written to a sleeping driver and the robot never moves.
    enableDriver();

    setLeftMotor(driveControlLaw(driveL));
    setRightMotor(driveControlLaw(driveR));
}

static void pursuitStop() {
    driveL.setpoint = 0.0f;
    driveR.setpoint = 0.0f;
    driveResetWheel(driveL);
    driveResetWheel(driveR);
    allStop();
}

static void connectWiFi() {
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);          // power save drops broadcast packets mid-move
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
        Serial.printf("  Pose          : (%.3f, %.3f) hdg %.2f deg\n",
                      currentX, currentY, currentHeading * 180.0f / PI);
    } else {
        Serial.println("  Pose          : NOT SEEN (marker not detected?)");
    }
    Serial.println("---- SETTINGS ----");
    Serial.printf("  path: figure-8 r=%.2f m (%.2f x %.2f m), %d pts\n",
                  LOBE_RADIUS_M, 4.0f * LOBE_RADIUS_M, 2.0f * LOBE_RADIUS_M,
                  pathCount);
    Serial.printf("  pursuit: lookahead %.3f m  cruise %.2f m/s  maxK %.1f\n",
                  LOOKAHEAD_M, CRUISE_MPS, MAX_CURVATURE);
    Serial.printf("  drive: Kv=%.3f Ks=%.1f Kp=%.2f Ki=%.2f\n",
                  DRIVE_KV, DRIVE_KS, DRIVE_KP, DRIVE_KI);
    Serial.printf("  TICKS_PER_DEG = %.4f   TICKS_PER_METER = %.1f\n",
                  TICKS_PER_DEG, TICKS_PER_METER);
    Serial.printf("  spin: %s   batt comp: %s (%.2f V)\n",
                  INVERT_SPIN ? "INVERTED" : "normal",
                  USE_BATT_COMP ? "on" : "off", readBatteryVoltage());
    Serial.println("------------------\n");
}

// =============================================================================
// PATH ACQUISITION
// =============================================================================
//
// On start, or after losing the path, snap to whichever waypoint is nearest.
// This is the ONLY place a full-path search is allowed — during normal
// following the search is windowed and forward-only.

static void acquirePath() {
    int bestIdx = 0;
    float bestDist = 1e9f;
    for (int i = 0; i < pathCount; i++) {
        float d = distToPoint(i);
        if (d < bestDist) { bestDist = d; bestIdx = i; }
    }
    pathIndex = bestIdx;
    Serial.printf("[path] acquired at point %d (%.3f, %.3f), %.3f m away\n",
                  pathIndex, pathX[pathIndex], pathY[pathIndex], bestDist);
}

// =============================================================================
// SETUP
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

    ledcAttach(PWMA_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWMB_PIN, PWM_FREQ, PWM_RESOLUTION);

    allStop();

    leftUnit  = setupEncoder(LEFT_ENC_A,  LEFT_ENC_B,  true);
    rightUnit = setupEncoder(RIGHT_ENC_A, RIGHT_ENC_B, false);

    driveResetWheel(driveL);
    driveResetWheel(driveR);
    driveL.Kv = DRIVE_KV; driveL.Ks = DRIVE_KS;
    driveR.Kv = DRIVE_KV; driveR.Ks = DRIVE_KS;

    Serial.println("\n=== FIGURE EIGHT (pure pursuit) ===");
    Serial.printf("Robot ID %d, cruise %.2f m/s, lookahead %.3f m\n",
                  THIS_ROBOT_ID, CRUISE_MPS, LOOKAHEAD_M);

    buildFigureEight();

    connectWiFi();
    printStatus();
}

// =============================================================================
// LOOP
// =============================================================================
//
// Unlike approach_origin.ino this does NOT block for the duration of a
// movement. Each pass is one control step: read the latest pose, compute one
// pursuit update, write the motors, return. Everything stays responsive, so a
// STOP lands within one cycle.

void loop() {
    pollPackets();

    static unsigned long lastPrint   = 0;
    static unsigned long lastControl = 0;
    static bool          haveAcquired = false;
    bool doPrint = (millis() - lastPrint > PURSUIT_PRINT_MS);

    // ── Safety gates ───────────────────────────────────────────────────────
    if (!linkAlive()) {
        pursuitStop();
        // NOTE: haveAcquired is deliberately NOT reset here. Brief packet gaps
        // are normal; re-acquiring on every one meant the controller only ever
        // got a few consecutive steps before being stopped again, which is not
        // enough to overcome stiction. The path-lost check below handles the
        // case where we genuinely drifted away while blind.
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
        pursuitStop();
        haveAcquired = false;
        if (doPrint) {
            lastPrint = millis();
            Serial.println("[stop] server says STOP");
        }
        delay(20);
        return;
    }
    if (!haveHeading) {
        pursuitStop();
        if (doPrint) {
            lastPrint = millis();
            Serial.printf("[wait] link OK (pkt %lu) but marker %d not in packets\n",
                          (unsigned long)lastPacketNum, THIS_ROBOT_ID);
        }
        delay(20);
        return;
    }

    // ── Sentinel guard ─────────────────────────────────────────────────────
    // tracker.py sends (-9999, -9999) when the marker is not detected. The
    // packet still arrives, so haveHeading stays true — the VALUES have to be
    // checked. Without this the controller would swerve violently toward a
    // point 14 km away.
    if (currentX < -9000.0f || currentY < -9000.0f) {
        pursuitStop();
        if (doPrint) {
            lastPrint = millis();
            Serial.println("[wait] pose is LOST sentinel — marker not detected");
        }
        delay(20);
        return;
    }

    // ── Acquire the path on first good pose ────────────────────────────────
    if (!haveAcquired) {
        acquirePath();
        haveAcquired = true;
        lapCount = 0;
    }

    // ── Lost the path? Re-acquire rather than lurching toward it. ───────────
    float distToPath = distToPoint(pathIndex);
    if (distToPath > PATH_LOST_DIST_M) {
        Serial.printf("[path] lost (%.3f m from point %d) — re-acquiring\n",
                      distToPath, pathIndex);
        pursuitStop();
        acquirePath();
        delay(20);
        return;
    }

    // ── One control step ───────────────────────────────────────────────────
    // Paced to the drive loop rate the PI gains were tuned at.
    if (millis() - lastControl < (DRIVE_LOOP_US / 1000)) {
        delay(1);
        return;
    }
    lastControl = millis();

    pursuitStep();

    if (doPrint) {
        lastPrint = millis();
        Serial.printf("[run] pos(%.3f,%.3f) hdg%6.1f  pt%3d/%d lap%d  "
                      "xtrack%.3f  k%+.2f  rpm L%+.0f R%+.0f\n",
                      currentX, currentY, currentHeading * 180.0f / PI,
                      pathIndex, pathCount, lapCount,
                      distToPath, lastCurvature,
                      driveL.setpoint, driveR.setpoint);
    }
}
