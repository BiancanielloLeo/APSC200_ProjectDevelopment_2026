// =============================================================================
// FOLLOW THE LEADER — breadcrumb trail pursuit
// =============================================================================
//
// This robot follows another robot by driving the path the leader ACTUALLY
// DROVE, holding a fixed gap behind it.
//
// WHY A TRAIL AND NOT "DRIVE AT THE LEADER"
//
//   Aiming straight at the leader's current position cuts every corner. On a
//   circle the follower steers at a chord instead of the arc, so it spirals
//   inward and ends up driving a smaller circle than the leader — or crosses
//   the middle entirely. Aiming at a point AHEAD of the leader is worse: the
//   follower tries to overtake, and since it is faster, it rams.
//
//   Instead the leader's positions are recorded as breadcrumbs. The follower
//   targets the crumb that is STANDOFF_M of TRAIL LENGTH behind the leader.
//   It therefore traces the leader's exact route, one gap behind, and on a
//   circle it drives the same circle.
//
// SPEED
//
//   Constant speed does not work for following — too slow and the gap opens
//   forever, too fast and you collide. The follower estimates the leader's
//   speed from the crumbs and matches it, plus a proportional term on how far
//   it is from its target crumb. Sitting exactly on target means matching the
//   leader's speed exactly.
//
// EASY CONFIG:
//   THIS_ROBOT_ID    -> the follower's ArUco marker ID
//   LEADER_ROBOT_ID  -> the leader's ArUco marker ID
//   STANDOFF_M       -> how far behind the leader to sit
//
// =============================================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/pulse_cnt.h"

// ── EASY CONFIG ────────────────────────────────────────────────────────────
#define THIS_ROBOT_ID      5      // Follower's ArUco marker ID
#define LEADER_ROBOT_ID    1      // Leader's ArUco marker ID

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
//  FOLLOWING BEHAVIOUR
// ═══════════════════════════════════════════════════════════════════════════

// How far behind the leader to sit, measured ALONG THE TRAIL (not straight
// line). On a circle these differ: 0.40 m of arc is a shorter straight line.
// Must be comfortably larger than MIN_GAP_M.
float STANDOFF_M = 0.40f;

// Hard floor on straight-line distance to the leader. Inside this the
// follower stops regardless of what the pursuit maths wants. This is the
// anti-collision backstop, not a control target.
float MIN_GAP_M = 0.22f;

// Distance the leader must move before a new breadcrumb is recorded.
// Smaller = finer trail but a shorter history for the same buffer.
float CRUMB_SPACING_M = 0.04f;

// Ceiling on follower speed. Must exceed the leader's cruise (0.15 m/s in
// circle_loop.ino) or the gap can never be closed after a dropout — but only
// by enough to catch up at a sensible rate, not to sprint.
//
// At 0.20 the follower closes a gap at 0.05 m/s, so a 1 m deficit takes about
// 20 seconds to erase. Raise this if catch-up feels too slow; lower it (or
// lower the LEADER's CRUISE_MPS) if the follower looks frantic.
float MAX_FOLLOW_MPS = 0.20f;

// Proportional gain on gap error, (m/s) per metre. Higher closes gaps faster
// but overshoots into the leader. At 0.35 the follower only asks for full
// speed once it is ~0.15 m behind its target, so small errors produce small
// corrections instead of immediately pinning the throttle.
float GAP_KP = 0.35f;

// Below this speed the follower just stops rather than creeping — the motors
// cannot deliver useful torque down here anyway.
float MIN_MOVE_MPS = 0.04f;

// Curvature limit, 1/m. 3.0 is a 0.33 m turning radius — tight enough to
// recover from real error, loose enough that it never snaps the wheels round.
float MAX_CURVATURE = 3.0f;

// MINIMUM distance used in the steering maths, metres.
//
// Pure pursuit steers with kappa = 2*sin(alpha)/ld. The target crumb is what
// sets the SPEED, but it is the wrong thing to steer at directly: at steady
// state the follower sits right on that crumb, ld collapses toward zero, and
// kappa explodes into the curvature limit. The better the tracking, the
// harder it oversteers.
//
// Clamping the steering distance to a floor decouples the two. Speed still
// uses the true distance to the crumb; steering behaves as though the goal
// were never closer than this. Same role LOOKAHEAD_M plays in the circle.
//
// Too small: twitchy, oversteers. Too large: lazy, cuts corners on the trail.
float STEER_LOOKAHEAD_MIN_M = 0.20f;

// Turn-in-place when the target crumb is off to the side or behind, with
// hysteresis so it cannot chatter against the arc controller.
//
// Entry at 110 deg rather than 90: spinning stops the robot dead, so it
// should be reserved for "the target is genuinely behind me", not for a
// wide-but-drivable turn the arc controller could handle while moving.
float SPIN_ENTER_DEG    = 110.0f;
float SPIN_EXIT_DEG     = 25.0f;
float TURN_IN_PLACE_RPM = 35.0f;

// How long the leader can be missing before the follower stops driving.
#define LEADER_TIMEOUT_MS  1500

// How long the leader must be missing before the breadcrumb TRAIL is thrown
// away. Deliberately much longer than LEADER_TIMEOUT_MS.
//
// Stopping and discarding the trail are different decisions. A 1-second
// packet gap means "don't drive on stale data" — but the trail is still a
// perfectly good record of where the leader went, and the leader cannot have
// gone far in that time. Wiping it on every brief gap forces a full
// STANDOFF_M rebuild (~3 s of sitting in "wait trail") to recover from a
// dropout that lasted under a second, which is what turns network hiccups
// into visible stop-start motion.
//
// Only after several seconds is the leader genuinely somewhere unknown and
// the old trail worth discarding.
#define TRAIL_DISCARD_MS   4000

// ── DRIVE — feedforward + PI velocity loop ─────────────────────────────────
float DRIVE_KV = 1.0f;
float DRIVE_KS = 46.0f;
float DRIVE_KP = 2.5f;
float DRIVE_KI = 1.0f;

float DRIVE_MAX_DUTY = 178.0f;   // 70% of 255
#define DRIVE_LOOP_HZ    100
#define DRIVE_LOOP_MS    (1000 / DRIVE_LOOP_HZ)
#define RPM_FILTER_ALPHA 0.3f
#define RPM_GLITCH_CEIL  200.0f

// ── Geometry ───────────────────────────────────────────────────────────────
#define TICKS_PER_REV      1400.0f
#define WHEEL_DIAMETER_M   0.065f
#define TRACK_WIDTH_M      0.125f
static const float MPS_TO_RPM = 60.0f / (PI * WHEEL_DIAMETER_M);

// ── Freshness / safety ─────────────────────────────────────────────────────
// Observed broadcast gaps on this setup run 700-1200 ms, so these are set to
// ride through them rather than tripping on every one. That is tolerating a
// symptom: the gaps come from the tracker starving its own broadcast thread,
// and fixing that upstream lets all three of these come back down to ~800.
#define PACKET_TIMEOUT_MS  2000
#define POSE_TIMEOUT_MS    1500
#define LOST_SENTINEL_M   -9000.0f

bool INVERT_SPIN = false;

// ── Battery monitoring (reporting only) ────────────────────────────────────
#define R1              100000.0f
#define R2               47000.0f
#define DIVIDER_RATIO   (R2 / (R1 + R2))
#define ADC_RESOLUTION  4095.0f
#define ADC_VREF            3.3f

// ── Encoder / PCNT ─────────────────────────────────────────────────────────
#define PCNT_HIGH_LIMIT   32767
#define PCNT_LOW_LIMIT   -32768
#define GLITCH_FILTER_NS  12500

#define PRINT_MS  500

// ═══════════════════════════════════════════════════════════════════════════

static pcnt_unit_handle_t leftUnit = NULL, rightUnit = NULL;
static WiFiUDP udp;
static uint8_t rxBuf[512];

// ── Link / vision state ────────────────────────────────────────────────────
static bool          serverRun      = false;

static bool          haveHeading    = false;
static bool          newPose        = false;
static float         currentX = 0.0f, currentY = 0.0f, currentHeading = 0.0f;
static unsigned long lastHeadingMs  = 0;

static bool          haveLeader     = false;
static float         leaderX = 0.0f, leaderY = 0.0f, leaderHeading = 0.0f;
static unsigned long lastLeaderMs   = 0;
static float         leaderSpeed    = 0.0f;   // m/s, filtered

static unsigned long lastPacketMs   = 0;

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

// =============================================================================
// MOTOR CONTROL
// =============================================================================
//
// Duty is carried as FLOAT all the way to ledcWrite. The previous version
// passed it through an int8_t parameter, which silently wraps above 127 — a
// requested duty of 178 arrived as -78 and drove the motor backwards at speed.

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

static void allStop() {
    setLeftMotor(0);
    setRightMotor(0);
    digitalWrite(STBY_PIN, LOW);
}

static float readBatteryVoltage() {
    long sum = 0;
    const int N = 8;
    for (int i = 0; i < N; i++) sum += analogRead(BATTERY_PIN);
    float pinVolts = ((sum / (float)N) / ADC_RESOLUTION) * ADC_VREF;
    return pinVolts / DIVIDER_RATIO;
}

// =============================================================================
// PACKET PARSING  —  "!cIB" + "!Bfff", BIG-ENDIAN
// =============================================================================
//
// The '!' in udp.py's struct format means network byte order = big-endian.
// The ESP32 is little-endian. A straight memcpy of the four bytes therefore
// produces a completely different float (0.5 decodes as 8.8e-44). The bytes
// MUST be reversed, which is what be32/beFloat do.

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
#define ROBOT_ENTRY_SIZE  13     // struct.calcsize("!Bfff")

// Forward declaration — defined with the breadcrumb code below.
static void pushCrumb(float x, float y, unsigned long nowMs);

static void pollPackets() {
    while (udp.parsePacket() > 0) {
        int len = udp.read(rxBuf, sizeof(rxBuf));
        if (len < HDR_SIZE) continue;

        uint8_t  cmd     = rxBuf[0];
        uint8_t  nRobots = rxBuf[5];
        if (len < HDR_SIZE + (int)nRobots * ROBOT_ENTRY_SIZE) continue;

        serverRun    = (cmd == 'R');
        lastPacketMs = millis();

        for (uint8_t i = 0; i < nRobots; i++) {
            const uint8_t* e = &rxBuf[HDR_SIZE + i * ROBOT_ENTRY_SIZE];
            uint8_t rid = e[0];

            if (rid != THIS_ROBOT_ID && rid != LEADER_ROBOT_ID) continue;

            float x = beFloat(&e[1]);
            float y = beFloat(&e[5]);
            float t = beFloat(&e[9]);

            if (x < LOST_SENTINEL_M || y < LOST_SENTINEL_M) continue;

            if (rid == THIS_ROBOT_ID) {
                currentX = x; currentY = y; currentHeading = t;
                haveHeading = true;
                newPose = true;
                lastHeadingMs = millis();
            }
            if (rid == LEADER_ROBOT_ID) {
                leaderX = x; leaderY = y; leaderHeading = t;
                haveLeader = true;
                pushCrumb(x, y, millis());
                lastLeaderMs = millis();
            }
        }
    }
}

static bool linkAlive()    { return (millis() - lastPacketMs)  < PACKET_TIMEOUT_MS; }
static bool poseFresh()    { return haveHeading && (millis() - lastHeadingMs) < POSE_TIMEOUT_MS; }
static bool leaderFresh()  { return haveLeader  && (millis() - lastLeaderMs)  < LEADER_TIMEOUT_MS; }

// =============================================================================
// DRIVE — feedforward + PI velocity loop
// =============================================================================

struct DriveWheel {
    float Kv, Ks;
    float setpoint;
    float rpm;
    float integral;
    int16_t lastTicks;
};

static DriveWheel driveL, driveR;

static void  driveResetWheel(DriveWheel &w);
static void  driveUpdateRPM(DriveWheel &w, int16_t rawTicks, float dt);
static float driveControlLaw(DriveWheel &w, float dt);

static void driveResetWheel(DriveWheel &w) {
    w.setpoint = 0.0f; w.rpm = 0.0f; w.integral = 0.0f; w.lastTicks = 0;
}

static void driveResyncTicks() {
    driveL.lastTicks = readTicks(leftUnit);
    driveR.lastTicks = readTicks(rightUnit);
}

static void driveUpdateRPM(DriveWheel &w, int16_t rawTicks, float dt) {
    int16_t delta = rawTicks - w.lastTicks;
    w.lastTicks = rawTicks;
    if (dt < 1e-4f) return;

    float rawRpm = ((float)delta / TICKS_PER_REV) * (60.0f / dt);
    if (fabsf(rawRpm) > RPM_GLITCH_CEIL) return;
    w.rpm += RPM_FILTER_ALPHA * (rawRpm - w.rpm);
}

static float driveControlLaw(DriveWheel &w, float dt) {
    if (fabsf(w.setpoint) < 0.5f) { w.integral = 0.0f; return 0.0f; }

    float err = w.setpoint - w.rpm;
    float dir = (w.setpoint >= 0) ? 1.0f : -1.0f;
    float ff  = dir * (w.Kv * fabsf(w.setpoint) + w.Ks);

    float newIntegral = w.integral + err * dt;
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

static void driveTick(float dt) {
    driveUpdateRPM(driveL, readTicks(leftUnit),  dt);
    driveUpdateRPM(driveR, readTicks(rightUnit), dt);
    enableDriver();
    setLeftMotor(driveControlLaw(driveL,  dt));
    setRightMotor(driveControlLaw(driveR, dt));
}

// =============================================================================
// BREADCRUMB TRAIL
// =============================================================================
//
// A ring buffer of places the leader has been. A crumb is recorded only once
// the leader has moved CRUMB_SPACING_M from the last one, so the trail is
// evenly spaced in DISTANCE rather than in time — a leader that stops does
// not flood the buffer with duplicates.
//
// 128 crumbs at 0.04 m spacing is about 5.1 m of history, comfortably more
// than one lap of the 0.60 m circle (3.77 m circumference).

#define CRUMB_MAX  128

struct Crumb { float x, y; unsigned long t; };

static Crumb crumbs[CRUMB_MAX];
static int   crumbHead  = 0;    // index of the newest crumb
static int   crumbCount = 0;

static void pushCrumb(float x, float y, unsigned long nowMs) {
    if (crumbCount > 0) {
        Crumb &last = crumbs[crumbHead];
        float dx = x - last.x, dy = y - last.y;
        float moved = sqrtf(dx * dx + dy * dy);
        if (moved < CRUMB_SPACING_M) return;    // not far enough yet

        // Leader speed estimate, from how long that move took.
        unsigned long dtMs = nowMs - last.t;
        if (dtMs > 0 && dtMs < 2000) {
            float v = moved / (dtMs / 1000.0f);
            if (v < 1.0f) leaderSpeed += 0.3f * (v - leaderSpeed);
        }
    }

    crumbHead = (crumbHead + 1) % CRUMB_MAX;
    crumbs[crumbHead] = { x, y, nowMs };
    if (crumbCount < CRUMB_MAX) crumbCount++;
}

static void clearCrumbs() {
    crumbHead = 0;
    crumbCount = 0;
    leaderSpeed = 0.0f;
}

// Walk backwards along the trail from the newest crumb, accumulating distance
// until STANDOFF_M has been covered. That crumb is the follow target.
//
// Returns false if the trail is too short — the leader has not driven far
// enough yet for there to be a point STANDOFF_M behind it.
static bool findTargetCrumb(float &tx, float &ty, float &trailLen) {
    if (crumbCount < 2) return false;

    float accumulated = 0.0f;
    int idx = crumbHead;

    for (int k = 1; k < crumbCount; k++) {
        int prev = (idx - 1 + CRUMB_MAX) % CRUMB_MAX;
        float dx = crumbs[idx].x - crumbs[prev].x;
        float dy = crumbs[idx].y - crumbs[prev].y;
        accumulated += sqrtf(dx * dx + dy * dy);
        idx = prev;

        if (accumulated >= STANDOFF_M) {
            tx = crumbs[idx].x;
            ty = crumbs[idx].y;
            trailLen = accumulated;
            return true;
        }
    }
    return false;   // trail exists but is shorter than the standoff
}

// =============================================================================
// FOLLOW CONTROL
// =============================================================================

static float lastCurvature = 0.0f;
static float lastAlphaDeg  = 0.0f;
static float lastGoalDist  = 0.0f;
static float lastCmdMps    = 0.0f;
static bool  spinning      = false;
static const char *lastMode = "init";

static void followStop() {
    driveResetWheel(driveL);
    driveResetWheel(driveR);
    spinning = false;
    allStop();
}

// If no new crumb has appeared for this long, the leader has stopped moving.
// At 0.15 m/s a 0.04 m crumb gap takes ~270 ms, so 500 ms means genuinely
// stationary rather than just between crumbs.
//
// This matters because the speed command is leaderSpeed + GAP_KP*distance.
// Without decaying the estimate, a stopped leader leaves leaderSpeed frozen
// at its last value, the follower never commands less than that, and it
// drives straight over its target crumb into the collision floor.
#define LEADER_STALL_MS  500

static void updateLeaderSpeedDecay() {
    if (crumbCount == 0) { leaderSpeed = 0.0f; return; }
    if (millis() - crumbs[crumbHead].t > LEADER_STALL_MS) leaderSpeed = 0.0f;
}

static void followUpdate() {
    updateLeaderSpeedDecay();

    // ── Anti-collision backstop ────────────────────────────────────────────
    float ldx = leaderX - currentX, ldy = leaderY - currentY;
    float gapToLeader = sqrtf(ldx * ldx + ldy * ldy);

    if (gapToLeader < MIN_GAP_M) {
        driveL.setpoint = 0.0f;
        driveR.setpoint = 0.0f;
        lastMode = "TOO CLOSE";
        lastCmdMps = 0.0f;
        return;
    }

    // ── Pick the goal ──────────────────────────────────────────────────────
    float gx, gy, trailLen;
    if (!findTargetCrumb(gx, gy, trailLen)) {
        // Trail too short. The leader has not moved far enough to have laid
        // down a point STANDOFF_M back, so there is nowhere legitimate to go.
        // Hold position rather than inventing a target and lunging at it.
        driveL.setpoint = 0.0f;
        driveR.setpoint = 0.0f;
        lastMode = "wait trail";
        lastCmdMps = 0.0f;
        return;
    }

    float dx = gx - currentX, dy = gy - currentY;
    float ld = sqrtf(dx * dx + dy * dy);
    if (ld < 0.01f) ld = 0.01f;
    lastGoalDist = ld;

    // ── World -> robot frame ───────────────────────────────────────────────
    float c  = cosf(currentHeading);
    float sn = sinf(currentHeading);
    float xr =  c * dx + sn * dy;
    float yr = -sn * dx + c * dy;

    float alpha = atan2f(yr, xr);
    lastAlphaDeg = alpha * 180.0f / PI;

    // ── Latching spin decision ─────────────────────────────────────────────
    if (!spinning && fabsf(lastAlphaDeg) > SPIN_ENTER_DEG) spinning = true;
    if ( spinning && fabsf(lastAlphaDeg) < SPIN_EXIT_DEG)  spinning = false;

    if (spinning) {
        // Goal is beside or behind us: rotate to face it before driving.
        // Direction depends on WHICH SIDE the goal is on — the old version
        // always spun the same way regardless, so half the time it turned
        // away from the target and had to go the long way round.
        float spinRpm = TURN_IN_PLACE_RPM * ((alpha > 0) ? 1.0f : -1.0f);
        if (INVERT_SPIN) spinRpm = -spinRpm;
        driveL.setpoint = -spinRpm;
        driveR.setpoint =  spinRpm;
        lastCurvature = 0.0f;
        lastCmdMps = 0.0f;
        lastMode = "spin";
        return;
    }

    // ── Speed: match the leader, plus a term for closing the gap ───────────
    // On target (ld == 0) this reduces to exactly the leader's speed, which
    // is the steady state we want. Falling behind adds speed proportionally.
    float v = leaderSpeed + GAP_KP * ld;
    v = constrain(v, 0.0f, MAX_FOLLOW_MPS);

    if (v < MIN_MOVE_MPS) {
        driveL.setpoint = 0.0f;
        driveR.setpoint = 0.0f;
        lastCmdMps = 0.0f;
        lastMode = "hold";
        return;
    }
    lastCmdMps = v;
    lastMode = "follow";

    // ── Steering: pure pursuit toward the target crumb ─────────────────────
    // ld (true distance) set the speed above. Steering uses the clamped
    // version so that closing in on the crumb does not spike the curvature.
    float ldSteer = (ld < STEER_LOOKAHEAD_MIN_M) ? STEER_LOOKAHEAD_MIN_M : ld;

    float kappa = 2.0f * sinf(alpha) / ldSteer;
    kappa = constrain(kappa, -MAX_CURVATURE, MAX_CURVATURE);
    lastCurvature = kappa;

    float omega = v * kappa;
    float vL = v - omega * (TRACK_WIDTH_M / 2.0f);
    float vR = v + omega * (TRACK_WIDTH_M / 2.0f);

    driveL.setpoint = vL * MPS_TO_RPM;
    driveR.setpoint = vR * MPS_TO_RPM;
}

// =============================================================================
// WIFI
// =============================================================================

static void connectWiFi() {
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);      // power save silently drops broadcast frames
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
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(800);

    Serial.println("\n=== BUILD " __DATE__ " " __TIME__ " ===");
    Serial.println("=== FOLLOW THE LEADER (breadcrumb trail) ===");

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

    Serial.printf("Follower ID %d  ->  Leader ID %d\n", THIS_ROBOT_ID, LEADER_ROBOT_ID);
    Serial.printf("Standoff %.2f m, min gap %.2f m, max speed %.2f m/s\n",
                  STANDOFF_M, MIN_GAP_M, MAX_FOLLOW_MPS);
    Serial.printf("Battery %.2f V\n", readBatteryVoltage());

    connectWiFi();

    Serial.println("\nWaiting for: RUN command, own marker, and leader marker.\n");
}

// =============================================================================
// LOOP
// =============================================================================
//
// Two rates:
//   ~20 Hz (on a new camera pose) -> followUpdate() decides WHERE to go
//   100 Hz (timer)                -> driveTick() makes the wheels do it
//
// Critically, the safety gates call followStop(), which actually WRITES the
// motors off. The previous version only zeroed the setpoints and then
// returned before the drive loop ran, so the PWM registers kept their last
// value and the robot carried on driving through a link loss.

void loop() {
    pollPackets();

    static unsigned long lastPrint  = 0;
    static unsigned long lastTickMs = 0;
    static bool          wasStopped = true;

    bool doPrint = (millis() - lastPrint > PRINT_MS);

    // ── Safety gates ───────────────────────────────────────────────────────
    const char *stopReason = NULL;

    if (!linkAlive())         stopReason = "no packets from server";
    else if (!serverRun)      stopReason = "server says STOP";
    else if (!poseFresh())    stopReason = "my marker not detected";
    else if (!leaderFresh())  stopReason = "leader marker not detected";

    if (stopReason) {
        if (!wasStopped) { followStop(); wasStopped = true; }

        // A briefly missing leader is not a reason to throw away the trail —
        // only a long absence is. See TRAIL_DISCARD_MS.
        if (haveLeader && (millis() - lastLeaderMs) > TRAIL_DISCARD_MS) {
            if (crumbCount > 0) {
                Serial.printf("[trail] leader gone %lu ms — discarding %d crumbs\n",
                              millis() - lastLeaderMs, crumbCount);
            }
            clearCrumbs();
        }

        if (doPrint) {
            lastPrint = millis();
            Serial.printf("[hold] %s | pkt %lu ms, me %lu ms, ldr %lu ms\n",
                          stopReason, millis() - lastPacketMs,
                          millis() - lastHeadingMs, millis() - lastLeaderMs);
        }
        delay(5);
        return;
    }

    if (wasStopped) {
        driveResyncTicks();
        lastTickMs = millis();
        wasStopped = false;
    }

    // ── Geometry, only on a new pose ───────────────────────────────────────
    if (newPose) {
        newPose = false;
        followUpdate();
    }

    // ── Servo, fixed 100 Hz with measured dt ───────────────────────────────
    unsigned long now = millis();
    if (now - lastTickMs >= DRIVE_LOOP_MS) {
        float dt = (now - lastTickMs) / 1000.0f;
        lastTickMs = now;
        driveTick(dt);
    }

    if (doPrint) {
        lastPrint = millis();
        float ldx = leaderX - currentX, ldy = leaderY - currentY;
        Serial.printf("[%s] me(%+.2f,%+.2f) ldr(%+.2f,%+.2f) gap%.2f "
                      "crumbs%3d ldrV%.2f cmdV%.2f goal%.2f a%+6.1f k%+.2f "
                      "set L%+.0f R%+.0f act L%+.0f R%+.0f\n",
                      lastMode, currentX, currentY, leaderX, leaderY,
                      sqrtf(ldx * ldx + ldy * ldy),
                      crumbCount, leaderSpeed, lastCmdMps, lastGoalDist,
                      lastAlphaDeg, lastCurvature,
                      driveL.setpoint, driveR.setpoint, driveL.rpm, driveR.rpm);
    }
}
