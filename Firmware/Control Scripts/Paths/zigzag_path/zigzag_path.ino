// =============================================================================
// ZIGZAG PATH — pure pursuit path following (leader sketch)
// =============================================================================
//
// Drives a closed path forever, for the leader robot in the follow-the-leader
// pair. PATH_MODE selects the shape:
//
//   PATH_ZIGZAG_RECT (default) — a zigzag down the long axis of the arena,
//                                then a straight return along the bottom.
//   PATH_CIRCLE                — the original circle.
//
// Adding a shape means writing a vertex list and handing it to
// buildFromWaypoints(); the pursuit and drive code below never changes.
//
// Derived from figure_eight.ino. The drive stack (per-wheel feedforward + PI
// velocity loop, PCNT encoders, packet parsing) is the same validated code.
// Three structural things changed, all of which were causing erratic
// behaviour in the figure-eight sketch:
//
//   1. POSE STALENESS IS NOW CHECKED.
//      figure_eight.ino set lastHeadingMs on every pose update and then never
//      read it. udp.py keeps re-broadcasting a robot's last known position
//      forever (robot_positions.update() never deletes entries), so if the
//      marker drops out of detection the packets keep flowing, linkAlive()
//      stays true, haveHeading stays true, and the robot happily pure-pursues
//      a pose frozen in the past. It drives off and does not stop.
//      Now: no fresh pose within POSE_TIMEOUT_MS -> stop.
//
//   2. GEOMETRY IS RECOMPUTED ONLY ON A NEW POSE, NOT AT 100 Hz.
//      The pursuit solution only changes when the camera says something new
//      (~20 Hz). Running it at 100 Hz off a stale pose did no good and made
//      the timing hard to reason about. Now pursuitUpdate() runs on new poses
//      and sets the wheel RPM setpoints; driveTick() runs the PI loop at
//      100 Hz to hold those setpoints. Clean separation of the two rates.
//
//   3. THE TURN-IN-PLACE BRANCH HAS HYSTERESIS.
//      The old `if (xr < 0 && |yr| > 0.05)` test sits right on a boundary, so
//      a robot roughly side-on to its goal chattered between "spin in place"
//      and "drive an arc" every control step — the wheels reverse direction
//      on each flip. Now it latches: enter spin above SPIN_ENTER_DEG, stay
//      there until back inside SPIN_EXIT_DEG.
//
// The PI loop also uses measured dt instead of a hardcoded 1/100 s, and
// resyncs its tick baseline on resume instead of leaning on the glitch
// rejector to swallow the first bad sample after a stop.
//
// TUNING (rough order of what to try first):
//   LOOKAHEAD_M      too small -> weaves/oscillates; too large -> cuts inside
//   CRUISE_MPS       lower it if the robot cannot hold the arc
//   CIRCLE_RADIUS_M  keep 2*r + robot width inside the camera frame
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
//  THE CIRCLE
// ═══════════════════════════════════════════════════════════════════════════
//
// Arena is 3.60 m x 2.03 m with the world origin at the centre of the camera
// frame, so x is in [-1.80, +1.80] and y is in [-1.015, +1.015]. The binding
// dimension is y: the circle spans 2*r vertically, and the robot itself is
// 100 mm wide, so keep r below about 0.90 m. 0.60 m leaves comfortable margin
// for camera edge distortion and marker dropout near the frame border.

// ═══════════════════════════════════════════════════════════════════════════
//  PATH SELECTION
// ═══════════════════════════════════════════════════════════════════════════

#define PATH_CIRCLE        0
#define PATH_ZIGZAG_RECT   1

int PATH_MODE = PATH_ZIGZAG_RECT;

// ── Circle ─────────────────────────────────────────────────────────────────
float CIRCLE_RADIUS_M   = 0.60f;
float CIRCLE_CENTER_X   = 0.00f;
float CIRCLE_CENTER_Y   = 0.00f;
bool  CIRCLE_CCW        = true;

// ── Zigzag rectangle ───────────────────────────────────────────────────────
//
// A closed loop: a zigzag running the LONG way down the arena (the x axis),
// then a plain straight return along the bottom, joined by two short ends.
//
//   +y  ‾\    /\    /\    /‾        <- zigzag leg, left to right
//         \/    \/    \/
//        |                 |        <- short ends
//        |_________________|        <- straight return, right to left
//
// THE PITCH AND SWING ARE NOT INDEPENDENT.
//
// The diagonals meet at each zigzag vertex with an interior turn of
// 2*atan(SWING/PITCH). A follower with a minimum turning radius r needs
// r*tan(turn/2) of straight either side of that vertex, and the diagonals are
// only sqrt(PITCH^2 + SWING^2) long. Make the swing too large relative to the
// pitch and the corner stops being drivable no matter how it is tuned.
//
// At 0.40 / 0.20 the diagonals sit at 26.6 deg, the vertex turn is 53 deg,
// and a 0.33 m radius needs 0.16 m of run-in against 0.45 m available.
//
// NOTE ON MAKING THE ZIGZAG "LONGER":
//   Path length of the zigzag leg is span * sqrt(1 + (SWING/PITCH)^2). It
//   depends on the RATIO only, so halving both PITCH and SWING doubles the
//   number of zigs while leaving the distance travelled essentially the same.
//   More distance requires a steeper ratio, which sharpens the vertices.
//
//   The drivability limit, for a follower of minimum turn radius r, is:
//       PITCH >= 2*r*k / sqrt(1+k^2)    where k = SWING/PITCH
//   At r = 0.33 and k = 0.5 that floor is 0.295 m, so 0.40 still has room.
//   Going below it, or raising k much past 0.5, needs the follower's
//   MAX_CURVATURE raised (3.0 -> 4.5 puts r at 0.22 and the floor at 0.20).
float ZIG_X_MIN     = -1.40f;   // arena is +/-1.80 in x; leave room to turn
float ZIG_X_MAX     =  1.40f;
float ZIG_PITCH_M   =  0.40f;   // x advance per diagonal
float ZIG_Y_LO      =  0.45f;   // arena is +/-1.015 in y
float ZIG_Y_HI      =  0.65f;
float RETURN_Y      = -0.75f;   // straight leg home

// Spacing between generated waypoints.
float PATH_POINT_SPACING_M = 0.05f;

// ═══════════════════════════════════════════════════════════════════════════
//  PURE PURSUIT
// ═══════════════════════════════════════════════════════════════════════════

// THE tuning knob. Distance ahead along the path to aim at.
// Too small: the robot weaves and can oscillate. Too large: it cuts inside
// the circle and settles on a smaller radius. Start 0.15, step by 0.02.
//
// Rule of thumb: lookahead should be roughly CRUISE_MPS * (1 to 1.5 s) and
// at least 2x the camera position noise. Below ~0.10 m at this speed the
// vision latency alone will make it unstable.
float LOOKAHEAD_M = 0.15f;

// Forward travel speed, m/s. 0.15 m/s is about 44 RPM at the wheel.
float CRUISE_MPS  = 0.15f;

// How many points ahead to consider when re-locating ourselves on the path.
// Large enough to keep up if we fall behind, small enough that we cannot
// teleport forward past a section we have not driven.
#define PATH_SEARCH_WINDOW  12

// Curvature limit, 1/m. Caps how hard it will try to turn. A circle of
// r = 0.60 needs kappa = 1.67 in steady state, so 6.0 leaves plenty of
// authority for cutting back onto the path without commanding a reversal.
float MAX_CURVATURE = 6.0f;

// Turn-in-place when the goal is well off to the side or behind, with
// hysteresis so it cannot chatter against the arc controller.
float SPIN_ENTER_DEG   = 90.0f;   // above this bearing error -> spin
float SPIN_EXIT_DEG    = 25.0f;   // below this -> back to arcing
float TURN_IN_PLACE_RPM = 35.0f;

// Further than this from the tracked path point means we have lost the path
// (picked up, shoved, or bad camera data). Re-acquire from scratch.
float PATH_LOST_DIST_M = 0.45f;

// ── DRIVE — feedforward + PI velocity loop ─────────────────────────────────
// Per-wheel feedforward: duty = Kv * |RPM| + Ks. Shared PI on top.
// Fitted defaults from the tuning rig; re-run 'ff' there if the floor
// surface changes and update these.
float DRIVE_KV = 1.0f;
float DRIVE_KS = 46.0f;
float DRIVE_KP = 2.5f;
float DRIVE_KI = 1.0f;

float DRIVE_MAX_DUTY = 178.0f;   // 70% of 255 — motors are 6 V on a 7.2 V pack
#define DRIVE_LOOP_HZ    100
#define DRIVE_LOOP_MS    (1000 / DRIVE_LOOP_HZ)
#define RPM_FILTER_ALPHA 0.3f
#define RPM_GLITCH_CEIL  200.0f  // motor no-load max; above this is a bus glitch

// ── Geometry ───────────────────────────────────────────────────────────────
#define TICKS_PER_REV      1400.0f
#define WHEEL_DIAMETER_M   0.065f
#define TRACK_WIDTH_M      0.125f
static const float MPS_TO_RPM = 60.0f / (PI * WHEEL_DIAMETER_M);

// ── Freshness / safety ─────────────────────────────────────────────────────
// PACKET_TIMEOUT_MS  — any packet at all from the server (proves the link).
// POSE_TIMEOUT_MS    — a packet that actually CONTAINED THIS ROBOT with a
//                      valid position. This is the one that matters: udp.py
//                      rebroadcasts stale entries indefinitely, so a live
//                      link says nothing about whether the camera can
//                      currently see us.
//
// Both are generous because the observed broadcast gaps on this setup run
// 520-960 ms. That is a tracker-side problem worth fixing (see notes at the
// bottom of this file) — these numbers are tolerating a symptom, not a spec.
#define PACKET_TIMEOUT_MS  1200
#define POSE_TIMEOUT_MS     800

// tracker.py may send this instead of a real position when a robot is
// expected but not detected.
#define LOST_SENTINEL_M   -9000.0f

// ── Direction ──────────────────────────────────────────────────────────────
// Camera theta is CCW-positive, 0 deg = +X axis.
// If the robot drives backwards along the path, the motor outputs are
// swapped — check wiring before flipping this.
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

// ── Status print interval, ms ──────────────────────────────────────────────
#define PRINT_MS  500

// ═══════════════════════════════════════════════════════════════════════════

static pcnt_unit_handle_t leftUnit = NULL, rightUnit = NULL;
static WiFiUDP udp;
static uint8_t rxBuf[512];

// ── Link / vision state ────────────────────────────────────────────────────
static bool          serverRun      = false;
static bool          haveHeading    = false;
static bool          newPose        = false;   // set on each fresh pose, cleared when consumed
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

// =============================================================================
// MOTOR CONTROL
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

static void allStop() {
    setLeftMotor(0);
    setRightMotor(0);
    digitalWrite(STBY_PIN, LOW);
}

// =============================================================================
// BATTERY (status reporting only — the PI integral absorbs sag on its own)
// =============================================================================

static float readBatteryVoltage() {
    long sum = 0;
    const int N = 8;
    for (int i = 0; i < N; i++) sum += analogRead(BATTERY_PIN);
    float pinVolts = ((sum / (float)N) / ADC_RESOLUTION) * ADC_VREF;
    return pinVolts / DIVIDER_RATIO;
}

// =============================================================================
// PACKET PARSING  —  "!cIB" + "!Bfff", big-endian, unpadded
// =============================================================================

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
#define ROBOT_ENTRY_SIZE  13     // struct.calcsize("!Bfff") -> id, x, y, theta

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

            float x = beFloat(&e[1]);
            float y = beFloat(&e[5]);
            float t = beFloat(&e[9]);

            // Sentinel guard. The entry is present but means "not detected".
            if (x < LOST_SENTINEL_M || y < LOST_SENTINEL_M) break;

            currentX       = x;
            currentY       = y;
            currentHeading = t;
            haveHeading    = true;
            newPose        = true;
            lastHeadingMs  = millis();
            break;
        }
    }
}

static bool linkAlive() { return (millis() - lastPacketMs)  < PACKET_TIMEOUT_MS; }
static bool poseFresh() { return haveHeading &&
                                 (millis() - lastHeadingMs) < POSE_TIMEOUT_MS; }

// =============================================================================
// DRIVE — feedforward + PI velocity loop
// =============================================================================
// Same control law as the validated sketches. Two changes: the sample period
// is measured rather than assumed, and the tick baseline can be resynced
// without faking a glitch.

struct DriveWheel {
    float Kv, Ks;
    float setpoint;      // RPM
    float rpm;           // filtered measurement
    float integral;
    int16_t lastTicks;
};

static DriveWheel driveL, driveR;

// Manual forward declarations. Arduino's auto-prototype generator inserts its
// own prototypes above the struct definition, which will not compile.
static void  driveResetWheel(DriveWheel &w);
static void  driveUpdateRPM(DriveWheel &w, int16_t rawTicks, float dt);
static float driveControlLaw(DriveWheel &w, float dt);

static void driveResetWheel(DriveWheel &w) {
    w.setpoint  = 0.0f;
    w.rpm       = 0.0f;
    w.integral  = 0.0f;
    w.lastTicks = 0;
}

// Called on resume so the first sample after a stop is a real delta rather
// than a jump from a stale baseline.
static void driveResyncTicks() {
    driveL.lastTicks = readTicks(leftUnit);
    driveR.lastTicks = readTicks(rightUnit);
}

static void driveUpdateRPM(DriveWheel &w, int16_t rawTicks, float dt) {
    int16_t delta = rawTicks - w.lastTicks;   // int16 wrap handled by the type
    w.lastTicks = rawTicks;
    if (dt < 1e-4f) return;

    float rawRpm = ((float)delta / TICKS_PER_REV) * (60.0f / dt);
    if (fabsf(rawRpm) > RPM_GLITCH_CEIL) return;
    w.rpm += RPM_FILTER_ALPHA * (rawRpm - w.rpm);
}

static float driveControlLaw(DriveWheel &w, float dt) {
    if (fabsf(w.setpoint) < 0.5f) {
        w.integral = 0.0f;
        return 0.0f;
    }

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

    // Conditional integration anti-windup.
    float unclamped = ff + DRIVE_KP * err + DRIVE_KI * newIntegral;
    bool satHigh = (unclamped >  DRIVE_MAX_DUTY) && (err > 0);
    bool satLow  = (unclamped < -DRIVE_MAX_DUTY) && (err < 0);
    if (!satHigh && !satLow) w.integral = newIntegral;

    float out = ff + DRIVE_KP * err + DRIVE_KI * w.integral;
    return constrain(out, -DRIVE_MAX_DUTY, DRIVE_MAX_DUTY);
}

// One 100 Hz servo tick: measure, control, write.
static void driveTick(float dt) {
    driveUpdateRPM(driveL, readTicks(leftUnit),  dt);
    driveUpdateRPM(driveR, readTicks(rightUnit), dt);

    enableDriver();   // any safety stop pulled STBY low; PWM into standby does nothing
    setLeftMotor(driveControlLaw(driveL,  dt));
    setRightMotor(driveControlLaw(driveR, dt));
}

// =============================================================================
// CIRCLE PATH
// =============================================================================
//
// Generated once at boot so the geometry can be changed by editing
// CIRCLE_RADIUS_M / PATH_POINT_SPACING_M with nothing to recompute by hand.
//
// Points are laid out in the direction of travel: index increasing always
// means "further along the path", which is what lets the index search be
// forward-only.

#define PATH_MAX_POINTS  256

static float pathX[PATH_MAX_POINTS];
static float pathY[PATH_MAX_POINTS];
static int   pathCount = 0;

static int   pathIndex = 0;   // waypoint we are currently tracking from
static int   lapCount  = 0;

static void buildCircle() {
    pathCount = 0;

    float circumference = 2.0f * PI * CIRCLE_RADIUS_M;
    int n = (int)(circumference / PATH_POINT_SPACING_M);
    if (n < 12) n = 12;
    if (n > PATH_MAX_POINTS) n = PATH_MAX_POINTS;

    float dir = CIRCLE_CCW ? 1.0f : -1.0f;

    for (int i = 0; i < n; i++) {
        float t = dir * (2.0f * PI * i) / n;
        pathX[pathCount] = CIRCLE_CENTER_X + CIRCLE_RADIUS_M * cosf(t);
        pathY[pathCount] = CIRCLE_CENTER_Y + CIRCLE_RADIUS_M * sinf(t);
        pathCount++;
    }

    Serial.printf("[path] circle: %d points, r=%.2f m, centre (%.2f, %.2f), %s\n",
                  pathCount, CIRCLE_RADIUS_M, CIRCLE_CENTER_X, CIRCLE_CENTER_Y,
                  CIRCLE_CCW ? "CCW" : "CW");
    Serial.printf("[path] spans x [%.2f, %.2f]  y [%.2f, %.2f]\n",
                  CIRCLE_CENTER_X - CIRCLE_RADIUS_M, CIRCLE_CENTER_X + CIRCLE_RADIUS_M,
                  CIRCLE_CENTER_Y - CIRCLE_RADIUS_M, CIRCLE_CENTER_Y + CIRCLE_RADIUS_M);
}

// Resample a closed polyline of corner vertices into evenly spaced path
// points. The last vertex joins back to the first.
//
// Even spacing matters: findGoalPoint walks forward accumulating distance
// between points, so unevenly spaced points make the effective lookahead
// vary with position on the path.
static void buildFromWaypoints(const float *vx, const float *vy, int nv) {
    pathCount = 0;

    for (int i = 0; i < nv; i++) {
        int j = (i + 1) % nv;                    // wraps, closing the loop
        float dx = vx[j] - vx[i];
        float dy = vy[j] - vy[i];
        float segLen = sqrtf(dx * dx + dy * dy);
        int steps = (int)(segLen / PATH_POINT_SPACING_M);
        if (steps < 1) steps = 1;

        // Emit the start of the segment but not its end — the end is the next
        // segment's start, so this avoids a duplicated point at every vertex.
        for (int s = 0; s < steps && pathCount < PATH_MAX_POINTS; s++) {
            float t = (float)s / (float)steps;
            pathX[pathCount] = vx[i] + dx * t;
            pathY[pathCount] = vy[i] + dy * t;
            pathCount++;
        }
    }
}

static void buildZigzagRect() {
    float vx[64], vy[64];
    int nv = 0;

    // Zigzag leg, left to right, alternating between the two y levels.
    int zigs = (int)roundf((ZIG_X_MAX - ZIG_X_MIN) / ZIG_PITCH_M);
    if (zigs < 1) zigs = 1;

    for (int i = 0; i <= zigs && nv < 60; i++) {
        vx[nv] = ZIG_X_MIN + ZIG_PITCH_M * i;
        vy[nv] = (i % 2 == 0) ? ZIG_Y_LO : ZIG_Y_HI;
        nv++;
    }

    // Down the right end, straight back along the bottom, and the left end
    // closes automatically back to the first vertex.
    vx[nv] = ZIG_X_MAX;  vy[nv] = RETURN_Y;  nv++;
    vx[nv] = ZIG_X_MIN;  vy[nv] = RETURN_Y;  nv++;

    buildFromWaypoints(vx, vy, nv);

    float turnDeg = 2.0f * atanf(fabsf(ZIG_Y_HI - ZIG_Y_LO) / ZIG_PITCH_M)
                    * 180.0f / PI;
    Serial.printf("[path] zigzag rect: %d vertices -> %d points\n", nv, pathCount);
    Serial.printf("[path] %d zigs, pitch %.2f m, swing %.2f m\n",
                  zigs, ZIG_PITCH_M, fabsf(ZIG_Y_HI - ZIG_Y_LO));
    Serial.printf("[path] zigzag vertex turn %.0f deg — a follower limited to\n",
                  turnDeg);
    Serial.printf("[path]   radius r needs %.2f*r of straight either side\n",
                  tanf(turnDeg * PI / 360.0f));
    Serial.printf("[path] spans x [%.2f, %.2f]  y [%.2f, %.2f]\n",
                  ZIG_X_MIN, ZIG_X_MAX, RETURN_Y, ZIG_Y_HI);
}

static void buildPath() {
    if (PATH_MODE == PATH_ZIGZAG_RECT) buildZigzagRect();
    else                               buildCircle();
}

static float distToPoint(int idx) {
    float dx = pathX[idx] - currentX;
    float dy = pathY[idx] - currentY;
    return sqrtf(dx * dx + dy * dy);
}

// Full search — only on start or after losing the path.
static void acquirePath() {
    int   bestIdx  = 0;
    float bestDist = 1e9f;
    for (int i = 0; i < pathCount; i++) {
        float d = distToPoint(i);
        if (d < bestDist) { bestDist = d; bestIdx = i; }
    }
    pathIndex = bestIdx;
    Serial.printf("[path] acquired at point %d (%.3f, %.3f), %.3f m away\n",
                  pathIndex, pathX[pathIndex], pathY[pathIndex], bestDist);
}

// Advance pathIndex to the nearest point within a FORWARD window, so progress
// is monotonic and the robot cannot slide backwards along its own path.
static void advancePathIndex() {
    int   bestIdx  = pathIndex;
    float bestDist = distToPoint(pathIndex);

    for (int k = 1; k <= PATH_SEARCH_WINDOW; k++) {
        int idx = (pathIndex + k) % pathCount;
        float d = distToPoint(idx);
        if (d < bestDist) { bestDist = d; bestIdx = idx; }
    }

    if (bestIdx != pathIndex) {
        if (bestIdx < pathIndex) lapCount++;   // wrapped past index 0
        pathIndex = bestIdx;
    }
}

// Walk forward from pathIndex accumulating arc length until LOOKAHEAD_M.
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
// Transform the goal into the robot frame, take the bearing alpha to it, and
// the arc joining robot to goal has curvature
//
//     kappa = 2 * sin(alpha) / Ld
//
// Differential drive turns curvature into a wheel speed split directly:
// omega = v * kappa, then v_left/right = v -/+ omega * track / 2.

static float lastCurvature = 0.0f;
static float lastAlphaDeg  = 0.0f;
static bool  spinning      = false;

static void pursuitUpdate() {
    advancePathIndex();
    int goalIdx = findGoalPoint();

    float dx = pathX[goalIdx] - currentX;
    float dy = pathY[goalIdx] - currentY;

    // World -> robot frame (rotate by -heading).
    float c  = cosf(currentHeading);
    float sn = sinf(currentHeading);
    float xr =  c * dx + sn * dy;
    float yr = -sn * dx + c * dy;

    float ld = sqrtf(dx * dx + dy * dy);
    if (ld < 0.02f) ld = 0.02f;                 // guard divide-by-zero

    float alpha = atan2f(yr, xr);               // bearing to goal, rad, CCW+
    lastAlphaDeg = alpha * 180.0f / PI;

    // Latching spin decision — no chatter on the boundary.
    if (!spinning && fabsf(lastAlphaDeg) > SPIN_ENTER_DEG) spinning = true;
    if ( spinning && fabsf(lastAlphaDeg) < SPIN_EXIT_DEG)  spinning = false;

    if (spinning) {
        float spinRpm = TURN_IN_PLACE_RPM * ((alpha > 0) ? 1.0f : -1.0f);
        if (INVERT_SPIN) spinRpm = -spinRpm;
        driveL.setpoint = -spinRpm;
        driveR.setpoint =  spinRpm;
        lastCurvature = 0.0f;
        return;
    }

    float kappa = 2.0f * sinf(alpha) / ld;
    kappa = constrain(kappa, -MAX_CURVATURE, MAX_CURVATURE);
    lastCurvature = kappa;

    float v     = CRUISE_MPS;
    float omega = v * kappa;                                  // rad/s
    float vL    = v - omega * (TRACK_WIDTH_M / 2.0f);
    float vR    = v + omega * (TRACK_WIDTH_M / 2.0f);

    driveL.setpoint = vL * MPS_TO_RPM;
    driveR.setpoint = vR * MPS_TO_RPM;
}

static void pursuitStop() {
    driveResetWheel(driveL);
    driveResetWheel(driveR);
    spinning = false;
    allStop();
}

// =============================================================================
// WIFI
// =============================================================================

static void connectWiFi() {
    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);        // power save silently drops broadcast frames
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
    Serial.printf("  WiFi     : %s (%s)\n",
                  WiFi.status() == WL_CONNECTED ? "connected" : "DOWN",
                  WiFi.localIP().toString().c_str());
    Serial.printf("  Link     : %s (last pkt %lu ms ago)\n",
                  linkAlive() ? "alive" : "TIMED OUT", millis() - lastPacketMs);
    Serial.printf("  Pose     : %s (last %lu ms ago)\n",
                  poseFresh() ? "fresh" : "STALE", millis() - lastHeadingMs);
    Serial.printf("  Server   : %s\n", serverRun ? "RUN" : "STOP");
    Serial.printf("  Robot ID : %d\n", THIS_ROBOT_ID);
    Serial.println("---- SETTINGS ----");
    Serial.printf("  pursuit : lookahead %.3f m  cruise %.2f m/s  maxK %.1f\n",
                  LOOKAHEAD_M, CRUISE_MPS, MAX_CURVATURE);
    Serial.printf("  drive   : Kv=%.2f Ks=%.1f Kp=%.2f Ki=%.2f  cap %.0f\n",
                  DRIVE_KV, DRIVE_KS, DRIVE_KP, DRIVE_KI, DRIVE_MAX_DUTY);
    Serial.printf("  battery : %.2f V\n", readBatteryVoltage());
    Serial.println("------------------\n");
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
    Serial.begin(115200);
    delay(800);

    Serial.println("\n=== BUILD " __DATE__ " " __TIME__ " ===");
    Serial.println("=== CIRCLE LOOP (pure pursuit) ===");

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

    buildPath();
    connectWiFi();
    printStatus();
}

// =============================================================================
// LOOP
// =============================================================================
//
// Two rates, deliberately separated:
//   ~20 Hz (on a new camera pose)  -> pursuitUpdate() decides WHERE to go
//   100 Hz (timer)                 -> driveTick() makes the wheels do it
//
// Nothing blocks, so a STOP command lands within one servo tick.

void loop() {
    pollPackets();

    static unsigned long lastPrint   = 0;
    static unsigned long lastTickMs  = 0;
    static bool          haveAcquired = false;
    static bool          wasStopped   = true;

    // The path-lost check is only ARMED once we have actually reached the
    // path. Otherwise a cold start from the middle of the arena — further
    // from the circle than PATH_LOST_DIST_M — would trip "lost" on the first
    // step, stop, re-acquire, and repeat forever without ever driving.
    static bool          onPath       = false;

    bool doPrint = (millis() - lastPrint > PRINT_MS);

    // ── Safety gates ───────────────────────────────────────────────────────
    const char *stopReason = NULL;

    if (!linkAlive())      stopReason = "no packets from server";
    else if (!serverRun)   stopReason = "server says STOP";
    else if (!poseFresh()) stopReason = "no fresh pose (marker not detected?)";

    if (stopReason) {
        if (!wasStopped) { pursuitStop(); wasStopped = true; }
        // haveAcquired is deliberately NOT cleared on a brief gap — the
        // path-lost check below handles genuine drift. Clearing it on every
        // dropout meant never getting enough consecutive steps to overcome
        // stiction.
        if (!serverRun) haveAcquired = false;
        if (doPrint) {
            lastPrint = millis();
            Serial.printf("[hold] %s | pkt %lu ms, pose %lu ms, rssi %d\n",
                          stopReason,
                          millis() - lastPacketMs, millis() - lastHeadingMs,
                          WiFi.RSSI());
        }
        delay(5);
        return;
    }

    // ── Resuming after a stop ──────────────────────────────────────────────
    if (wasStopped) {
        driveResyncTicks();
        lastTickMs = millis();
        wasStopped = false;
    }

    if (!haveAcquired) {
        acquirePath();
        haveAcquired = true;
        onPath = false;          // must reach the circle before "lost" applies
        lapCount = 0;
    }

    float distToPath = distToPoint(pathIndex);

    // Arm the lost check the first time we get onto the path. Until then the
    // robot is simply driving in from wherever it was placed, which pure
    // pursuit handles on its own (the spin branch turns it to face the path
    // first if it was set down pointing the wrong way).
    if (!onPath && distToPath < PATH_LOST_DIST_M) {
        onPath = true;
        Serial.println("[path] on path — lost-detection armed");
    }

    // ── Lost the path? Re-acquire rather than lurching toward it. ──────────
    if (onPath && distToPath > PATH_LOST_DIST_M) {
        Serial.printf("[path] lost (%.3f m from point %d) — re-acquiring\n",
                      distToPath, pathIndex);
        pursuitStop();
        wasStopped = true;
        onPath = false;
        acquirePath();
        return;
    }

    // ── Geometry: only when the camera has told us something new ───────────
    if (newPose) {
        newPose = false;
        pursuitUpdate();
    }

    // ── Servo: fixed 100 Hz, measured dt ───────────────────────────────────
    unsigned long now = millis();
    if (now - lastTickMs >= DRIVE_LOOP_MS) {
        float dt = (now - lastTickMs) / 1000.0f;
        lastTickMs = now;
        driveTick(dt);
    }

    if (doPrint) {
        lastPrint = millis();
        Serial.printf("[run] pos(%+.3f,%+.3f) hdg%+7.1f  pt%3d/%d lap%d  "
                      "xtrk%.3f  a%+6.1f  k%+.2f  set L%+.0f R%+.0f  "
                      "act L%+.0f R%+.0f%s\n",
                      currentX, currentY, currentHeading * 180.0f / PI,
                      pathIndex, pathCount, lapCount,
                      distToPath, lastAlphaDeg, lastCurvature,
                      driveL.setpoint, driveR.setpoint,
                      driveL.rpm, driveR.rpm,
                      spinning ? "  [SPIN]" : "");
    }
}

// =============================================================================
// NOTES — things outside this sketch that affect how it behaves
// =============================================================================
//
// 1. tracker.py SCENE dimensions are wrong for this arena.
//    It currently has SCENE_WIDTH_M = 2.0 and SCENE_HEIGHT_M = 1.125, but the
//    camera views 3.60 m x 2.03 m. Every position it reports is therefore
//    scaled by ~0.56. The robot believes it is much closer to the origin than
//    it is, so a 0.60 m circle in firmware is driven as a ~1.08 m circle on
//    the floor, and every metre of real motion registers as 0.56 m of
//    progress. Pure pursuit reads that as persistent lag and over-steers.
//    Set SCENE_WIDTH_M = 3.6 and SCENE_HEIGHT_M = 2.03.
//
// 2. udp.py never forgets a robot.
//    update_positions() calls dict.update(), so an entry persists after the
//    marker stops being detected and the last known pose is rebroadcast
//    forever. Either drop entries not seen for ~0.5 s, or send the
//    (-9999, -9999) sentinel the project docs describe. This sketch defends
//    against it with POSE_TIMEOUT_MS, but the other robots will still be
//    consuming a ghost position for this one.
//
// 3. The 520-960 ms broadcast gaps are probably tracker.py starving the
//    broadcast thread. tracker.run() loops as fast as the CPU allows, calling
//    process_frame() and imshow() on the SAME frame repeatedly (the threaded
//    grabber only refreshes at 30 fps), so ArUco detection on 1920x1080 runs
//    far more often than needed and holds the GIL. Two cheap fixes: only
//    process when the frame is actually new, and imshow a downscaled copy.
//
// 4. udp.py hardcodes SERVER_IP 192.168.1.100 / BROADCAST_IP 192.168.1.255.
//    If the router hands out a different subnet, the bind fails or the
//    broadcast goes nowhere and every robot sits in [hold] forever.
//
// =============================================================================
