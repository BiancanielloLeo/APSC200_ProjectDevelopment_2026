// Leo Biancaniello + Claude
//
// oval_flock_pursuit.ino — distributed oval flocking, continuous drive model
// =============================================================================
//
// Built on figure_eight.ino's validated drive stack: per-wheel feedforward +
// PI velocity loop, encoder handling, packet parsing, and safety gates are
// UNCHANGED. What's replaced is the PATH: figure_eight.ino pursues a fixed,
// precomputed waypoint array. Here there is no fixed path — the pursuit goal
// is generated fresh every packet from three flocking rules, evaluated from
// live swarm state:
//
//   Every camera packet (~20 Hz):
//     1. Compute a steering vector = cohesion (toward a synchronized
//        "phantom leader" orbiting the arena on a fixed oval) + separation
//        (away from neighbours that are too close) + wall avoidance.
//     2. Project a goal point LOOKAHEAD_M ahead of the robot in that
//        direction — this plays exactly the role findGoalPoint() played in
//        the static-path version.
//     3. Transform that goal into the robot frame, curvature kappa = 2*y_r/Ld^2.
//     4. Split into left/right wheel RPM setpoints and hand to the same PI
//        velocity loop.
//
// SYNCHRONIZATION: the phantom leader's angle is computed directly from the
// broadcast packet_number (ovalTheta = packet_number * dTheta) rather than
// from a locally incremented counter. Every robot sees the same
// packet_number, so all four robots agree on "where the leader is" exactly,
// even across dropped packets — there's no local state that can desync.
//
// IMPORTANT: set THIS_ROBOT_ID below to a unique value (1-4) before flashing
// each robot. All four robots run this exact same file otherwise.
//
// Requires ESP32 Arduino Core v3.0+ (uses ledcAttach()/ledcWrite(pin, duty)).
// The old ledcSetup()/ledcAttachPin()/channel-based ledcWrite() do not exist
// on this core and will fail to compile.
//
// TUNING (in rough order of what to try first):
//   LOOKAHEAD_M     too small -> weaves/oscillates;  too large -> cuts corners
//   FLOCK_CRUISE_MPS lower it if the robot can't keep up with a tight radius
//   OVAL_A/OVAL_B   size of the phantom leader's orbit; keep inside the arena
//   W_WALL > W_SEPARATION > W_COHESION -- walls should always win, then
//                    collision avoidance, then following the leader.
//
// =============================================================================

#include <WiFi.h>
#include <WiFiUdp.h>
#include <math.h>
#include <string.h>
#include "driver/pulse_cnt.h"

// ── Identity ───────────────────────────────────────────────────────────────
// Must match the ArUco marker AND be in the ID list typed into initiation.py.
#define THIS_ROBOT_ID   2

// ── Network (must match udp.py) ────────────────────────────────────────────
#define WIFI_SSID       "RobotWifi"
#define WIFI_PASSWORD   "12345678"
#define UDP_PORT        5005

// ── Pin Definitions (from figure_eight.ino / pcnt_test.ino) ────────────────
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
//  DRIVE TUNABLES (unchanged from figure_eight.ino's validated defaults)
// ═══════════════════════════════════════════════════════════════════════════

// Per-wheel feedforward: duty = Kv * |RPM| + Ks. Shared PI on top.
// These are the fitted defaults from the tuning rig; re-run 'ff' on that
// sketch if the floor changes and update these.
float DRIVE_KV = 1.0f;
float DRIVE_KS = 46.0f;
float DRIVE_KP = 2.5f;
float DRIVE_KI = 1.0f;
float DRIVE_MAX_DUTY = 178.0f;  // 70% of 255 — same cap as the tuning rig

#define DRIVE_LOOP_HZ   100
#define DRIVE_DT        (1.0f / DRIVE_LOOP_HZ)
#define DRIVE_LOOP_US   (1000000 / DRIVE_LOOP_HZ)
#define RPM_FILTER_ALPHA 0.3f
#define RPM_GLITCH_CEIL  200.0f   // motor no-load spec max; anything above is a bus glitch

// ── Pure pursuit ────────────────────────────────────────────────────────────
// THE tuning knob. Distance ahead along the steering direction to aim at.
// Too small: robot weaves and can oscillate. Too large: it cuts corners and
// under-reacts to nearby neighbours/walls. Start 0.15, adjust in 0.02 steps.
float LOOKAHEAD_M = 0.15f;

// Forward travel speed, m/s. Constant cruise, same philosophy as
// figure_eight.ino: the flocking rules steer, the PI loop holds speed.
float FLOCK_CRUISE_MPS = 0.15f;

// Curvature limit, 1/m. Caps how hard it will try to turn; without this a
// large lateral error commands a wheel reversal instead of an arc.
float MAX_CURVATURE = 6.0f;

// Used when the goal ends up behind the robot (e.g. after a wall bounce or
// being placed down facing the wrong way) — spin toward it instead of arcing.
float TURN_IN_PLACE_RPM = 35.0f;

// If the steering vector's magnitude is below this, hold still rather than
// jittering toward an effectively-arrived target. Rare in practice since the
// phantom leader is always moving.
float TARGET_DEADZONE_M = 0.05f;

// ── Direction calibration ───────────────────────────────────────────────────
// If turning goes the wrong way, flip INVERT_SPIN. If the drive goes
// backwards instead of forwards, flip DRIVE_FORWARD_SIGN — check wiring too.
bool  INVERT_SPIN        = false;
float DRIVE_FORWARD_SIGN = 1.0f;

// ═══════════════════════════════════════════════════════════════════════════
//  FLOCKING TUNABLES
// ═══════════════════════════════════════════════════════════════════════════

// Arena geometry (metres) -- from tracker.py SCENE_WIDTH_M / SCENE_HEIGHT_M
// (3.6m x 2.03m). World origin (0,0) is the arena centre. Half-extents below
// sit a little inside the true wall for margin.
float ARENA_HALF_X = 1.7f;
float ARENA_HALF_Y = 0.9f;
float WALL_MARGIN  = 0.35f;   // start repelling this far from the boundary

// Oval path (phantom leader).
float OVAL_A        = 1.3f;    // semi-axis along x, metres
float OVAL_B        = 0.6f;    // semi-axis along y, metres
float OVAL_PERIOD_S = 40.0f;   // seconds for one full lap -- tune on the bench
#define BROADCAST_HZ 20.0f     // must match udp.py's rate_hz
// Angular step per broadcast packet so one lap takes OVAL_PERIOD_S seconds.
#define OVAL_DTHETA  ((2.0f * PI) / (OVAL_PERIOD_S * BROADCAST_HZ))

// Steering rule weights -- tune these on the bench. Walls should always win,
// then collision avoidance, then following the leader.
float W_COHESION        = 1.0f;
float W_SEPARATION      = 1.4f;
float W_WALL            = 2.0f;
float NEIGHBOR_MIN_DIST = 0.35f;   // start repelling neighbours closer than this (m)

#define MAX_ROBOTS  8

// ── Geometry ───────────────────────────────────────────────────────────────
#define TICKS_PER_REV      1400.0f
#define WHEEL_DIAMETER_M   0.065f
#define TRACK_WIDTH_M      0.125f

// ── Freshness / safety ─────────────────────────────────────────────────────
#define PACKET_TIMEOUT_MS    1200
#define PURSUIT_PRINT_MS      500

// ── Battery (status/diagnostics only) ───────────────────────────────────────
#define R1              100000.0f
#define R2               47000.0f
#define DIVIDER_RATIO   (R2 / (R1 + R2))
#define ADC_RESOLUTION  4095.0f
#define ADC_VREF            3.3f

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

struct NeighbourState {
    uint8_t id;
    float   x, y;
    bool    valid;   // false if the tracker reported this robot as not-detected
};
static NeighbourState neighbours[MAX_ROBOTS];
static int             numNeighbours = 0;

// =============================================================================
// ENCODER SETUP  (unchanged from figure_eight.ino / pcnt_test.ino)
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
// MOTOR CONTROL  (unchanged from figure_eight.ino)
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
// BATTERY  (status/diagnostics only)
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
// Unchanged from figure_eight.ino, extended to also keep every OTHER robot's
// position (needed for the separation rule).

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

        numNeighbours = 0;

        for (uint8_t i = 0; i < nRobots; i++) {
            const uint8_t* e = &rxBuf[HDR_SIZE + i * ROBOT_ENTRY_SIZE];
            uint8_t id     = e[0];
            float   x      = beFloat(&e[1]);
            float   y      = beFloat(&e[5]);
            float   theta  = beFloat(&e[9]);

            if (id == THIS_ROBOT_ID) {
                currentX       = x;
                currentY       = y;
                currentHeading = theta;
                haveHeading    = true;
            } else if (numNeighbours < MAX_ROBOTS) {
                // tracker.py sends (-9999, -9999) when a marker isn't
                // detected; the entry is still present, just marked invalid.
                neighbours[numNeighbours].id    = id;
                neighbours[numNeighbours].x     = x;
                neighbours[numNeighbours].y     = y;
                neighbours[numNeighbours].valid = !(x < -9000.0f || y < -9000.0f);
                numNeighbours++;
            }
        }
    }
}

static bool linkAlive() {
    return (millis() - lastPacketMs) < PACKET_TIMEOUT_MS;
}

// =============================================================================
// DRIVE — feedforward + PI velocity loop  (unchanged from figure_eight.ino)
// =============================================================================
// Flocking only changes what the setpoints ARE — the loop that achieves them
// is exactly as validated there.

struct DriveWheel {
    float Kv, Ks;
    float setpoint;
    float rpm;
    float integral;
    int16_t lastTicks;
};

static DriveWheel driveL, driveR;

static void  driveResetWheel(DriveWheel &w);
static void  driveUpdateRPM(DriveWheel &w, int16_t rawTicks);
static float driveControlLaw(DriveWheel &w);

static void driveResetWheel(DriveWheel &w) {
    w.setpoint = 0.0f;
    w.rpm = 0.0f;
    w.integral = 0.0f;
    w.lastTicks = 0;
}

static void driveUpdateRPM(DriveWheel &w, int16_t rawTicks) {
    int16_t delta = rawTicks - w.lastTicks;
    w.lastTicks = rawTicks;
    float rawRpm = ((float)delta / TICKS_PER_REV) * (60.0f / DRIVE_DT);

    if (fabsf(rawRpm) > RPM_GLITCH_CEIL) return;
    w.rpm += RPM_FILTER_ALPHA * (rawRpm - w.rpm);
}

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
// FLOCKING STEERING
// =============================================================================
// Recomputed fresh from each packet -- no state carried between calls except
// what's already in currentX/Y/Heading and neighbours[].

static float wallForce(float pos, float halfExtent) {
    float distToPosWall = halfExtent - pos;
    float distToNegWall = pos + halfExtent;

    if (distToPosWall < WALL_MARGIN) {
        float strength = (WALL_MARGIN - distToPosWall) / WALL_MARGIN;
        return -W_WALL * strength;
    }
    if (distToNegWall < WALL_MARGIN) {
        float strength = (WALL_MARGIN - distToNegWall) / WALL_MARGIN;
        return W_WALL * strength;
    }
    return 0.0f;
}

static void computeSteeringVector(float &tx, float &ty) {
    tx = 0.0f;
    ty = 0.0f;

    // --- Cohesion: pull toward the phantom leader on the oval path ---
    // Derived directly from the shared, monotonic packet_number rather than
    // a locally incremented counter, so every robot agrees on the leader's
    // position exactly, even across dropped packets.
    float ovalTheta = fmodf((float)lastPacketNum * OVAL_DTHETA, 2.0f * PI);
    float phantomX = OVAL_A * cosf(ovalTheta);
    float phantomY = OVAL_B * sinf(ovalTheta);
    tx += W_COHESION * (phantomX - currentX);
    ty += W_COHESION * (phantomY - currentY);

    // --- Separation: push away from neighbours that are too close ---
    for (int i = 0; i < numNeighbours; i++) {
        if (!neighbours[i].valid) continue;
        float dx = currentX - neighbours[i].x;
        float dy = currentY - neighbours[i].y;
        float dist = sqrtf(dx * dx + dy * dy);
        if (dist > 0.001f && dist < NEIGHBOR_MIN_DIST) {
            float strength = (NEIGHBOR_MIN_DIST - dist) / NEIGHBOR_MIN_DIST;
            tx += W_SEPARATION * strength * (dx / dist);
            ty += W_SEPARATION * strength * (dy / dist);
        }
    }

    // --- Wall avoidance: push back in from the boundary ---
    tx += wallForce(currentX, ARENA_HALF_X);
    ty += wallForce(currentY, ARENA_HALF_Y);
}

// =============================================================================
// PURE PURSUIT  (curvature math unchanged from figure_eight.ino; the goal
// point is generated live from the flocking steering vector instead of a
// precomputed path array)
// =============================================================================

static float lastCurvature = 0.0f;

static void flockPursuitStep() {
    float tx, ty;
    computeSteeringVector(tx, ty);

    float mag = sqrtf(tx * tx + ty * ty);
    if (mag < TARGET_DEADZONE_M) {
        driveL.setpoint = 0.0f;
        driveR.setpoint = 0.0f;
    } else {
        // Project a pure-pursuit goal LOOKAHEAD_M ahead of the robot in the
        // steering direction -- plays the role findGoalPoint() played for a
        // static path, just generated fresh each packet instead of by
        // walking a precomputed array.
        float goalX = currentX + (tx / mag) * LOOKAHEAD_M;
        float goalY = currentY + (ty / mag) * LOOKAHEAD_M;

        float dx = goalX - currentX;
        float dy = goalY - currentY;

        // World -> robot frame. currentHeading is CCW-positive radians.
        float c  = cosf(currentHeading);
        float sn = sinf(currentHeading);
        float xr =  c * dx + sn * dy;
        float yr = -sn * dx + c * dy;

        float ld = sqrtf(dx * dx + dy * dy);
        if (ld < 0.02f) ld = 0.02f;   // guard divide-by-zero

        float kappa = 2.0f * yr / (ld * ld);
        kappa = constrain(kappa, -MAX_CURVATURE, MAX_CURVATURE);
        lastCurvature = kappa;

        if (xr < 0.0f && fabsf(yr) > 0.05f) {
            // Goal is behind us -- spin toward it rather than driving a
            // degenerate arc.
            float spinRpm = TURN_IN_PLACE_RPM * ((yr > 0) ? 1.0f : -1.0f);
            if (INVERT_SPIN) spinRpm = -spinRpm;
            driveL.setpoint = -spinRpm;
            driveR.setpoint =  spinRpm;
        } else {
            float v = FLOCK_CRUISE_MPS * DRIVE_FORWARD_SIGN;
            float omega = v * kappa;
            float vL = v - omega * (TRACK_WIDTH_M / 2.0f);
            float vR = v + omega * (TRACK_WIDTH_M / 2.0f);

            float toRpm = 60.0f / (PI * WHEEL_DIAMETER_M);
            driveL.setpoint = vL * toRpm;
            driveR.setpoint = vR * toRpm;
        }
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
    Serial.printf("  oval: a=%.2f b=%.2f period=%.1fs\n",
                  OVAL_A, OVAL_B, OVAL_PERIOD_S);
    Serial.printf("  weights: cohesion=%.2f separation=%.2f wall=%.2f\n",
                  W_COHESION, W_SEPARATION, W_WALL);
    Serial.printf("  pursuit: lookahead %.3f m  cruise %.2f m/s  maxK %.1f\n",
                  LOOKAHEAD_M, FLOCK_CRUISE_MPS, MAX_CURVATURE);
    Serial.printf("  drive: Kv=%.3f Ks=%.1f Kp=%.2f Ki=%.2f\n",
                  DRIVE_KV, DRIVE_KS, DRIVE_KP, DRIVE_KI);
    Serial.printf("  spin: %s   batt: %.2f V\n",
                  INVERT_SPIN ? "INVERTED" : "normal", readBatteryVoltage());
    Serial.println("------------------\n");
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

    Serial.println("\n=== OVAL FLOCK (pure pursuit, continuous drive) ===");
    Serial.printf("Robot ID %d, cruise %.2f m/s, lookahead %.3f m\n",
                  THIS_ROBOT_ID, FLOCK_CRUISE_MPS, LOOKAHEAD_M);

    connectWiFi();
    printStatus();
}

// =============================================================================
// LOOP
// =============================================================================
//
// Non-blocking: each pass is one control step -- read the latest pose,
// compute one pursuit update, write the motors, return. A STOP or a lost
// link lands within one cycle.

void loop() {
    pollPackets();

    static unsigned long lastPrint   = 0;
    static unsigned long lastControl = 0;
    bool doPrint = (millis() - lastPrint > PURSUIT_PRINT_MS);

    // ── Safety gates ───────────────────────────────────────────────────────
    if (!linkAlive()) {
        pursuitStop();
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

    // ── One control step ───────────────────────────────────────────────────
    // Paced to the drive loop rate the PI gains were tuned at.
    if (millis() - lastControl < (DRIVE_LOOP_US / 1000)) {
        delay(1);
        return;
    }
    lastControl = millis();

    flockPursuitStep();

    if (doPrint) {
        lastPrint = millis();
        Serial.printf("[run] pos(%.3f,%.3f) hdg%6.1f  neighbours%d  "
                      "k%+.2f  rpm L%+.0f R%+.0f\n",
                      currentX, currentY, currentHeading * 180.0f / PI,
                      numNeighbours, lastCurvature,
                      driveL.setpoint, driveR.setpoint);
    }
}
