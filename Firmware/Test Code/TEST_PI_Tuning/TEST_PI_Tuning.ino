// Leo Biancaniello + Claude
// TEST: Feedforward + PI Velocity Control (Gain Tuning Rig)
//
// PURPOSE:
//   Two fully independent velocity loops, one per wheel, run at 100 Hz.
//   No cross-coupling terms of any kind, so the same controller drives
//   straight (equal setpoints) and arcs (different setpoints).
//
//   Kp and Ki are SHARED across both wheels for tuning simplicity — the two
//   loops still run separately, they just happen to use the same P and I gains.
//   Kv and Ks are PER-WHEEL: they capture each motor's individual friction and
//   duty/RPM slope, which is the entire reason feedforward makes the robot go
//   straight. Those are fitted automatically by the 'ff' sweep.
//
//   Per-wheel control law:
//       ff   = Kv * |RPM_des| + Ks          (only when |RPM_des| > 0)
//       err  = RPM_des - RPM_meas
//       I   += err * dt                      (clamped, anti-windup)
//       duty = ff + Kp*err + Ki*I            (clamped to +/- MAX_DUTY)
//
// LOOP RATE: 100 Hz (dt = 0.01 s)   TICKS/REV: 1400 (7 PPR x 50 gear x 4 edges)
//
// SERIAL COMMANDS (115200 baud, newline-terminated):
//   v <RPM>        both wheels to this target RPM (negative = reverse)
//   x              stop
//   set <Kp> <Ki>  set shared Kp and Ki
//   ff             run open-loop duty sweep, auto-fit + apply per-wheel Kv/Ks
//   g              print current gains
//   p              toggle CSV telemetry
//   step <RPM>     0 -> RPM step for 3 s (telemetry auto-on)
//   arc <L> <R>    independent left/right setpoints (arc sanity check)
//   h              help
//
// CSV TELEMETRY: t_ms,L_set,L_rpm,L_duty,L_err,R_set,R_rpm,R_duty,R_err
//
// TUNING PROCEDURE:
//   1. Robot on the FLOOR (the surface you'll actually drive on), space ahead.
//   2. 'ff' — sweeps open-loop duty, fits duty = Kv*RPM + Ks per wheel by
//      least squares, and applies the results automatically. Off-the-ground is
//      only useful as a left-vs-right health check, not for these gains.
//   3. 'set 2.5 0' then 'step 70'. Raise Kp until fast with slight overshoot,
//      back off ~20%.
//   4. 'set <Kp> 1' and raise Ki until steady-state error dies in ~0.5 s
//      without sustained oscillation.
//   5. 'arc 80 40' — confirm each wheel independently holds its own setpoint.
//
// SAFETY: duty hard-capped at 70% of 255; integrator clamped and frozen on
// saturation so it can't wind past the cap.

#include "driver/pulse_cnt.h"

// ── Pins ───────────────────────────────────────────────────────────────────────
#define PWMA_PIN     4
#define AIN2_PIN     5
#define AIN1_PIN     6
#define STBY_PIN     7
#define BIN1_PIN     8
#define BIN2_PIN     9
#define PWMB_PIN     10

#define LEFT_ENC_A   1
#define LEFT_ENC_B   2
#define RIGHT_ENC_A  21
#define RIGHT_ENC_B  47

// ── Config ─────────────────────────────────────────────────────────────────────
#define PWM_FREQ         20000
#define PWM_RESOLUTION   8
#define MAX_DUTY         178.0f   // 70% of 255
#define TICKS_PER_REV    1400.0f
#define LOOP_HZ          100
#define LOOP_US          (1000000 / LOOP_HZ)
#define DT               (1.0f / LOOP_HZ)

#define PCNT_HIGH_LIMIT  32767
#define PCNT_LOW_LIMIT  -32768
#define GLITCH_FILTER_NS 12500

// RPM low-pass. One tick/loop at 100 Hz is ~4.3 RPM of quantization; alpha=0.3
// gives a ~5 Hz corner, above the motor's mechanical bandwidth but kills jitter.
#define RPM_FILTER_ALPHA 0.3f

// Feedforward sweep parameters.
//   Pyramid: duty ramps START -> MAX in STEP increments, then back down.
//   Each duty level is visited twice (up pass + down pass) and the two
//   steady-state RPM readings are averaged, which cancels most hysteresis
//   (backlash, thermal drift) between the ascending and descending legs.
//   Low-duty points where the wheel never spins are rejected before the fit.
#define FF_DUTY_START    40
#define FF_DUTY_MAX      175      // top of the pyramid (<= MAX_DUTY)
#define FF_DUTY_STEP     5
#define FF_SETTLE_MS     1500     // wait for transient to die out
#define FF_AVG_MS        1000     // then average steady state
#define FF_MIN_RPM       3.0f     // reject points below this (wheel not spinning)

// Number of distinct duty levels in one leg of the pyramid.
#define FF_NUM_LEVELS    (((FF_DUTY_MAX - FF_DUTY_START) / FF_DUTY_STEP) + 1)

// ── Shared gains ───────────────────────────────────────────────────────────────
float sharedKp = 2.5f;
float sharedKi = 1.0f;

// ── Per-wheel state ────────────────────────────────────────────────────────────
struct Wheel {
  float Kv, Ks;        // per-wheel feedforward (fitted by 'ff')
  float setpoint;      // desired RPM (signed)
  float rpm;           // filtered measured RPM (signed)
  float integral;
  float duty;          // last commanded duty (signed, for logging)
  float err;

  pcnt_unit_handle_t unit;
  int16_t lastTicks;
  int pwmPin, in1Pin, in2Pin;
};

Wheel L, R;

// Forward declaration: Arduino's auto-prototyping can mis-handle the Wheel&
// parameter when the definition sits below its caller.
static void fitAndApply(const float *x, const float *y, int n, Wheel &w, const char *name);

bool telemetryOn = false;
uint32_t lastLoopUs = 0;
uint32_t startMs = 0;
bool stepActive = false;
uint32_t stepEndMs = 0;

// ── Encoder setup (from pcnt_test.ino) ─────────────────────────────────────────
static pcnt_unit_handle_t setupEncoder(int pinA, int pinB, bool invertDirection) {
  pcnt_unit_config_t unitCfg = {
    .low_limit = PCNT_LOW_LIMIT,
    .high_limit = PCNT_HIGH_LIMIT,
    .intr_priority = 0,
    .flags = { .accum_count = false }
  };
  pcnt_unit_handle_t unit = NULL;
  ESP_ERROR_CHECK(pcnt_new_unit(&unitCfg, &unit));

  pcnt_glitch_filter_config_t filterCfg = { .max_glitch_ns = GLITCH_FILTER_NS };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filterCfg));

  pcnt_channel_edge_action_t onRise = invertDirection
      ? PCNT_CHANNEL_EDGE_ACTION_DECREASE : PCNT_CHANNEL_EDGE_ACTION_INCREASE;
  pcnt_channel_edge_action_t onFall = invertDirection
      ? PCNT_CHANNEL_EDGE_ACTION_INCREASE : PCNT_CHANNEL_EDGE_ACTION_DECREASE;

  pcnt_chan_config_t ch0Cfg = { .edge_gpio_num = pinA, .level_gpio_num = pinB, .flags = {} };
  pcnt_channel_handle_t ch0 = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(unit, &ch0Cfg, &ch0));
  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch0, onRise, onFall));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch0,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

  pcnt_chan_config_t ch1Cfg = { .edge_gpio_num = pinB, .level_gpio_num = pinA, .flags = {} };
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

// ── Low-level output ───────────────────────────────────────────────────────────
static void applyDuty(Wheel &w, float signedDuty) {
  if (signedDuty > MAX_DUTY)  signedDuty =  MAX_DUTY;
  if (signedDuty < -MAX_DUTY) signedDuty = -MAX_DUTY;
  if (signedDuty >= 0) { digitalWrite(w.in1Pin, HIGH); digitalWrite(w.in2Pin, LOW); }
  else                 { digitalWrite(w.in1Pin, LOW);  digitalWrite(w.in2Pin, HIGH); }
  ledcWrite(w.pwmPin, (uint32_t)(fabsf(signedDuty) + 0.5f));
}

// ── Velocity read (int16 delta handles +/-32767 wrap) ──────────────────────────
// A dropped/extra encoder edge or a bus glitch can produce a single delta far
// larger than any physical motion (e.g. -309 ticks in one 10 ms sample = -1324
// RPM). The motor's no-load spec max is 200 RPM, so any sample implying more
// than RPM_GLITCH_CEIL is non-physical: discard it, hold the last filtered
// value, and DON'T let the bad count corrupt lastTicks — resync lastTicks to the
// current reading so the next delta is measured cleanly from here.
#define RPM_GLITCH_CEIL 200.0f
static void updateRPM(Wheel &w) {
  int raw = 0;
  ESP_ERROR_CHECK(pcnt_unit_get_count(w.unit, &raw));
  int16_t current = (int16_t)raw;
  int16_t delta = current - w.lastTicks;
  w.lastTicks = current;
  float rawRpm = ((float)delta / TICKS_PER_REV) * (60.0f / DT);

  if (fabsf(rawRpm) > RPM_GLITCH_CEIL) {
    // Glitch: ignore this sample. lastTicks already resynced above, so the
    // next call measures a fresh delta rather than compounding the error.
    return;
  }
  w.rpm += RPM_FILTER_ALPHA * (rawRpm - w.rpm);
}

// ── Per-wheel control law (shared Kp/Ki, per-wheel Kv/Ks) ──────────────────────
static void controlWheel(Wheel &w) {
  updateRPM(w);

  if (fabsf(w.setpoint) < 0.5f) {
    w.integral = 0.0f; w.err = 0.0f; w.duty = 0.0f;
    digitalWrite(w.in1Pin, LOW); digitalWrite(w.in2Pin, LOW);
    ledcWrite(w.pwmPin, 0);
    return;
  }

  w.err = w.setpoint - w.rpm;

  float dir = (w.setpoint >= 0) ? 1.0f : -1.0f;
  float ff  = dir * (w.Kv * fabsf(w.setpoint) + w.Ks);

  float newIntegral = w.integral + w.err * DT;

  // Clamp so the I term alone can't exceed the cap (and guard Ki==0).
  if (sharedKi > 0.0001f) {
    float iLimit = MAX_DUTY / sharedKi;
    if (newIntegral >  iLimit) newIntegral =  iLimit;
    if (newIntegral < -iLimit) newIntegral = -iLimit;
  } else {
    newIntegral = 0.0f;
  }

  float unclamped = ff + sharedKp * w.err + sharedKi * newIntegral;

  // Conditional-integration anti-windup.
  bool satHigh = (unclamped >  MAX_DUTY) && (w.err > 0);
  bool satLow  = (unclamped < -MAX_DUTY) && (w.err < 0);
  if (!satHigh && !satLow) w.integral = newIntegral;

  float out = ff + sharedKp * w.err + sharedKi * w.integral;
  if (out >  MAX_DUTY) out =  MAX_DUTY;
  if (out < -MAX_DUTY) out = -MAX_DUTY;

  w.duty = out;
  applyDuty(w, out);
}

// ── Setup ──────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(AIN1_PIN, OUTPUT); pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT); pinMode(BIN2_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  ledcAttach(PWMA_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB_PIN, PWM_FREQ, PWM_RESOLUTION);
  digitalWrite(STBY_PIN, HIGH);

  L.pwmPin = PWMA_PIN; L.in1Pin = AIN1_PIN; L.in2Pin = AIN2_PIN;
  L.unit = setupEncoder(LEFT_ENC_A, LEFT_ENC_B, true);
  R.pwmPin = PWMB_PIN; R.in1Pin = BIN1_PIN; R.in2Pin = BIN2_PIN;
  R.unit = setupEncoder(RIGHT_ENC_A, RIGHT_ENC_B, false);

  // Feedforward starting points from the design doc; overwritten by 'ff'.
  L.Kv = R.Kv = 1.0f;
  L.Ks = R.Ks = 46.0f;

  ESP_ERROR_CHECK(pcnt_unit_clear_count(L.unit));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(R.unit));
  L.lastTicks = R.lastTicks = 0;
  L.rpm = R.rpm = 0.0f;
  L.integral = R.integral = 0.0f;
  L.setpoint = R.setpoint = 0.0f;

  startMs = millis();
  lastLoopUs = micros();
  printHelp();
}

// ── 100 Hz loop ────────────────────────────────────────────────────────────────
void loop() {
  handleSerial();

  if (stepActive && millis() >= stepEndMs) {
    stopAll(); stepActive = false; telemetryOn = false;
    Serial.println("# step complete");
  }

  uint32_t now = micros();
  if ((uint32_t)(now - lastLoopUs) >= LOOP_US) {
    lastLoopUs += LOOP_US;
    controlWheel(L);
    controlWheel(R);
    if (telemetryOn) printTelemetry();
  }
}

// ── Telemetry ──────────────────────────────────────────────────────────────────
void printTelemetry() {
  Serial.printf("%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n",
    millis() - startMs,
    L.setpoint, L.rpm, L.duty, L.err,
    R.setpoint, R.rpm, R.duty, R.err);
}
void printCsvHeader() {
  Serial.println("t_ms,L_set,L_rpm,L_duty,L_err,R_set,R_rpm,R_duty,R_err");
}

// ── Commands ───────────────────────────────────────────────────────────────────
void stopAll() {
  L.setpoint = R.setpoint = 0.0f;
  L.integral = R.integral = 0.0f;
  L.duty = R.duty = 0.0f;
  applyDuty(L, 0); applyDuty(R, 0);
  digitalWrite(AIN1_PIN, LOW); digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW); digitalWrite(BIN2_PIN, LOW);
}

void printGains() {
  Serial.printf("# shared:  Kp=%.3f  Ki=%.3f\n", sharedKp, sharedKi);
  Serial.printf("# LEFT  ff: Kv=%.4f  Ks=%.2f\n", L.Kv, L.Ks);
  Serial.printf("# RIGHT ff: Kv=%.4f  Ks=%.2f\n", R.Kv, R.Ks);
}

void printHelp() {
  Serial.println("\n=== FF + PI Velocity Tuning Rig ===");
  Serial.println("  v <RPM>        both wheels to target RPM");
  Serial.println("  x              stop");
  Serial.println("  set <Kp> <Ki>  set shared Kp and Ki");
  Serial.println("  ff             sweep + auto-fit per-wheel Kv/Ks");
  Serial.println("  g              print gains");
  Serial.println("  p              toggle CSV telemetry");
  Serial.println("  step <RPM>     3 s step response");
  Serial.println("  arc <L> <R>    independent setpoints (arc check)");
  Serial.println("  h              help");
  printGains();
  Serial.println();
}

void handleSerial() {
  static char buf[64];
  static uint8_t idx = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (idx == 0) continue;
      buf[idx] = '\0';
      parseCommand(buf);
      idx = 0;
    } else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
}

void parseCommand(char *line) {
  char cmd[12];
  float a = 0, b = 0;
  int n = sscanf(line, "%11s %f %f", cmd, &a, &b);
  if (n < 1) return;

  if (!strcmp(cmd, "v") && n >= 2) {
    L.setpoint = R.setpoint = a;
    Serial.printf("# both = %.1f RPM\n", a);
  }
  else if (!strcmp(cmd, "x")) { stopAll(); Serial.println("# stopped"); }
  else if (!strcmp(cmd, "set") && n == 3) {
    sharedKp = a; sharedKi = b;
    L.integral = R.integral = 0.0f;
    printGains();
  }
  else if (!strcmp(cmd, "arc") && n == 3) {
    L.setpoint = a; R.setpoint = b;
    Serial.printf("# arc L=%.1f R=%.1f\n", a, b);
  }
  else if (!strcmp(cmd, "g")) { printGains(); }
  else if (!strcmp(cmd, "h")) { printHelp(); }
  else if (!strcmp(cmd, "p")) {
    telemetryOn = !telemetryOn;
    if (telemetryOn) printCsvHeader(); else Serial.println("# telemetry off");
  }
  else if (!strcmp(cmd, "step") && n >= 2) {
    stopAll(); delay(200);
    L.rpm = R.rpm = 0.0f;
    startMs = millis();
    telemetryOn = true; printCsvHeader();
    L.setpoint = R.setpoint = a;
    stepActive = true; stepEndMs = millis() + 3000;
  }
  else if (!strcmp(cmd, "ff")) { runFeedforwardSweep(); }
  else { Serial.println("# unknown command — press 'h'"); }
}

// Hold one open-loop duty, let the transient settle, then average steady-state
// RPM for both wheels. Fills *outL / *outR.
static void measurePoint(int duty, float *outL, float *outR) {
  applyDuty(L, duty); applyDuty(R, duty);

  uint32_t t0 = millis();
  while (millis() - t0 < FF_SETTLE_MS) { updateRPM(L); updateRPM(R); delay(10); }

  float sumL = 0, sumR = 0; int samples = 0;
  t0 = millis();
  while (millis() - t0 < FF_AVG_MS) {
    updateRPM(L); updateRPM(R);
    sumL += L.rpm; sumR += R.rpm; samples++;
    delay(10);
  }
  *outL = sumL / samples;
  *outR = sumR / samples;
}

// ── Feedforward sweep + on-device least-squares fit ────────────────────────────
// Pyramid sweep: ramp duty up FF_DUTY_START -> FF_DUTY_MAX, then back down, in
// FF_DUTY_STEP increments. Each duty level is measured on both legs and the two
// steady-state RPM readings are averaged, cancelling backlash/thermal hysteresis.
// Fits duty = Kv*RPM + Ks per wheel by least squares (RPM independent, duty
// dependent — the direction we invert at runtime) and applies the result.
// Robot must be on the floor / free to move.
void runFeedforwardSweep() {
  Serial.println("# pyramid feedforward sweep — robot free to move!");
  Serial.printf("# %d levels x 2 passes, ~%.1fs each\n",
                FF_NUM_LEVELS, (FF_SETTLE_MS + FF_AVG_MS) / 1000.0f);

  // Per-level accumulated RPM and pass count, indexed by level.
  float dutyLvl[FF_NUM_LEVELS];
  float lSum[FF_NUM_LEVELS];
  float rSum[FF_NUM_LEVELS];
  int   passes[FF_NUM_LEVELS];
  for (int i = 0; i < FF_NUM_LEVELS; i++) {
    dutyLvl[i] = FF_DUTY_START + i * FF_DUTY_STEP;
    lSum[i] = rSum[i] = 0.0f;
    passes[i] = 0;
  }

  stopAll();
  delay(300);

  Serial.println("duty,pass,L_rpm,R_rpm");

  // Up leg: indices 0 .. N-1. Down leg: indices N-1 .. 0.
  for (int leg = 0; leg < 2; leg++) {
    bool goingUp = (leg == 0);
    for (int k = 0; k < FF_NUM_LEVELS; k++) {
      int i = goingUp ? k : (FF_NUM_LEVELS - 1 - k);
      float aL, aR;
      measurePoint((int)dutyLvl[i], &aL, &aR);
      lSum[i] += aL; rSum[i] += aR; passes[i]++;
      Serial.printf("%.0f,%s,%.1f,%.1f\n",
                    dutyLvl[i], goingUp ? "up" : "dn", aL, aR);
    }
  }

  stopAll();

  // Build the fit inputs from per-level averages, rejecting low/non-spinning
  // points on EITHER wheel (they flatten the slope and corrupt the intercept).
  float dutyFit[FF_NUM_LEVELS];
  float lFit[FF_NUM_LEVELS];
  float rFit[FF_NUM_LEVELS];
  int nL = 0, nR = 0;
  float dutyFitR[FF_NUM_LEVELS];

  for (int i = 0; i < FF_NUM_LEVELS; i++) {
    float avgL = lSum[i] / passes[i];
    float avgR = rSum[i] / passes[i];
    if (avgL >= FF_MIN_RPM) { dutyFit[nL]  = dutyLvl[i]; lFit[nL] = avgL; nL++; }
    if (avgR >= FF_MIN_RPM) { dutyFitR[nR] = dutyLvl[i]; rFit[nR] = avgR; nR++; }
  }

  Serial.printf("# usable points: L=%d R=%d (rejected below %.0f RPM)\n",
                nL, nR, FF_MIN_RPM);

  if (nL < 2 || nR < 2) { Serial.println("# not enough spinning points to fit"); return; }

  fitAndApply(lFit, dutyFit,  nL, L, "LEFT");
  fitAndApply(rFit, dutyFitR, nR, R, "RIGHT");

  Serial.println("# fit complete — Kv/Ks applied:");
  printGains();
}

// Least-squares fit of y = m*x + c, then Kv=m (duty per RPM), Ks=c.
// Points are already pre-filtered (non-spinning levels removed) by the caller.
static void fitAndApply(const float *x, const float *y, int n,
                        Wheel &w, const char *name) {
  float sx = 0, sy = 0, sxx = 0, sxy = 0; int used = 0;
  for (int i = 0; i < n; i++) {
    sx += x[i]; sy += y[i]; sxx += x[i]*x[i]; sxy += x[i]*y[i]; used++;
  }
  if (used < 2) { Serial.printf("# %s: too few points, keeping old\n", name); return; }

  float denom = used * sxx - sx * sx;
  if (fabsf(denom) < 1e-6f) { Serial.printf("# %s: degenerate fit, keeping old\n", name); return; }

  float m = (used * sxy - sx * sy) / denom;   // Kv
  float c = (sy - m * sx) / used;             // Ks

  // Sanity guard: Kv should be positive; if the fit is garbage, keep old values.
  if (m <= 0.0f) { Serial.printf("# %s: non-physical slope, keeping old\n", name); return; }

  w.Kv = m; w.Ks = c;
  Serial.printf("# %s fit: Kv=%.4f Ks=%.2f (from %d pts)\n", name, m, c, used);
}
