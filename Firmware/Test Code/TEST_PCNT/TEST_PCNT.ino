// Leo Biancaniello + Claude
// TEST: Hardware PCNT Quadrature Encoder
//
// PURPOSE:
//   Validates that the hardware Pulse Counter (PCNT) peripheral correctly
//   reads both motor encoders in full quadrature (4x) mode.
//   Spin either motor by hand and watch tick counts in the Serial Monitor.
//   Positive ticks = forward, negative = backward.
//
// USES THE NEW ESP-IDF v5 / Arduino Core v3+ PCNT API (driver/pulse_cnt.h)
//   - Handle-based: pcnt_new_unit(), pcnt_new_channel()
//   - Replaces the deprecated unit-ID-based driver/pcnt.h API
//   - This is the API that should be used in the final robot firmware
//
// HARDWARE (from pin assignment doc):
//   Left  encoder A -> GPIO1  | Left  encoder B -> GPIO2
//   Right encoder A -> GPIO21 | Right encoder B -> GPIO47
//   Encoder VCC -> 3.3V (ESP), Encoder GND -> GND (ESP)
//
// QUADRATURE DECODING (4x / 1400 ticks per revolution):
//   Two PCNT channels per unit share one hardware counter.
//   Channel 0: A = edge pin,   B = level pin  -> count INC on A-rise, DEC on A-fall;
//              flip both when B is HIGH
//   Channel 1: B = edge pin,   A = level pin  -> count DEC on B-rise, INC on B-fall;
//              flip both when A is HIGH
//   Together this decodes all 4 edges per quadrature cycle.
//   Forward sequence  00->01->11->10  gives positive ticks.
//   Backward sequence 00->10->11->01  gives negative ticks.
//
// LEFT MOTOR INVERSION (hardware-level, in setupEncoder):
//   The left and right motors are mounted on opposite sides of the robot facing
//   opposite directions. When both motors drive forward, their shafts spin in
//   physically mirrored directions, so the raw left encoder reads negative for
//   forward motion. To fix this, the left unit is initialised with
//   invertDirection=true, which swaps INCREASE<->DECREASE on both channels in
//   hardware. Every layer above (PID, odometry) then sees positive = forward on
//   both sides with no further negation needed.
//
// TICKS PER REVOLUTION:
//   Formula: 7 (motor PPR) x 50 (gear ratio) x 4 (edges) = 1400 ticks/rev
//
// SERIAL COMMANDS:
//   'r' -> reset both counters to zero
//
// EXPECTED OUTPUT:
//   Left:    +XX  |  Right:    +XX      (spinning forward by hand)
//   Left:    -XX  |  Right:    -XX      (spinning backward by hand)
//
// WIRING NOTE:
//   Keep encoder signal wires away from motor power wires to avoid noise.

#include "driver/pulse_cnt.h"   // New ESP-IDF v5 API (Arduino Core v3+)
#include "esp_log.h"

// ── Pin definitions ────────────────────────────────────────────────────────────
#define LEFT_ENC_A   1
#define LEFT_ENC_B   2
#define RIGHT_ENC_A  21
#define RIGHT_ENC_B  47

// ── PCNT configuration ─────────────────────────────────────────────────────────
// Counter range: int16_t limits so that wrap-around delta math works correctly.
// The firmware PID loop must read deltas faster than 32767 ticks accumulate.
// At 1400 ticks/rev and 200 RPM max, that gives ~7ms minimum read interval.
#define PCNT_HIGH_LIMIT  32767
#define PCNT_LOW_LIMIT  -32768

// Glitch filter: rejects pulses shorter than this many APB clock cycles.
// APB clock = 80 MHz -> 1 cycle = 12.5 ns.
// 1000 cycles = 12.5 µs. Fine for encoder signals; rejects wire-coupling spikes.
#define GLITCH_FILTER_NS  12500   // nanoseconds (12.5 µs)

static const char* TAG = "PCNT_TEST";

// ── Unit handles ───────────────────────────────────────────────────────────────
static pcnt_unit_handle_t leftUnit  = NULL;
static pcnt_unit_handle_t rightUnit = NULL;


// ── setupEncoder ───────────────────────────────────────────────────────────────
// Allocates one PCNT unit and two channels for a single quadrature encoder.
// The driver manages unit IDs internally; we only keep a handle.
//
// pinA / pinB:      the two encoder signal pins
// invertDirection:  pass true for the left motor to flip the count direction in
//                   hardware, so that positive ticks = forward on both sides
// returns:          the allocated unit handle
static pcnt_unit_handle_t setupEncoder(int pinA, int pinB, bool invertDirection) {

    // 1. Allocate a PCNT unit
    pcnt_unit_config_t unitCfg = {
        .low_limit  = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
        .intr_priority = 0,     // let the driver choose interrupt priority
        .flags = {
            .accum_count = false,   // no software accumulator needed;
        }                           // we read deltas between polls instead
    };
    pcnt_unit_handle_t unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unitCfg, &unit));

    // 2. Enable glitch filter (must be done before pcnt_unit_enable)
    pcnt_glitch_filter_config_t filterCfg = {
        .max_glitch_ns = GLITCH_FILTER_NS,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(unit, &filterCfg));

    // Determine edge actions based on whether this motor needs direction inversion.
    // For the right motor (invertDirection=false): rising A = +1, falling A = -1.
    // For the left motor  (invertDirection=true):  rising A = -1, falling A = +1.
    // Swapping these two actions in hardware is all that's needed; the level
    // (control) actions stay the same because they just mirror whatever the edge
    // actions do.
    pcnt_channel_edge_action_t onRise = invertDirection
        ? PCNT_CHANNEL_EDGE_ACTION_DECREASE
        : PCNT_CHANNEL_EDGE_ACTION_INCREASE;
    pcnt_channel_edge_action_t onFall = invertDirection
        ? PCNT_CHANNEL_EDGE_ACTION_INCREASE
        : PCNT_CHANNEL_EDGE_ACTION_DECREASE;

    // 3. Channel 0: A = edge pin, B = level pin
    //    Normal  (B LOW):  rising A -> onRise,  falling A -> onFall
    //    Inverted (B HIGH): both actions flipped by LEVEL_ACTION_INVERSE
    pcnt_chan_config_t ch0Cfg = {
        .edge_gpio_num  = pinA,
        .level_gpio_num = pinB,
        .flags = {}
    };
    pcnt_channel_handle_t ch0 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &ch0Cfg, &ch0));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch0,
        onRise,                             // rising  edge
        onFall                              // falling edge
    ));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch0,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,     // B LOW  -> normal
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE   // B HIGH -> invert
    ));

    // 4. Channel 1: B = edge pin, A = level pin  (swapped roles for 4x decode)
    //    Channel 1 always uses the opposite actions from channel 0 on its own
    //    edge pin; invertDirection is already baked into onRise/onFall above.
    pcnt_chan_config_t ch1Cfg = {
        .edge_gpio_num  = pinB,
        .level_gpio_num = pinA,
        .flags = {}
    };
    pcnt_channel_handle_t ch1 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(unit, &ch1Cfg, &ch1));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch1,
        onFall,                             // rising  edge -> opposite of ch0
        onRise                              // falling edge -> opposite of ch0
    ));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch1,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,     // A LOW  -> normal
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE   // A HIGH -> invert
    ));

    // 5. Enable then start counting
    ESP_ERROR_CHECK(pcnt_unit_enable(unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(unit));
    ESP_ERROR_CHECK(pcnt_unit_start(unit));

    return unit;
}


// ── setup ──────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println("=== PCNT Quadrature Encoder Test ===");
    Serial.println("Using new ESP-IDF v5 pulse_cnt.h driver");
    Serial.println("Spin either motor by hand to test.");
    Serial.println("Send 'r' to reset both counters.\n");

    // Left motor is mounted mirrored, so invert its direction in hardware.
    // Right motor is the reference direction; no inversion needed.
    leftUnit  = setupEncoder(LEFT_ENC_A,  LEFT_ENC_B,  true);   // inverted
    rightUnit = setupEncoder(RIGHT_ENC_A, RIGHT_ENC_B, false);  // normal

    ESP_LOGI(TAG, "Both PCNT units initialised successfully.");
}


// ── loop ───────────────────────────────────────────────────────────────────────
void loop() {
    // Handle reset command
    if (Serial.available() && Serial.read() == 'r') {
        ESP_ERROR_CHECK(pcnt_unit_clear_count(leftUnit));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(rightUnit));
        Serial.println("Counters reset.");
    }

    // Read both counters
    // pcnt_unit_get_count returns int (underlying 16-bit hardware register).
    // Cast to int16_t so that delta math wraps correctly in the PID firmware.
    int rawLeft  = 0;
    int rawRight = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(leftUnit,  &rawLeft));
    ESP_ERROR_CHECK(pcnt_unit_get_count(rightUnit, &rawRight));

    int16_t leftTicks  = (int16_t)rawLeft;
    int16_t rightTicks = (int16_t)rawRight;

    Serial.printf("Left: %6d  |  Right: %6d\n", leftTicks, rightTicks);

    delay(200);  // 5 Hz display rate
}
