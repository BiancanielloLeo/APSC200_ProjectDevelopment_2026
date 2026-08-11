//Leo Biancaniello + Claude

// ============================================================
// This code is intended to test the motors response to different inputs
// When the robot is assembled, run this test first to check the motors
//
// Connect the usb to your device and use the serial monitor to pick between different tests
// Watch the motors and check that the response is as expected
// ============================================================


// --- Pin Definitions ---
#define PWMA_PIN   4
#define AIN1_PIN   6
#define AIN2_PIN   5
#define STBY_PIN   7
#define BIN1_PIN   8
#define BIN2_PIN   9
#define PWMB_PIN   10

// --- PWM Config ---
#define PWM_FREQ       20000   // 20 kHz
#define PWM_RESOLUTION 8       // 8-bit (0–255)
#define MAX_DUTY       178     // 70% of 255 — motor voltage cap
#define TEST_DUTY      128     // 50% — used for on/coast/brake tests
#define RAMP_STEP_MS   15      // ms between ramp steps

// ============================================================
//  Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);

  // LEDC pin-based API (ESP32 Arduino Core v3.0+)
  ledcAttach(PWMA_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(PWMB_PIN, PWM_FREQ, PWM_RESOLUTION);

  // Start safe
  allOff();

  printMenu();
}

// ============================================================
//  Main Loop — Serial Menu
// ============================================================
void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read(); // flush newline

    switch (cmd) {
      case '1': testOnLeft();   break;
      case '2': testOnRight();  break;
      case '3': testOnBoth();   break;
      case '4': testRampLeft(); break;
      case '5': testRampRight();break;
      case '6': testRampBoth(); break;
      case '7': testCoast();    break;
      case '8': testBrake();    break;
      case '0': allOff();       Serial.println("All motors off.\n"); printMenu(); break;
      case 'm': printMenu();    break;
      default:
        Serial.println("Unknown command. Press 'm' to show the menu.\n");
        break;
    }
  }
}

// ============================================================
//  Menu
// ============================================================
void printMenu() {
  Serial.println("========================================");
  Serial.println("         MOTOR TEST MENU");
  Serial.println("  (Place robot on its side first!)");
  Serial.println("========================================");
  Serial.println(" 1 - Left motor on  (50%, forward)");
  Serial.println(" 2 - Right motor on (50%, forward)");
  Serial.println(" 3 - Both motors on (50%, forward)");
  Serial.println(" ------");
  Serial.println(" 4 - Ramp left motor  (0 → 70% → 0)");
  Serial.println(" 5 - Ramp right motor (0 → 70% → 0)");
  Serial.println(" 6 - Ramp both motors (0 → 70% → 0)");
  Serial.println(" ------");
  Serial.println(" 7 - Coast (runs both at 50%, then coasts)");
  Serial.println(" 8 - Brake (runs both at 50%, then brakes)");
  Serial.println(" ------");
  Serial.println(" 0 - All motors off");
  Serial.println(" m - Show this menu");
  Serial.println("========================================\n");
}

// ============================================================
//  Tests 1–3: Motor On
// ============================================================
void testOnLeft() {
  Serial.println("[1] Left motor ON at 50% forward.");
  enableDriver();
  setLeft(true);
  ledcWrite(PWMA_PIN, TEST_DUTY);
  ledcWrite(PWMB_PIN, 0);
  Serial.println("    Press '0' to stop.\n");
}

void testOnRight() {
  Serial.println("[2] Right motor ON at 50% forward.");
  enableDriver();
  setRight(true);
  ledcWrite(PWMA_PIN, 0);
  ledcWrite(PWMB_PIN, TEST_DUTY);
  Serial.println("    Press '0' to stop.\n");
}

void testOnBoth() {
  Serial.println("[3] Both motors ON at 50% forward.");
  enableDriver();
  setLeft(true);
  setRight(true);
  ledcWrite(PWMA_PIN, TEST_DUTY);
  ledcWrite(PWMB_PIN, TEST_DUTY);
  Serial.println("    Press '0' to stop.\n");
}

// ============================================================
//  Tests 4–6: Ramp
// ============================================================
void testRampLeft() {
  Serial.println("[4] Ramping LEFT motor: 0 → 70% → 0");
  enableDriver();
  setLeft(true);
  setRight(true);
  ledcWrite(PWMB_PIN, 0); // right stays off
  ramp(PWMA_PIN, 0, MAX_DUTY);
  delay(500);
  ramp(PWMA_PIN, MAX_DUTY, 0);
  allOff();
  Serial.println("    Done.\n");
  printMenu();
}

void testRampRight() {
  Serial.println("[5] Ramping RIGHT motor: 0 → 70% → 0");
  enableDriver();
  setLeft(true);
  setRight(true);
  ledcWrite(PWMA_PIN, 0); // left stays off
  ramp(PWMB_PIN, 0, MAX_DUTY);
  delay(500);
  ramp(PWMB_PIN, MAX_DUTY, 0);
  allOff();
  Serial.println("    Done.\n");
  printMenu();
}

void testRampBoth() {
  Serial.println("[6] Ramping BOTH motors: 0 → 70% → 0");
  enableDriver();
  setLeft(true);
  setRight(true);
  for (int duty = 0; duty <= MAX_DUTY; duty++) {
    ledcWrite(PWMA_PIN, duty);
    ledcWrite(PWMB_PIN, duty);
    printDuty(duty);
    delay(RAMP_STEP_MS);
  }
  delay(500);
  for (int duty = MAX_DUTY; duty >= 0; duty--) {
    ledcWrite(PWMA_PIN, duty);
    ledcWrite(PWMB_PIN, duty);
    printDuty(duty);
    delay(RAMP_STEP_MS);
  }
  allOff();
  Serial.println("    Done.\n");
  printMenu();
}

// ============================================================
//  Tests 7–8: Coast and Brake
// ============================================================
void testCoast() {
  Serial.println("[7] Both motors at 50% for 2s, then COAST.");
  enableDriver();
  setLeft(true);
  setRight(true);
  ledcWrite(PWMA_PIN, TEST_DUTY);
  ledcWrite(PWMB_PIN, TEST_DUTY);
  delay(2000);
  Serial.println("    Coasting — motors spin down freely...");
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, LOW);
  ledcWrite(PWMA_PIN, 0);
  ledcWrite(PWMB_PIN, 0);
  Serial.println("    Press '0' when done observing.\n");
}

void testBrake() {
  Serial.println("[8] Both motors at 50% for 2s, then BRAKE.");
  enableDriver();
  setLeft(true);
  setRight(true);
  ledcWrite(PWMA_PIN, TEST_DUTY);
  ledcWrite(PWMB_PIN, TEST_DUTY);
  delay(2000);
  Serial.println("    Braking — motors stop hard...");
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, HIGH);
  digitalWrite(BIN1_PIN, HIGH);
  digitalWrite(BIN2_PIN, HIGH);
  ledcWrite(PWMA_PIN, 0);
  ledcWrite(PWMB_PIN, 0);
  Serial.println("    Press '0' when done observing.\n");
}

// ============================================================
//  Helpers
// ============================================================
void enableDriver() {
  digitalWrite(STBY_PIN, HIGH);
}

// Forward: IN1=HIGH, IN2=LOW  |  Reverse: IN1=LOW, IN2=HIGH
void setLeft(bool forward) {
  digitalWrite(AIN1_PIN, forward ? HIGH : LOW);
  digitalWrite(AIN2_PIN, forward ? LOW  : HIGH);
}

void setRight(bool forward) {
  digitalWrite(BIN1_PIN, forward ? HIGH : LOW);
  digitalWrite(BIN2_PIN, forward ? LOW  : HIGH);
}

void allOff() {
  ledcWrite(PWMA_PIN, 0);
  ledcWrite(PWMB_PIN, 0);
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, LOW);
  digitalWrite(STBY_PIN, LOW);
}

void ramp(int pin, int from, int to) {
  int step = (to > from) ? 1 : -1;
  for (int duty = from; duty != to + step; duty += step) {
    ledcWrite(pin, duty);
    printDuty(duty);
    delay(RAMP_STEP_MS);
  }
}

void printDuty(int duty) {
  Serial.print("    Duty: ");
  Serial.print(duty);
  Serial.print("/");
  Serial.print(MAX_DUTY);
  Serial.print("  (");
  Serial.print((duty * 100) / 255);
  Serial.println("%)");
}
