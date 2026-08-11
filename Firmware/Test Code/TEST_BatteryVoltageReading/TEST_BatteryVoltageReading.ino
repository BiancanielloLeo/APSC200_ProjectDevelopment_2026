// =============================================================================
// Simple Battery Monitor + Variable Motor Speed
// =============================================================================

#define BATTERY_PIN 18
#define PWMA_PIN    4
#define PWMB_PIN    10
#define AIN1_PIN    6
#define AIN2_PIN    5
#define BIN1_PIN    8
#define BIN2_PIN    9
#define STBY_PIN    7

// --- Voltage Divider ---
#define R1          100000.0f
#define R2           47000.0f
#define DIVIDER_RATIO (R2 / (R1 + R2))  // 0.3197
#define ADC_RESOLUTION 4095.0f
#define ADC_VREF    3.3f

// --- Moving Average Filter ---
#define MA_WINDOW 16
static float maBuffer[MA_WINDOW];
static int maIndex = 0;
static bool maFilled = false;

// --- PWM ---
#define PWM_FREQ 20000
#define PWM_RESOLUTION 8

// --- Runtime state ---
static int g_motorDuty = 0;
static bool g_motorsRunning = false;
static unsigned long g_startTime = 0;
static float g_startVoltage = 0.0f;

float batteryReadVoltage() {
    long sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += analogRead(BATTERY_PIN);
    }
    float adcAvg = sum / 8.0f;
    float pinVolts = (adcAvg / ADC_RESOLUTION) * ADC_VREF;
    float battVolts = pinVolts / DIVIDER_RATIO;
    return battVolts;
}

float batteryGetFiltered() {
    float raw = batteryReadVoltage();
    
    // Add to moving average buffer
    maBuffer[maIndex] = raw;
    maIndex = (maIndex + 1) % MA_WINDOW;
    
    // Average the buffer
    float sum = 0;
    for (int i = 0; i < MA_WINDOW; i++) {
        sum += maBuffer[i];
    }
    return sum / MA_WINDOW;
}

// Scale duty based on battery voltage
// At 7.2V nominal: duty = requested
// At lower voltage: duty scales up to maintain torque
// At higher voltage: duty scales down to protect motor
int scaleDutyByBattery(int requestedDuty, float batteryVoltage) {
    float scaleFactor = 7.2f / batteryVoltage;  // Normalize to 7.2V
    int scaledDuty = (int)(requestedDuty * scaleFactor);
    
    // Cap at 255 to avoid overflow
    if (scaledDuty > 255) scaledDuty = 255;
    
    return scaledDuty;
}

void motorSetDuty(int dutyA, int dutyB, float batteryVoltage) {
    int scaledA = scaleDutyByBattery(dutyA, batteryVoltage);
    int scaledB = scaleDutyByBattery(dutyB, batteryVoltage);
    
    digitalWrite(STBY_PIN, HIGH);
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
    ledcWrite(PWMA_PIN, scaledA);
    ledcWrite(PWMB_PIN, scaledB);
}

void motorStop() {
    ledcWrite(PWMA_PIN, 0);
    ledcWrite(PWMB_PIN, 0);
    digitalWrite(STBY_PIN, LOW);
    g_motorsRunning = false;
}

void printMenu() {
    Serial.println("\n=== Battery Monitor + Motor Control ===");
    Serial.println("Commands:");
    Serial.println("  run <0-255> - Run motors at duty (watch battery drain)");
    Serial.println("  stop - Stop motors");
    Serial.println("  a <0-255> - Set motor A speed (one-time)");
    Serial.println("  b <0-255> - Set motor B speed (one-time)");
    Serial.println("  ab <0-255> - Set both motors (one-time)");
    Serial.println("  m - Show this menu");
    Serial.println("====================================\n");
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    pinMode(BATTERY_PIN, INPUT);
    pinMode(AIN1_PIN, OUTPUT);
    pinMode(AIN2_PIN, OUTPUT);
    pinMode(BIN1_PIN, OUTPUT);
    pinMode(BIN2_PIN, OUTPUT);
    pinMode(STBY_PIN, OUTPUT);
    
    ledcAttach(PWMA_PIN, PWM_FREQ, PWM_RESOLUTION);
    ledcAttach(PWMB_PIN, PWM_FREQ, PWM_RESOLUTION);
    
    // Seed moving average
    for (int i = 0; i < MA_WINDOW; i++) {
        maBuffer[i] = batteryReadVoltage();
    }
    
    printMenu();
}

void loop() {
    float v = batteryGetFiltered();
    
    // Print battery every 1000ms
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 1000) {
        lastPrint = millis();
        float pct = (v - 6.0) / (8.4 - 6.0) * 100.0f;
        if (pct > 100) pct = 100;
        if (pct < 0) pct = 0;
        Serial.printf("Battery: %.2fV  (%.1f%%)\n", v, pct);
    }
    
    // Keep motors running if in continuous mode
    if (g_motorsRunning) {
        motorSetDuty(g_motorDuty, g_motorDuty, v);
    }
    
    // Parse serial commands
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        // Split command and argument
        int spaceIndex = input.indexOf(' ');
        String cmd = (spaceIndex > 0) ? input.substring(0, spaceIndex) : input;
        
        if (cmd == "run") {
            int duty = input.substring(spaceIndex + 1).toInt();
            g_motorDuty = duty;
            g_motorsRunning = true;
            g_startTime = millis();
            g_startVoltage = v;
            int scaled = scaleDutyByBattery(duty, v);
            Serial.printf("Running motors at %d (scaled to %d)\n", duty, scaled);
        }
        else if (cmd == "stop") {
            motorStop();
            Serial.println("Motors stopped");
        }
        else if (cmd == "a") {
            int duty = input.substring(spaceIndex + 1).toInt();
            motorSetDuty(duty, 0, v);
            int scaled = scaleDutyByBattery(duty, v);
            Serial.printf("Motor A: %d (scaled to %d)\n", duty, scaled);
        }
        else if (cmd == "b") {
            int duty = input.substring(spaceIndex + 1).toInt();
            motorSetDuty(0, duty, v);
            int scaled = scaleDutyByBattery(duty, v);
            Serial.printf("Motor B: %d (scaled to %d)\n", duty, scaled);
        }
        else if (cmd == "ab") {
            int duty = input.substring(spaceIndex + 1).toInt();
            motorSetDuty(duty, duty, v);
            int scaled = scaleDutyByBattery(duty, v);
            Serial.printf("Both motors: %d (scaled to %d)\n", duty, scaled);
        }
        else if (cmd == "m") {
            printMenu();
        }
    }
}
