/*
 * ESP32 UDP Packet Display Test
 * 
 * Connects to WiFi and displays received UDP packets in a readable format.
 * Shows connection status, timestamps, and complete packet details.
 * 
 * Packet format (matching udp.py):
 *   Header: command (1B) + packet_number (4B) + num_robots (1B) = 6 bytes
 *   Each robot: robot_id (1B) + x (4B float) + y (4B float) + theta (4B float) = 13 bytes
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <time.h>

// ─── WiFi Configuration ──────────────────────────────────────────────────────
const char* WIFI_SSID = "AgentWifi";
const char* WIFI_PASSWORD = "12345678";

// ─── UDP Configuration ───────────────────────────────────────────────────────
const uint16_t UDP_PORT = 5005;
const size_t UDP_BUFFER_SIZE = 256;

// ─── Global Objects ──────────────────────────────────────────────────────────
WiFiUDP udpServer;
unsigned long last_packet_time = 0;
uint32_t packet_count = 0;
bool wifi_connected = false;

// ─── Display formatting constants ────────────────────────────────────────────
const char* DIVIDER = "════════════════════════════════════════════════════════════════";

// ─── Function prototypes ─────────────────────────────────────────────────────
void setup_wifi();
void setup_ntp();
String get_formatted_time();
void display_wifi_status();
void parse_and_display_packet(const uint8_t* data, size_t length);

// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1000);  // Allow serial to initialize

  Serial.println("\n\n");
  Serial.println(DIVIDER);
  Serial.println("  ESP32 UDP PACKET DISPLAY TEST");
  Serial.println(DIVIDER);
  Serial.println();

  // Setup WiFi connection
  setup_wifi();

  // Setup NTP for accurate timestamps
  setup_ntp();

  // Start UDP server
  if (udpServer.begin(UDP_PORT)) {
    Serial.printf("[UDP] Server listening on port %d\n\n", UDP_PORT);
  } else {
    Serial.println("[ERROR] Failed to start UDP server!");
  }

  display_wifi_status();
}

// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  // Check WiFi connection
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifi_connected) {
      wifi_connected = true;
      display_wifi_status();
    }

    // Check for incoming UDP packets
    int packet_size = udpServer.parsePacket();
    if (packet_size > 0) {
      // Allocate buffer for packet data
      uint8_t* buffer = new uint8_t[packet_size];
      int bytes_read = udpServer.read(buffer, packet_size);

      if (bytes_read > 0) {
        packet_count++;
        last_packet_time = millis();

        // Display the packet
        parse_and_display_packet(buffer, bytes_read);
      }

      delete[] buffer;
    }
  } else {
    if (wifi_connected) {
      wifi_connected = false;
      Serial.println("\n[!] WiFi connection lost!");
      display_wifi_status();
    }
  }

  delay(10);  // Small delay to prevent watchdog issues
}

// ═════════════════════════════════════════════════════════════════════════════
// SETUP: Initialize WiFi connection
// ═════════════════════════════════════════════════════════════════════════════
void setup_wifi() {
  Serial.println("[WiFi] Connecting to network...");
  Serial.printf("  SSID: %s\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  int max_attempts = 20;  // 10 seconds with 500ms delays

  while (WiFi.status() != WL_CONNECTED && attempts < max_attempts) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    Serial.println("[✓] WiFi Connected!");
  } else {
    wifi_connected = false;
    Serial.println("[✗] WiFi Connection Failed!");
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// SETUP: Initialize NTP time synchronization
// ═════════════════════════════════════════════════════════════════════════════
void setup_ntp() {
  Serial.println("[NTP] Synchronizing time with NTP server...");

  // Configure time with NTP server (timezone offset in seconds, DST offset)
  configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");

  // Wait for time to be set
  time_t now = time(nullptr);
  int attempts = 0;
  while (now < 24 * 3600 && attempts < 20) {
    delay(500);
    Serial.print(".");
    now = time(nullptr);
    attempts++;
  }

  Serial.println();
  if (now > 24 * 3600) {
    Serial.println("[✓] Time synchronized");
  } else {
    Serial.println("[!] Time sync incomplete (packets will still display)");
  }
}

// ═════════════════════════════════════════════════════════════════════════════
// DISPLAY: Format current time as readable string
// ═════════════════════════════════════════════════════════════════════════════
String get_formatted_time() {
  time_t now = time(nullptr);
  struct tm* timeinfo = localtime(&now);

  char buffer[32];
  strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", timeinfo);
  return String(buffer);
}

// ═════════════════════════════════════════════════════════════════════════════
// DISPLAY: Show WiFi connection status
// ═════════════════════════════════════════════════════════════════════════════
void display_wifi_status() {
  Serial.println(DIVIDER);
  Serial.println("WiFi STATUS:");
  Serial.println(DIVIDER);

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("  Status:     ✓ CONNECTED\n");
    Serial.printf("  SSID:       %s\n", WiFi.SSID().c_str());
    Serial.printf("  IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("  Gateway:    %s\n", WiFi.gatewayIP().toString().c_str());
    Serial.printf("  Signal:     %d dBm\n", WiFi.RSSI());
  } else {
    Serial.printf("  Status:     ✗ DISCONNECTED\n");
    Serial.printf("  Reason:     %d\n", WiFi.status());
  }

  Serial.println();
}

// ═════════════════════════════════════════════════════════════════════════════
// PARSE: Extract and display UDP packet contents
// ═════════════════════════════════════════════════════════════════════════════
void parse_and_display_packet(const uint8_t* data, size_t length) {
  // Check minimum header size (6 bytes)
  if (length < 6) {
    Serial.println("\n[!] Packet too small (< 6 bytes)");
    return;
  }

  // Extract header (6 bytes total)
  char command_byte = data[0];
  uint32_t packet_number = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
  uint8_t num_robots = data[5];
  const char* command_str = (command_byte == 'R') ? "RUN" : "STOP";

  // Verify packet size is correct
  size_t expected_size = 6 + (num_robots * 13);  // 6 byte header + 13 bytes per robot
  if (length < expected_size) {
    Serial.printf("\n[!] Packet size mismatch! Got %d bytes, expected %d\n", length, expected_size);
    return;
  }

  // ─── Display packet header ───────────────────────────────────────────────────
  Serial.println();
  Serial.println(DIVIDER);
  Serial.printf("PACKET #%lu | Time: %s\n", packet_count, get_formatted_time().c_str());
  Serial.println(DIVIDER);

  Serial.printf("  Packet Number: %u (0x%08X)\n", packet_number, packet_number);
  Serial.printf("  Command:       %s (%c)\n", command_str, command_byte);
  Serial.printf("  Num Robots:    %u\n", num_robots);
  Serial.printf("  Total Size:    %u bytes\n", length);
  Serial.println();

  // ─── Display robot data ──────────────────────────────────────────────────────
  if (num_robots > 0) {
    Serial.println("ROBOT DATA:");
    Serial.println("─────────────────────────────────────────────────────────────────");

    for (uint8_t i = 0; i < num_robots; i++) {
      // Extract robot entry (13 bytes each)
      size_t offset = 6 + (i * 13);

      uint8_t robot_id = data[offset];

      // Extract floats (4 bytes each, big-endian)
      float x = extract_float(data, offset + 1);
      float y = extract_float(data, offset + 5);
      float theta = extract_float(data, offset + 9);

      // Display robot entry
      Serial.printf("  [Robot %u]\n", robot_id);
      Serial.printf("    Position: x=%.4f m, y=%.4f m\n", x, y);
      Serial.printf("    Angle:    θ=%.4f rad (%.2f°)\n", theta, theta * 57.2958f);  // Convert rad to degrees
      Serial.println();
    }
  } else {
    Serial.println("  (No robots in this packet)\n");
  }

  Serial.println(DIVIDER);
  Serial.println();
}

// ═════════════════════════════════════════════════════════════════════════════
// UTILITY: Extract big-endian float from byte array
// ═════════════════════════════════════════════════════════════════════════════
float extract_float(const uint8_t* data, size_t offset) {
  uint8_t bytes[4] = {data[offset], data[offset + 1], data[offset + 2], data[offset + 3]};
  float* ptr = (float*)bytes;
  return *ptr;
}
