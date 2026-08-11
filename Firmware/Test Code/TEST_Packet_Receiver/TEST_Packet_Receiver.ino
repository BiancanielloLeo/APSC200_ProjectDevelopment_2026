/*
 * TEST_Packet_Receiver.ino
 * 
 * UDP Packet Receiver Test for Robot Swarm Server
 * 
 * Packet format (big-endian):
 *   Byte 0:      Command ('R' or 'S')
 *   Bytes 1-4:   Packet number (uint32, big-endian)
 *   Byte 5:      Number of robots
 *   Bytes 6+:    Robot data (13 bytes each)
 *               - Byte 0:     Robot ID
 *               - Bytes 1-4:  X (float32, big-endian)
 *               - Bytes 5-8:  Y (float32, big-endian)
 *               - Bytes 9-12: Theta (float32, big-endian)
 */

#include <WiFi.h>
#include <WiFiUdp.h>

const char* WIFI_SSID = "AgentWifi";
const char* WIFI_PASSWORD = "12345678";
const uint16_t UDP_PORT = 5005;

WiFiUDP udp_server;
unsigned long packet_count = 0;
unsigned long last_packet_time = 0;

unsigned long interval_1 = 0;
unsigned long interval_2 = 0;
unsigned long interval_3 = 0;

bool wifi_connected = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  print_header();
  connect_wifi();
  start_udp_server();

  last_packet_time = millis();
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifi_connected) {
      wifi_connected = true;
      Serial.println("[WiFi] Reconnected\n");
    }
  } else {
    if (wifi_connected) {
      wifi_connected = false;
      Serial.println("[WiFi] Disconnected\n");
    }
    delay(100);
    return;
  }

  int packet_size = udp_server.parsePacket();
  if (packet_size == 0) {
    delay(1);
    return;
  }

  uint8_t buffer[256];
  if (packet_size > 256) {
    packet_size = 256;
  }
  int bytes_read = udp_server.read(buffer, packet_size);

  if (bytes_read < 6) {
    Serial.printf("Error: packet too small (%d bytes)\n\n", bytes_read);
    delay(1);
    return;
  }

  // Parse header (6 bytes total)
  char command = buffer[0];
  uint32_t server_packet_number = read_uint32_be(&buffer[1]);
  uint8_t num_robots = buffer[5];

  // Validate packet size
  if (6 + num_robots * 13 > bytes_read) {
    Serial.printf("Error: packet incomplete (got %d bytes, need %d)\n\n", 
                  bytes_read, 6 + num_robots * 13);
    delay(1);
    return;
  }

  // Calculate timing
  unsigned long now = millis();
  unsigned long time_since_last = now - last_packet_time;

  interval_3 = interval_2;
  interval_2 = interval_1;
  interval_1 = time_since_last;

  float rate = calculate_rate();

  last_packet_time = now;
  packet_count++;

  // Display packet header
  Serial.printf("Pkt #%lu | Server #%u | Interval: %lu ms | Rate: %.1f Hz\n",
                packet_count, server_packet_number, time_since_last, rate);

  // Display robot data
  if (num_robots > 0) {
    for (uint8_t i = 0; i < num_robots; i++) {
      print_robot_data(buffer, i);
    }
  }

  Serial.println();
  delay(1);
}

void print_header() {
  Serial.println("\n");
  Serial.println("╔════════════════════════════════════════════════════════════╗");
  Serial.println("║     UDP Packet Receiver Test - Robot Swarm Server         ║");
  Serial.println("╚════════════════════════════════════════════════════════════╝");
  Serial.println();
}

void connect_wifi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;
    Serial.println("[✓] WiFi Connected");
    Serial.printf("    IP Address: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("    Signal: %d dBm\n", WiFi.RSSI());
  } else {
    wifi_connected = false;
    Serial.println("[✗] WiFi Connection Failed");
  }
  Serial.println();
}

void start_udp_server() {
  if (udp_server.begin(UDP_PORT)) {
    Serial.printf("UDP Server listening on port %d\n", UDP_PORT);
    Serial.println("Waiting for packets...\n");
  } else {
    Serial.println("Error: Failed to start UDP server");
  }
}

// Read 32-bit big-endian unsigned integer
uint32_t read_uint32_be(const uint8_t* data) {
  return ((uint32_t)data[0] << 24) | 
         ((uint32_t)data[1] << 16) | 
         ((uint32_t)data[2] << 8) | 
         ((uint32_t)data[3]);
}

// Read 32-bit big-endian float (IEEE 754)
// Reads bytes in big-endian order and interprets as float
float read_float_be(const uint8_t* data) {
  uint32_t bits = read_uint32_be(data);
  float result;
  memcpy(&result, &bits, sizeof(float));
  return result;
}

// Calculate packet rate from last 3 intervals
float calculate_rate() {
  float rate = 0.0;

  if (interval_1 > 0 && interval_2 > 0 && interval_3 > 0) {
    unsigned long avg = (interval_1 + interval_2 + interval_3) / 3;
    if (avg > 0) {
      rate = 1000.0 / (float)avg;
    }
  } else if (interval_1 > 0 && interval_2 > 0) {
    unsigned long avg = (interval_1 + interval_2) / 2;
    if (avg > 0) {
      rate = 1000.0 / (float)avg;
    }
  }

  return rate;
}

// Print robot data from packet buffer
void print_robot_data(const uint8_t* buffer, uint8_t robot_index) {
  // Offset: header (6 bytes) + robot_index * robot_size (13 bytes)
  size_t offset = 6 + (robot_index * 13);

  uint8_t robot_id = buffer[offset];
  
  // Read floats in big-endian format directly
  float x = read_float_be(&buffer[offset + 1]);
  float y = read_float_be(&buffer[offset + 5]);
  float theta = read_float_be(&buffer[offset + 9]);

  // Convert theta from radians to degrees
  float theta_degrees = theta * 57.29577951f;

  Serial.printf("  Robot %d: x=%.3f y=%.3f theta=%.1f°\n",
                robot_id, x, y, theta_degrees);
}
