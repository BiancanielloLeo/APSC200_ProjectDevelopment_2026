/*
 * TEST_Packet_Receiver.ino - DEBUG VERSION
 * 
 * ESP32 UDP Packet Receiver for server testing
 * With detailed rate calculation debugging
 */

#include <WiFi.h>
#include <WiFiUdp.h>

// ─── WiFi Configuration ──────────────────────────────────────────────────────
const char* WIFI_SSID = "AgentWifi";
const char* WIFI_PASSWORD = "12345678";

// ─── UDP Configuration ───────────────────────────────────────────────────────
const uint16_t UDP_PORT = 5005;

// ─── Global Objects ────────────────────────────────────────────────────────
WiFiUDP udpServer;

// ─── Packet Tracking ───────────────────────────────────────────────────────
unsigned long total_packets = 0;
unsigned long last_packet_time = 0;
unsigned long prev_packet_time = 0;

// ─── Rate Tracking - Store last 3 intervals ────────────────────────────────
unsigned long interval1 = 0;  // Most recent
unsigned long interval2 = 0;  // Second most recent
unsigned long interval3 = 0;  // Third most recent

bool wifi_connected = false;

// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n");
  Serial.println("ESP32 UDP Packet Receiver - DEBUG");
  Serial.println("=================================\n");

  setup_wifi();

  if (udpServer.begin(UDP_PORT)) {
    Serial.printf("Listening on port %d\n", UDP_PORT);
    Serial.println("Waiting for packets...\n");
  } else {
    Serial.println("ERROR: Failed to start UDP server!");
  }

  last_packet_time = millis();
  prev_packet_time = millis();
}

// ═════════════════════════════════════════════════════════════════════════════
void setup_wifi() {
  Serial.print("Connecting to WiFi...");
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
    Serial.printf("Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Signal: %d dBm\n\n", WiFi.RSSI());
  } else {
    wifi_connected = false;
    Serial.println("Connection Failed!\n");
  }
}

// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifi_connected) {
      wifi_connected = true;
      Serial.println("WiFi reconnected!\n");
    }

    int packet_size = udpServer.parsePacket();

    if (packet_size > 0) {
      unsigned long current_time = millis();
      
      uint8_t* buffer = new uint8_t[packet_size];
      int bytes_read = udpServer.read(buffer, packet_size);

      if (bytes_read > 0 && bytes_read >= 6) {
        // Extract header
        char command_byte = buffer[0];
        
        // Packet number: bytes 1-4 (big-endian)
        uint32_t server_pkt_num = ((uint32_t)buffer[1] << 24) | 
                                  ((uint32_t)buffer[2] << 16) | 
                                  ((uint32_t)buffer[3] << 8) | 
                                  ((uint32_t)buffer[4]);
        
        uint8_t num_robots = buffer[5];

        // Calculate time since last packet
        unsigned long time_since_last = current_time - last_packet_time;
        
        // Shift intervals
        interval3 = interval2;
        interval2 = interval1;
        interval1 = time_since_last;

        // Calculate rate from last 3 intervals
        float rate = 0.0;
        if (interval1 > 0 && interval2 > 0 && interval3 > 0) {
          unsigned long avg_interval = (interval1 + interval2 + interval3) / 3;
          if (avg_interval > 0) {
            rate = 1000.0 / (float)avg_interval;
          }
        } else if (interval1 > 0 && interval2 > 0) {
          unsigned long avg_interval = (interval1 + interval2) / 2;
          if (avg_interval > 0) {
            rate = 1000.0 / (float)avg_interval;
          }
        }

        last_packet_time = current_time;
        total_packets++;

        // Display packet info
        Serial.printf("Pkt #%lu | Server #%u | Interval: %lu ms | Rate: %.1f Hz\n",
                      total_packets, server_pkt_num, time_since_last, rate);

        // Display robot positions
        if (num_robots > 0 && (6 + num_robots * 13) <= bytes_read) {
          for (int i = 0; i < num_robots; i++) {
            size_t offset = 6 + (i * 13);
            
            if (offset + 13 > bytes_read) {
              break;
            }
            
            uint8_t robot_id = buffer[offset];
            
            // Parse floats
            float x, y, theta;
            memcpy(&x, &buffer[offset + 1], 4);
            memcpy(&y, &buffer[offset + 5], 4);
            memcpy(&theta, &buffer[offset + 9], 4);

            Serial.printf("  Robot %d: x=%.2f y=%.2f theta=%.1f\n",
                          robot_id, x, y, theta * 57.2958);
          }
        }
        Serial.println();
      }

      delete[] buffer;
    }

  } else {
    if (wifi_connected) {
      wifi_connected = false;
      Serial.println("WiFi lost!\n");
    }
  }

  delay(1);
}
