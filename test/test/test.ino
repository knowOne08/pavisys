#include <WiFi.h>

const char* ssid = "XIAO_Hotspot";   // Change SSID name if you want
const char* password = "";   // Minimum 8 characters

void setup() {
  Serial.begin(115200);

  // Start WiFi in AP mode
  WiFi.softAP(ssid, password);

  // Get IP address of the ESP32-C3
  IPAddress IP = WiFi.softAPIP();
  Serial.print("Access Point started! IP address: ");
  Serial.println(IP);
}

void loop() {
  // Nothing needed here, AP will keep running
}
