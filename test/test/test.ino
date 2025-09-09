void setup() {
  // Initialize USB Serial for XIAO ESP32-C3
  Serial.begin(115200);
  
  // Wait for USB serial to be ready (critical for XIAO)
  while (!Serial && millis() < 3000) {
    delay(10);
  }
  
  delay(1000);
  Serial.println("=== XIAO ESP32-C3 SERIAL TEST ===");
  Serial.println("USB Serial initialized successfully");
  Serial.flush();
}

void loop() {
  static int count = 0;
  Serial.print("Message #");
  Serial.println(count++);
  Serial.flush();
  delay(2000);
}
