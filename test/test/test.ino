#include <WiFi.h>

// Change to your WiFi credentials
const char* ssid = "ephemeral";
const char* password = "9924408516";
#define DIO0  D0  // IO3

void preboot_lo_ra_isolate() {

  // Make DIO pins inputs with pullups so they don't float and trigger interrupts
  pinMode(DIO0, INPUT_PULLUP);
  // pinMode(LORA_DIO1, INPUT_PULLUP);

  // Optional short delay for pins to settle
  delay(10);
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println("\n=== ESP32-C3 WiFi STA Test ===");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 20) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[OK] Connected!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n[ERR] Failed to connect");
  }
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 5000) {
    Serial.print("WiFi status: ");
    Serial.println(WiFi.status());
    lastPrint = millis();
  }
}
