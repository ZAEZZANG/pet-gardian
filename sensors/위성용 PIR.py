// satellite_pir.ino
// XIAO ESP32-C3 + PIR + MQTT (문 통과 감지)

#include <WiFi.h>
#include <PubSubClient.h>

/* -------------------------------
   🔵 1. Wi-Fi 설정
-------------------------------- */
const char* WIFI_SSID     = "YOUR_WIFI_SSID";       // ← 수정
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";   // ← 수정

/* -------------------------------
   🔵 2. MQTT 설정
-------------------------------- */
const char* MQTT_SERVER    = "RASPBERRY_PI_IP";     // ← 예: "192.168.0.15"
const int   MQTT_PORT      = 1883;
const char* MQTT_CLIENT_ID = "satellite_pir_1";
const char* MQTT_TOPIC     = "room1/person";

/* -------------------------------
   🔵 3. PIR 설정
-------------------------------- */
const int PIR_PIN = D2;      // XIAO ESP32-C3 핀 (GPIO2)

// 디바운스/중복발행 방지
int lastPirState = LOW;
unsigned long lastTriggerTime = 0;
const unsigned long DEBOUNCE_INTERVAL = 3000; // 3초

WiFiClient espClient;
PubSubClient client(espClient);

/* -------------------------------
   WiFi 연결 함수
-------------------------------- */
void connectWiFi() {
  Serial.print("WiFi 연결 중...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi 연결 성공!");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());
}

/* -------------------------------
   MQTT 재연결 함수
-------------------------------- */
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("MQTT 연결 중...");
    if (client.connect(MQTT_CLIENT_ID)) {
      Serial.println("성공!");
      // 현재는 발행만 하므로 subscribe는 필수 아님
      // 필요하면: client.subscribe(MQTT_TOPIC);
    } else {
      Serial.print("실패. 코드=");
      Serial.print(client.state());
      Serial.println(" 1초 후 재시도...");
      delay(1000);
    }
  }
}

/* -------------------------------
   MQTT 발행 함수
-------------------------------- */
void sendDoorCrossEvent() {
  client.publish(MQTT_TOPIC, "door_cross");
  Serial.println("[전송] door_cross");
}

/* -------------------------------
   Setup
-------------------------------- */
void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT); // 필요하면 INPUT_PULLDOWN 으로 변경 가능

  connectWiFi();
  client.setServer(MQTT_SERVER, MQTT_PORT);

  Serial.println("위성 PIR 준비 완료");
}

/* -------------------------------
   Loop — PIR 감지 + MQTT 전송
-------------------------------- */
void loop() {
  if (!client.connected()) {
    reconnectMQTT();
  }
  client.loop();

  int pirState = digitalRead(PIR_PIN);
  unsigned long now = millis();

  // 🔥 LOW → HIGH (문을 통과한 순간으로 간주)
  if (pirState == HIGH && lastPirState == LOW) {
    if (now - lastTriggerTime > DEBOUNCE_INTERVAL) {
      sendDoorCrossEvent();
      lastTriggerTime = now;
    }
  }

  lastPirState = pirState;
  delay(50);
}
