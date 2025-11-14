/*
 * 위성 PIR(사람 입장 감지) → 허브(일체형) HTTP POST 알림
 * - 보드: Seeed XIAO ESP32-C3 (Arduino Core for ESP32)
 * - 동작: PIR이 HIGH로 안정되면 "사람이 출입하였습니다"를 허브에 전송
 * - 채터링 방지: 안정화 시간 + 쿨다운
 */

#include <WiFi.h>
#include <HTTPClient.h>

// ====== [사용자 설정] ======
const char* WIFI_SSID     = "YOUR_WIFI_SSID";
const char* WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

// 허브(일체형)의 HTTP 엔드포인트 (FastAPI 예시: POST /events/entry)
String HUB_BASE_URL = "http://192.168.0.50:8000";
String HUB_ENDPOINT = "/events/entry";

// 노드/위치 식별자
String NODE_ID = "satellite_pir_door1";
String ROOM_ID = "entrance";

// PIR 핀 (XIAO ESP32-C3에서 D2로 가정. 다른 핀 쓰면 바꾸세요)
const int PIR_PIN = D2;  // 또는 숫자 GPIO로 사용 가능 (예: 6, 7 등)

// 감지 판정 파라미터
const unsigned long STABLE_HIGH_MS  = 120;   // HIGH 유지되어야 하는 최소 시간(채터링 방지)
const unsigned long COOLDOWN_MS     = 8000;  // 한 번 전송 후 다음 전송까지 최소 간격
// =========================

unsigned long lastHighStart = 0;
bool          wasHigh       = false;
unsigned long lastSentMs    = 0;

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WiFi] Connecting");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    Serial.print(".");
    delay(300);
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected: " + WiFi.localIP().toString());
  } else {
    Serial.println("\n[WiFi] Failed to connect (will retry in loop)");
  }
}

bool postEntryEvent(const String& message) {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[HTTP] WiFi not connected. Skip send.");
      return false;
    }
  }

  HTTPClient http;
  String url = HUB_BASE_URL + HUB_ENDPOINT;
  http.begin(url);
  http.addHeader("Content-Type", "application/json; charset=utf-8");

  // 예시 JSON 페이로드 (FastAPI에서 request.body로 받기 쉬운 형태)
  // 필요한 필드는 허브 규격에 맞춰 자유롭게 추가하세요.
  String payload = String("{")
    + "\"node_id\":\"" + NODE_ID + "\","
    + "\"room_id\":\"" + ROOM_ID + "\","
    + "\"event\":\"entry\","
    + "\"message\":\"사람이 출입하였습니다\","
    + "\"ts\":" + String((unsigned long)time(nullptr)) +
  "}";

  Serial.println("[HTTP] POST " + url);
  Serial.println("[HTTP] Payload: " + payload);

  int code = http.POST(payload);
  if (code > 0) {
    Serial.printf("[HTTP] Response code: %d\n", code);
    String resp = http.getString();
    Serial.println("[HTTP] Body: " + resp);
  } else {
    Serial.printf("[HTTP] POST failed, error: %s\n", http.errorToString(code).c_str());
  }

  http.end();
  return (code >= 200 && code < 300);
}

void setup() {
  Serial.begin(115200);
  delay(400);

  pinMode(PIR_PIN, INPUT); // HC-SR501은 자체 드라이브 출력 → 내부 풀업/다운 불필요
  connectWiFi();

  Serial.println("\n=== Satellite PIR (Human Entry) ===");
  Serial.println("PIR_PIN: D2 (change if needed)");
  Serial.println("Cooldown: " + String(COOLDOWN_MS) + " ms, StableHigh: " + String(STABLE_HIGH_MS) + " ms");
}

void loop() {
  int pir = digitalRead(PIR_PIN);
  unsigned long now = millis();

  if (pir == HIGH) {
    if (!wasHigh) {
      // 막 HIGH로 올라온 시점 기록
      lastHighStart = now;
      wasHigh = true;
    } else {
      // HIGH가 일정 시간 이상 유지되면 '감지'로 판정
      if (now - lastHighStart >= STABLE_HIGH_MS) {
        // 쿨다운 체크
        if (now - lastSentMs >= COOLDOWN_MS) {
          Serial.println("▶ 사람 감지됨 → '사람이 출입하였습니다' 전송");

          bool ok = postEntryEvent("사람이 출입하였습니다");
          if (ok) {
            Serial.println("✅ 허브에 전송 성공");
          } else {
            Serial.println("❌ 전송 실패 (WiFi/HTTP 문제)");
          }
          lastSentMs = now;
        }
      }
    }
  } else {
    // LOW 상태로 내려옴 → 상태 리셋
    wasHigh = false;
  }

  delay(10); // 루프 템포
}
