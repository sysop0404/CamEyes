/**
 * CamEyes - ESP32-S3-CAM Node Firmware
 * =====================================
 * MJPEG 스트리밍 + NTP 동기화 + GPIO 동기화 트리거
 *
 * 하드웨어: ESP32-S3-CAM (N16R8) + OV2640
 *
 * 기능:
 *   1. WiFi 접속 (STA 모드) 또는 AP 모드
 *   2. MJPEG HTTP 스트림 서버 (포트 81)
 *   3. NTP 시간 동기화
 *   4. GPIO 트리거 입력 (동기화 촬영)
 *   5. 설정 REST API (해상도, FPS, 노출 변경)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include "esp_camera.h"

// ============================================================
// 설정 - 배포 시 config.yaml과 일치시키세요
// ============================================================

// WiFi 설정
#define WIFI_MODE_STA       true    // true: 공유기 접속, false: AP 모드
#define WIFI_SSID           "CamEyes_Net"
#define WIFI_PASSWORD       ""
#define WIFI_AP_SSID        "CamEyes_Cam1"
#define WIFI_AP_PASSWORD    "12345678"

// 카메라 설정
#define FRAME_SIZE          FRAMESIZE_VGA   // VGA 640x480
#define JPEG_QUALITY        12              // 1-63 (낮을수록 고화질, 대역폭 증가)
#define FB_COUNT            2               // 프레임 버퍼 수 (PSRAM 사용 시 2)

// NTP 설정
#define NTP_SERVER          "pool.ntp.org"
#define GMT_OFFSET_SEC      32400           // KST = UTC+9 = 32400
#define DAYLIGHT_OFFSET_SEC 0

// GPIO 동기화
#define SYNC_TRIGGER_PIN    3               // 동기화 트리거 입력 핀 (마스터에서 수신)

// 스트림 서버 포트
#define STREAM_PORT         81
#define CONFIG_PORT         80

// ============================================================
// ESP32-S3-CAM (AI-Thinker 호환) 핀 매핑
// 보드에 따라 다를 수 있음 - 실물 확인 후 수정
// ============================================================
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     15
#define SIOD_GPIO_NUM      4
#define SIOC_GPIO_NUM      5

#define Y9_GPIO_NUM       16
#define Y8_GPIO_NUM       17
#define Y7_GPIO_NUM       18
#define Y6_GPIO_NUM       12
#define Y5_GPIO_NUM       10
#define Y4_GPIO_NUM        8
#define Y3_GPIO_NUM        9
#define Y2_GPIO_NUM       11
#define VSYNC_GPIO_NUM     6
#define HREF_GPIO_NUM      7
#define PCLK_GPIO_NUM     13

// ============================================================
// 전역 변수
// ============================================================
WebServer configServer(CONFIG_PORT);
WebServer streamServer(STREAM_PORT);
volatile bool syncTriggered = false;
unsigned long lastFrameTime = 0;
int targetFps = 10;
int frameInterval = 1000 / 10;  // ms

// ============================================================
// 카메라 초기화
// ============================================================
bool initCamera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.grab_mode = CAMERA_GRAB_LATEST;

    // PSRAM 사용 시 고해상도 가능
    if (psramFound()) {
        config.frame_size = FRAME_SIZE;
        config.jpeg_quality = JPEG_QUALITY;
        config.fb_count = FB_COUNT;
        Serial.println("[CAM] PSRAM detected, using dual frame buffer");
    } else {
        config.frame_size = FRAMESIZE_QVGA;
        config.jpeg_quality = 20;
        config.fb_count = 1;
        Serial.println("[CAM] WARNING: No PSRAM, falling back to QVGA");
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] ERROR: Camera init failed (0x%x)\n", err);
        return false;
    }

    // 센서 감지 및 설정
    // NOTE: 이 보드는 OV3660이 기본. OV2640 사용 시 아래 설정 필요
    // (판매 페이지 Description 참고)
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        // 카메라 센서 모델 확인
        if (s->id.PID == OV2640_PID) {
            Serial.println("[CAM] Sensor: OV2640 detected");
            s->set_pixformat(s, PIXFORMAT_JPEG);  // OV2640은 JPEG 직접 출력
        } else if (s->id.PID == OV3660_PID) {
            Serial.println("[CAM] Sensor: OV3660 detected");
            // OV3660도 JPEG 지원하지만 초기 설정이 다를 수 있음
            s->set_pixformat(s, PIXFORMAT_JPEG);
        } else {
            Serial.printf("[CAM] Sensor: Unknown PID=0x%04X\n", s->id.PID);
        }

        s->set_framesize(s, FRAME_SIZE);
        s->set_quality(s, JPEG_QUALITY);
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        // 자동 노출/화이트밸런스 - 캘리브레이션 시 수동으로 전환
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_aec2(s, 1);
        s->set_gain_ctrl(s, 1);
    }

    Serial.println("[CAM] Camera initialized OK");
    return true;
}

// ============================================================
// WiFi 초기화
// ============================================================
void initWiFi() {
    if (WIFI_MODE_STA) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);

        int retries = 0;
        while (WiFi.status() != WL_CONNECTED && retries < 30) {
            delay(500);
            Serial.print(".");
            retries++;
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.printf("\n[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
            Serial.printf("[WiFi] RSSI: %d dBm\n", WiFi.RSSI());
        } else {
            Serial.println("\n[WiFi] STA connection failed, starting AP mode");
            WiFi.mode(WIFI_AP);
            WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
            Serial.printf("[WiFi] AP started: %s, IP: %s\n",
                          WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
        }
    } else {
        WiFi.mode(WIFI_AP);
        WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASSWORD);
        Serial.printf("[WiFi] AP started: %s, IP: %s\n",
                      WIFI_AP_SSID, WiFi.softAPIP().toString().c_str());
    }
}

// ============================================================
// NTP 시간 동기화
// ============================================================
void initNTP() {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    Serial.print("[NTP] Synchronizing");
    struct tm timeinfo;
    int retries = 0;
    while (!getLocalTime(&timeinfo) && retries < 10) {
        delay(500);
        Serial.print(".");
        retries++;
    }
    if (retries < 10) {
        Serial.printf("\n[NTP] Time: %04d-%02d-%02d %02d:%02d:%02d\n",
                      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        Serial.println("\n[NTP] WARNING: Time sync failed");
    }
}

// ============================================================
// MJPEG 스트림 핸들러
// ============================================================
void handleStream() {
    WiFiClient client = streamServer.client();

    String response = "HTTP/1.1 200 OK\r\n"
                      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
                      "Access-Control-Allow-Origin: *\r\n"
                      "\r\n";
    client.print(response);

    Serial.printf("[Stream] Client connected from %s\n",
                  client.remoteIP().toString().c_str());

    while (client.connected()) {
        unsigned long now = millis();
        if (now - lastFrameTime < (unsigned long)frameInterval) {
            delay(1);
            continue;
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[Stream] ERROR: Frame capture failed");
            delay(100);
            continue;
        }

        // 타임스탬프 (마이크로초)
        struct timeval tv;
        gettimeofday(&tv, NULL);
        int64_t timestamp_us = (int64_t)tv.tv_sec * 1000000LL + tv.tv_usec;

        // MJPEG 프레임 전송
        String header = "--frame\r\n"
                        "Content-Type: image/jpeg\r\n"
                        "Content-Length: " + String(fb->len) + "\r\n"
                        "X-Timestamp: " + String(timestamp_us) + "\r\n"
                        "\r\n";

        if (client.write(header.c_str(), header.length()) == 0) {
            esp_camera_fb_return(fb);
            break;
        }
        if (client.write(fb->buf, fb->len) == 0) {
            esp_camera_fb_return(fb);
            break;
        }
        client.print("\r\n");

        esp_camera_fb_return(fb);
        lastFrameTime = now;
    }

    Serial.println("[Stream] Client disconnected");
}

// ============================================================
// 설정 API 핸들러
// ============================================================
void handleStatus() {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    String json = "{";
    json += "\"id\":\"cam_node\",";
    json += "\"chip\":\"ESP32-S3\",";
    json += "\"psram\":" + String(psramFound() ? "true" : "false") + ",";
    json += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += "\"fps\":" + String(targetFps) + ",";
    json += "\"timestamp_us\":" + String((int64_t)tv.tv_sec * 1000000LL + tv.tv_usec) + ",";
    json += "\"free_heap\":" + String(ESP.getFreeHeap());
    json += "}";

    configServer.send(200, "application/json", json);
}

void handleSetConfig() {
    if (configServer.hasArg("fps")) {
        targetFps = configServer.arg("fps").toInt();
        targetFps = constrain(targetFps, 1, 30);
        frameInterval = 1000 / targetFps;
        Serial.printf("[Config] FPS set to %d\n", targetFps);
    }

    if (configServer.hasArg("quality")) {
        int q = configServer.arg("quality").toInt();
        q = constrain(q, 4, 63);
        sensor_t *s = esp_camera_sensor_get();
        if (s) s->set_quality(s, q);
        Serial.printf("[Config] JPEG quality set to %d\n", q);
    }

    if (configServer.hasArg("resolution")) {
        String res = configServer.arg("resolution");
        sensor_t *s = esp_camera_sensor_get();
        if (s) {
            if (res == "QVGA") s->set_framesize(s, FRAMESIZE_QVGA);
            else if (res == "VGA") s->set_framesize(s, FRAMESIZE_VGA);
            else if (res == "SVGA") s->set_framesize(s, FRAMESIZE_SVGA);
            else if (res == "XGA") s->set_framesize(s, FRAMESIZE_XGA);
            Serial.printf("[Config] Resolution set to %s\n", res.c_str());
        }
    }

    configServer.send(200, "text/plain", "OK");
}

// ============================================================
// GPIO 동기화 트리거 ISR
// ============================================================
void IRAM_ATTR onSyncTrigger() {
    syncTriggered = true;
}

void initSyncTrigger() {
    pinMode(SYNC_TRIGGER_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(SYNC_TRIGGER_PIN), onSyncTrigger, RISING);
    Serial.printf("[Sync] Trigger pin %d configured\n", SYNC_TRIGGER_PIN);
}

// ============================================================
// Setup
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("================================");
    Serial.println("  CamEyes - ESP32-S3-CAM Node");
    Serial.println("================================");
    Serial.printf("  PSRAM: %s\n", psramFound() ? "Yes" : "No");
    Serial.printf("  Free heap: %d bytes\n", ESP.getFreeHeap());

    // 1. 카메라 초기화
    if (!initCamera()) {
        Serial.println("[FATAL] Camera init failed. Restarting...");
        delay(3000);
        ESP.restart();
    }

    // 2. WiFi 접속
    initWiFi();

    // 3. NTP 동기화
    if (WIFI_MODE_STA && WiFi.status() == WL_CONNECTED) {
        initNTP();
    }

    // 4. GPIO 동기화 트리거
    initSyncTrigger();

    // 5. HTTP 서버 시작
    // 설정 API (포트 80)
    configServer.on("/status", handleStatus);
    configServer.on("/config", HTTP_GET, handleSetConfig);
    configServer.begin();
    Serial.printf("[HTTP] Config server on port %d\n", CONFIG_PORT);

    // 스트림 서버 (포트 81)
    streamServer.on("/stream", handleStream);
    streamServer.begin();
    Serial.printf("[HTTP] Stream server on port %d\n", STREAM_PORT);

    Serial.println();
    Serial.println("=== Ready ===");
    Serial.printf("  Stream: http://%s:%d/stream\n",
                  WiFi.localIP().toString().c_str(), STREAM_PORT);
    Serial.printf("  Status: http://%s/status\n",
                  WiFi.localIP().toString().c_str());
    Serial.println();
}

// ============================================================
// Loop
// ============================================================
void loop() {
    configServer.handleClient();
    streamServer.handleClient();

    // 동기화 트리거 처리
    if (syncTriggered) {
        syncTriggered = false;
        // 트리거 시점의 타임스탬프 기록 (향후 프레임 동기화에 사용)
        struct timeval tv;
        gettimeofday(&tv, NULL);
        // TODO: 트리거된 프레임을 별도 버퍼에 저장하거나 플래그 설정
    }

    delay(1);  // Watchdog 방지
}
