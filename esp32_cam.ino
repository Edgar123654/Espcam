/*
 * ESP32-CAM (AI Thinker) - Servidor de Captura de Imagens
 * Compatível com o código Python raspberrypi.py
 * 
 * Hardware: ESP32-CAM (AI Thinker)
 * 
 * Funcionalidades:
 * - WiFiManager para configuração WiFi via portal web
 * - Conecta ao WiFi automaticamente ou cria portal de configuração
 * - Captura imagens da câmera
 * - Servidor HTTP com rotas /capture e /stream
 * - Envia imagens JPEG para o código Python
 * 
 * Biblioteca necessária:
 * - WiFiManager by tzapu (instalar via Library Manager)
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <WiFiManager.h>  // WiFiManager para configuração WiFi automática

// ======================= CONFIGURAÇÃO WIFI =======================
// WiFiManager gerencia automaticamente a conexão WiFi
// Na primeira execução, cria um Access Point "ESP32-CAM-Config"
// Conecte-se a ele e configure o WiFi via portal web
WiFiManager wm;
// ================================================================

// ======================= CONFIGURAÇÃO CAMERA =======================
// Configuração para ESP32-CAM (AI Thinker)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22
// ================================================================

// Servidor Web na porta 80
WebServer server(80);

// Variável para armazenar o último frame capturado
camera_fb_t * fb = NULL;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  // Inicializar câmera
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Configuração de resolução baseada na memória disponível
  // Para ESP32-CAM com PSRAM, use FRAMESIZE_UXGA (1600x1200)
  // Para ESP32-CAM sem PSRAM, use FRAMESIZE_SVGA (800x600) ou menor
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;  // 1600x1200
    config.jpeg_quality = 10;            // 0-63, menor = maior qualidade
    config.fb_count = 2;
    Serial.println("✅ PSRAM encontrado - usando resolução UXGA");
  } else {
    config.frame_size = FRAMESIZE_SVGA;  // 800x600
    config.jpeg_quality = 12;            // 0-63
    config.fb_count = 1;
    Serial.println("⚠️ PSRAM não encontrado - usando resolução SVGA");
  }

  // Inicializar câmera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("❌ Erro ao inicializar câmera: 0x%x\n", err);
    return;
  }
  Serial.println("✅ Câmera inicializada com sucesso!");

  // Configurar sensor (ajuste conforme seu modelo)
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);     // -2 a 2
  s->set_contrast(s, 0);       // -2 a 2
  s->set_saturation(s, 0);     // -2 a 2
  s->set_special_effect(s, 0); // 0 a 6
  s->set_whitebal(s, 1);       // 0 = desabilitado, 1 = habilitado
  s->set_awb_gain(s, 1);       // 0 = desabilitado, 1 = habilitado
  s->set_wb_mode(s, 0);        // 0 a 4
  s->set_exposure_ctrl(s, 1);  // 0 = desabilitado, 1 = habilitado
  s->set_aec2(s, 0);           // 0 = desabilitado, 1 = habilitado
  s->set_ae_level(s, 0);       // -2 a 2
  s->set_aec_value(s, 300);    // 0 a 1200
  s->set_gain_ctrl(s, 1);      // 0 = desabilitado, 1 = habilitado
  s->set_agc_gain(s, 0);       // 0 a 30
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 a 6
  s->set_bpc(s, 0);            // 0 = desabilitado, 1 = habilitado
  s->set_wpc(s, 1);            // 0 = desabilitado, 1 = habilitado
  s->set_raw_gma(s, 1);        // 0 = desabilitado, 1 = habilitado
  s->set_lenc(s, 1);           // 0 = desabilitado, 1 = habilitado
  s->set_hmirror(s, 0);        // 0 = desabilitado, 1 = habilitado
  s->set_vflip(s, 0);          // 0 = desabilitado, 1 = habilitado
  s->set_dcw(s, 1);            // 0 = desabilitado, 1 = habilitado
  s->set_colorbar(s, 0);       // 0 = desabilitado, 1 = habilitado

  // Configurar WiFiManager
  // Descomente a linha abaixo para resetar as configurações WiFi salvas
  // wm.resetSettings();
  
  // Configurar timeout do portal (em segundos)
  wm.setConfigPortalTimeout(180);  // 3 minutos
  
  // Nome do Access Point quando não conseguir conectar
  wm.setAPStaticIPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  
  // Título do portal de configuração
  wm.setTitle("ESP32-CAM WiFi Config");
  
  // Conectar ao WiFi usando WiFiManager
  Serial.println("🔄 Iniciando WiFiManager...");
  
  // Tenta conectar à rede WiFi salva
  // Se não conseguir, cria um Access Point para configuração
  if (!wm.autoConnect("ESP32-CAM-Config", "config123")) {
    Serial.println("❌ Falha ao conectar e timeout do portal de configuração");
    Serial.println("🔄 Reiniciando ESP32...");
    delay(3000);
    ESP.restart();
  }
  
  // Se chegou aqui, está conectado ao WiFi
  Serial.println("✅ WiFi conectado!");
  Serial.print("📡 IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("📡 MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("📡 RSSI: ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");

  // Configurar rotas do servidor
  server.on("/", handleRoot);
  server.on("/capture", handleCapture);
  server.on("/stream", handleStream);
  server.on("/status", handleStatus);
  server.on("/resetwifi", handleResetWiFi);  // Rota para resetar configuração WiFi
  
  // Iniciar servidor
  server.begin();
  Serial.println("🚀 Servidor HTTP iniciado!");
  Serial.println("📸 Rotas disponíveis:");
  Serial.println("   - http://" + WiFi.localIP().toString() + "/capture");
  Serial.println("   - http://" + WiFi.localIP().toString() + "/stream");
  Serial.println("   - http://" + WiFi.localIP().toString() + "/status");
}

void loop() {
  server.handleClient();
  delay(1);
}

// Rota raiz - informações do servidor
void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<title>ESP32-CAM Server</title></head><body>";
  html += "<h1>📸 ESP32-CAM Server</h1>";
  html += "<p><strong>IP:</strong> " + WiFi.localIP().toString() + "</p>";
  html += "<p><strong>Status WiFi:</strong> " + String(WiFi.status() == WL_CONNECTED ? "Conectado" : "Desconectado") + "</p>";
  html += "<h2>Rotas disponíveis:</h2>";
  html += "<ul>";
  html += "<li><a href='/capture'>/capture</a> - Capturar uma imagem</li>";
  html += "<li><a href='/stream'>/stream</a> - Stream de vídeo</li>";
  html += "<li><a href='/status'>/status</a> - Status do sistema (JSON)</li>";
  html += "<li><a href='/resetwifi' onclick='return confirm(\"Tem certeza que deseja resetar o WiFi?\");'>/resetwifi</a> - Resetar configuração WiFi</li>";
  html += "</ul>";
  html += "<hr>";
  html += "<h2>Configuração WiFi:</h2>";
  html += "<p>Para reconfigurar o WiFi, acesse <a href='/resetwifi'>/resetwifi</a></p>";
  html += "<p>Ou mantenha o botão RESET pressionado por 10 segundos (se implementado)</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// Rota /capture - Captura uma imagem e retorna JPEG
void handleCapture() {
  Serial.println("📸 Requisição de captura recebida");
  
  // Capturar frame
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("❌ Erro ao capturar frame");
    server.send(500, "text/plain", "Erro ao capturar imagem");
    return;
  }

  Serial.printf("✅ Frame capturado: %d bytes\n", fb->len);

  // Enviar imagem JPEG
  server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Length", String(fb->len));
  server.send(200, "image/jpeg", (const char *)fb->buf, fb->len);

  // Liberar frame
  esp_camera_fb_return(fb);
  fb = NULL;
  
  Serial.println("✅ Imagem enviada com sucesso");
}

// Rota /stream - Stream de vídeo MJPEG
void handleStream() {
  Serial.println("📹 Requisição de stream recebida");
  
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (server.client().connected()) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("❌ Erro ao capturar frame para stream");
      break;
    }

    response = "--frame\r\n";
    response += "Content-Type: image/jpeg\r\n";
    response += "Content-Length: " + String(fb->len) + "\r\n\r\n";
    server.sendContent(response);
    server.sendContent((const char *)fb->buf, fb->len);
    server.sendContent("\r\n");

    esp_camera_fb_return(fb);
    fb = NULL;
    
    delay(100); // ~10 FPS
  }
  
  Serial.println("📹 Stream encerrado");
}

// Rota /status - Status do sistema
void handleStatus() {
  String json = "{";
  json += "\"wifi_connected\":" + String(WiFi.status() == WL_CONNECTED ? "true" : "false") + ",";
  json += "\"ip_address\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"mac_address\":\"" + WiFi.macAddress() + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"uptime_ms\":" + String(millis());
  json += "}";
  
  server.send(200, "application/json", json);
}

// Rota /resetwifi - Resetar configuração WiFi e abrir portal de configuração
void handleResetWiFi() {
  Serial.println("🔄 Resetando configuração WiFi...");
  
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'>";
  html += "<meta http-equiv='refresh' content='5;url=/'>";
  html += "<title>Reset WiFi</title></head><body>";
  html += "<h1>🔄 Resetando WiFi...</h1>";
  html += "<p>As configurações WiFi foram resetadas.</p>";
  html += "<p>O ESP32 será reiniciado e abrirá um portal de configuração.</p>";
  html += "<p>Conecte-se à rede <strong>ESP32-CAM-Config</strong> (senha: config123)</p>";
  html += "<p>Redirecionando em 5 segundos...</p>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
  
  delay(1000);
  wm.resetSettings();
  delay(1000);
  ESP.restart();
}

