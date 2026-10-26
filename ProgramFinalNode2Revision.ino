#include "driver/twai.h"
#include <SPI.h>
#include <TFT_eSPI.h>

TFT_eSPI tft = TFT_eSPI();

#define TX_PIN 21
#define RX_PIN 22
#define PIN_BUZZER 13

// ==========================================
// STRUKTUR GEOMETRI LAYAR BARU
// ==========================================
const int Y_LINE[7] = {30, 74, 118, 162, 206, 250, 294};
const int Y_ROW[6]  = {40, 84, 128, 172, 216, 260};

uint16_t last_rpm = 65535;
uint8_t last_load = 255, last_eng = 255, last_map = 255, last_speed = 255;
float last_volt = 255.0;

uint8_t current_rpm_state = 0, current_load_state = 0, current_eng_state = 0;
uint8_t current_map_state = 0, current_speed_state = 0, current_volt_state = 0;

unsigned long blinkTimer = 0; bool blinkState = false;
unsigned long lastRequestTime = 0; uint8_t pidIndex = 0;

const uint8_t pids[] = {0x0C, 0x04, 0x05, 0x0B, 0x0D, 0x42};
const int VLINE_1 = 100, VLINE_2 = 240, VLINE_3 = 435;

uint8_t raw_rpm_A = 0, raw_rpm_B = 0;
uint8_t raw_load = 0, raw_eng = 0, raw_map = 0, raw_speed = 0;
uint8_t raw_volt_A = 0, raw_volt_B = 0;

String serialBuffer = "";
String lastCmdToDraw = "";
bool needToDrawCmd = false; // Flag agar TFT tidak mem-blokir Serial
bool elm_space = true, elm_header = false;

// ==========================================
// LOGIKA STATUS
// ==========================================
uint8_t getNormalState(int val, int warn_val, int alarm_val) {
  if (val >= alarm_val) return 3; 
  if (val >= warn_val) return 2;  
  return 1; 
}

uint8_t getTempState(int val, int cold_val, int warn_val, int alarm_val) {
  if (val >= alarm_val) return 3; 
  if (val >= warn_val) return 2;  
  if (val < cold_val) return 0;   
  return 1; 
}

uint8_t getVoltState(float val) {
  if (val < 18.0 || val >= 32.0) return 3; 
  if (val < 23.0 || val >= 29.5) return 2; 
  return 1; 
}

uint16_t stateToColor(uint8_t state) {
  if (state == 0) return tft.color565(0, 191, 255);
  if (state == 1) return TFT_GREEN; 
  if (state == 2) return TFT_ORANGE; 
  if (state == 3) return TFT_RED; 
  return TFT_WHITE;
}

// ==========================================
// FUNGSI VISUAL BAR
// ==========================================
void drawDynamicStairBar(int x, int y, int val, int max_val, int warn_val, int alarm_val, int cold_val = -1) {
  int max_segments = 20;
  int filled_segments = map(val, 0, max_val, 0, max_segments);
  if(filled_segments > max_segments) filled_segments = max_segments;

  for (int i = 0; i < max_segments; i++) {
    int seg_height = 6 + (i * 1); int y_offset = y + (25 - seg_height);
    int seg_val = (i + 1) * max_val / max_segments;
    uint16_t seg_color = TFT_GREEN;

    if (seg_val >= alarm_val) seg_color = TFT_RED;
    else if (seg_val >= warn_val) seg_color = TFT_ORANGE;
    else if (cold_val != -1 && seg_val < cold_val) seg_color = tft.color565(0, 191, 255);

    if (i < filled_segments) tft.fillRect(x + (i * 9), y_offset, 7, seg_height, seg_color);
    else tft.fillRect(x + (i * 9), y_offset, 7, seg_height, TFT_DARKGREY);
  }
}

void drawVoltStairBar(int x, int y, float val) {
  int max_segments = 20;
  float max_volt_scale = 35.0; 
  int filled_segments = (val / max_volt_scale) * max_segments;
  if(filled_segments > max_segments) filled_segments = max_segments;

  for (int i = 0; i < max_segments; i++) {
    int seg_height = 6 + (i * 1); int y_offset = y + (25 - seg_height);
    float seg_val = (i + 1) * max_volt_scale / max_segments;
    uint16_t seg_color;

    if (seg_val < 18.0 || seg_val >= 32.0) seg_color = TFT_RED;
    else if (seg_val < 23.0 || seg_val >= 29.5) seg_color = TFT_ORANGE;
    else seg_color = TFT_GREEN;

    if (i < filled_segments) tft.fillRect(x + (i * 9), y_offset, 7, seg_height, seg_color);
    else tft.fillRect(x + (i * 9), y_offset, 7, seg_height, TFT_DARKGREY);
  }
}

void drawAlertIcon(int row_idx, uint8_t state) {
  int radius = 12; int center_x = VLINE_3 + ((480 - VLINE_3) / 2); int center_y = Y_ROW[row_idx] + 10;
  if (state == 0 || state == 1) { tft.fillCircle(center_x, center_y, radius, TFT_BLACK); }
  else if (state == 2) { tft.fillCircle(center_x, center_y, radius, TFT_ORANGE); tft.setTextColor(TFT_BLACK); tft.setTextSize(2); tft.drawString("!", center_x - 5, center_y - 7); }
  else if (state == 3) {
    if (blinkState) { tft.fillCircle(center_x, center_y, radius, TFT_RED); tft.setTextColor(TFT_WHITE); tft.setTextSize(2); tft.drawString("!", center_x - 5, center_y - 7); }
    else { tft.fillCircle(center_x, center_y, radius, TFT_BLACK); }
  }
}

// ==========================================
// ELM327 PARSER (DIPERBAIKI UNTUK AUTO DOCTOR)
// ==========================================
String byteToHex(uint8_t val) {
  String h = String(val, HEX); h.toUpperCase(); if (h.length() < 2) h = "0" + h; return h;
}

void processELMCommand(String cmd) {
  cmd.trim(); cmd.toUpperCase(); cmd.replace(" ", "");
  
  // Perbaikan 1: Balas \r> jika command kosong (Ping dari Auto Doctor)
  if (cmd == "") {
    Serial.print("\r>");
    return;
  }

  if (cmd.startsWith("01") && cmd.length() == 5) {
    cmd = cmd.substring(0, 4); 
  }

  // Perbaikan 2: Catch-all AT Commands untuk lolos inisialisasi software
  if (cmd.startsWith("AT")) {
    if (cmd == "ATZ" || cmd == "ATWS") { elm_space = true; elm_header = false; Serial.print("ELM327 v1.5\r>"); }
    else if (cmd == "ATI") { Serial.print("ELM327 v1.5\r>"); }
    else if (cmd == "ATH0") { elm_header = false; Serial.print("OK\r>"); } 
    else if (cmd == "ATH1") { elm_header = true; Serial.print("OK\r>"); }  
    else if (cmd == "ATS0") { elm_space = false; Serial.print("OK\r>"); }  
    else if (cmd == "ATS1") { elm_space = true; Serial.print("OK\r>"); }  
    else if (cmd == "ATDP") { Serial.print("ISO 15765-4 (CAN 11/500)\r>"); } 
    else if (cmd == "ATDPN") { Serial.print("6\r>"); }
    else if (cmd == "ATRV") { float v = ((raw_volt_A * 256.0) + raw_volt_B) / 1000.0; Serial.print(String(v, 1) + "V\r>"); }
    else { Serial.print("OK\r>"); } // <-- Menangani ATE0, ATL0, ATSP0 dll
    return;
  }

  String response = ""; bool hasData = false; uint8_t dataBytes = 0;

  if (cmd == "0100") { response = "41 00 98 38 00 01"; dataBytes = 6; hasData = true; }
  else if (cmd == "0120") { response = "41 20 00 00 00 01"; dataBytes = 6; hasData = true; }
  else if (cmd == "0140") { response = "41 40 40 00 00 00"; dataBytes = 6; hasData = true; }
  else if (cmd == "0101") { response = "41 01 00 07 E0 00"; dataBytes = 6; hasData = true; }
  
  else if (cmd == "010C") { response = "41 0C " + byteToHex(raw_rpm_A) + " " + byteToHex(raw_rpm_B); dataBytes = 4; hasData = true; }
  else if (cmd == "0104") { response = "41 04 " + byteToHex(raw_load); dataBytes = 3; hasData = true; }
  else if (cmd == "0105") { response = "41 05 " + byteToHex(raw_eng); dataBytes = 3; hasData = true; }
  else if (cmd == "010B") { response = "41 0B " + byteToHex(raw_map); dataBytes = 3; hasData = true; }
  else if (cmd == "010D") { response = "41 0D " + byteToHex(raw_speed); dataBytes = 3; hasData = true; }
  else if (cmd == "0142") { response = "41 42 " + byteToHex(raw_volt_A) + " " + byteToHex(raw_volt_B); dataBytes = 4; hasData = true; }

  if (hasData) {
    String finalMsg = "";
    if (elm_header) { finalMsg += "7E8 " + byteToHex(dataBytes) + " "; }
    finalMsg += response;
    if (!elm_space) finalMsg.replace(" ", "");
    Serial.print(finalMsg + "\r>");
  } else {
    if (elm_space) Serial.print("NO DATA\r>"); else Serial.print("NODATA\r>");
  }
}

void requestOBDPID(uint8_t pid) {
  twai_message_t tx_msg; tx_msg.identifier = 0x7DF; tx_msg.extd = 0; tx_msg.rtr = 0; tx_msg.data_length_code = 8;
  tx_msg.data[0] = 0x02; tx_msg.data[1] = 0x01; tx_msg.data[2] = pid;
  for(int i = 3; i < 8; i++) tx_msg.data[i] = 0x55;
  twai_transmit(&tx_msg, pdMS_TO_TICKS(5));
}

void setup() {
 Serial.begin(38400); // Baud rate standar ELM327 USB
  
  // Perbaikan Inisialisasi Buzzer Active LOW via Trik Hi-Z
  pinMode(PIN_BUZZER, INPUT);

  tft.init(); tft.setRotation(1); tft.fillScreen(TFT_BLACK);
  
  tft.setTextColor(TFT_WHITE, TFT_BLACK); tft.setTextSize(2);
  tft.drawString("DASHBOARD SIM CAN-BUS CATERPILLAR 950L", 12, 8);
  
  for(int i=0; i<7; i++) tft.drawLine(0, Y_LINE[i], 480, Y_LINE[i], TFT_DARKGREY);
  tft.drawLine(VLINE_1, 30, VLINE_1, 294, TFT_DARKGREY);
  tft.drawLine(VLINE_2, 30, VLINE_2, 294, TFT_DARKGREY);
  tft.drawLine(VLINE_3, 30, VLINE_3, 294, TFT_DARKGREY);

  tft.setTextSize(2); tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("RPM", 5, Y_ROW[0]);      tft.setTextColor(TFT_WHITE); tft.drawString("REV",  VLINE_2 - 45, Y_ROW[0]); tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("ENG LOAD", 5, Y_ROW[1]); tft.setTextColor(TFT_WHITE); tft.drawString("%",    VLINE_2 - 35, Y_ROW[1]); tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("ENG TMP", 5, Y_ROW[2]);  tft.setTextColor(TFT_WHITE); tft.drawString("C",    VLINE_2 - 35, Y_ROW[2]); tft.drawCircle(VLINE_2 - 42, Y_ROW[2] + 2, 2, TFT_WHITE); tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("MAP BST", 5, Y_ROW[3]);  tft.setTextColor(TFT_WHITE); tft.drawString("kPa",  VLINE_2 - 45, Y_ROW[3]); tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("SPEED", 5, Y_ROW[4]);    tft.setTextColor(TFT_WHITE); tft.drawString("KM/H", VLINE_2 - 60, Y_ROW[4]); tft.setTextColor(TFT_LIGHTGREY);
  tft.drawString("SYS VOLT", 5, Y_ROW[5]); tft.setTextColor(TFT_WHITE); tft.drawString("V",    VLINE_2 - 35, Y_ROW[5]);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_driver_install(&g_config, &t_config, &f_config); twai_start();
}

void loop() {
  // Perbaikan 3: Pemisahan Proses Serial dan Render TFT
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (serialBuffer.length() > 0) {
        processELMCommand(serialBuffer); // Proses data SECEPAT MUNGKIN
        lastCmdToDraw = serialBuffer;    // Simpan ke memori untuk di-render nanti
        needToDrawCmd = true;            // Angkat bendera bahwa ada data perlu digambar
        serialBuffer = "";
      } else if (c == '\r') {
        Serial.print("\r>"); // Keep-alive prompt untuk enter kosong
      }
    } else {
      serialBuffer += c;
    }
  }

  // Menggambar ke TFT hanya JIKA tidak sedang memproses Serial
  if (needToDrawCmd) {
    tft.fillRect(5, 302, 250, 15, TFT_BLACK);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK); tft.setTextSize(1);
    tft.drawString("CMD: " + lastCmdToDraw, 5, 305);
    needToDrawCmd = false;
  }

  bool forceIconUpdate = false;
  if (millis() - blinkTimer > 500) { blinkTimer = millis(); blinkState = !blinkState; forceIconUpdate = true; }
  if (millis() - lastRequestTime > 50) { requestOBDPID(pids[pidIndex]); pidIndex++; if (pidIndex >= 6) pidIndex = 0; lastRequestTime = millis(); }

  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, 0) == ESP_OK) {
    if (rx_msg.identifier == 0x7E8 && rx_msg.data[1] == 0x41) {
      uint8_t pid = rx_msg.data[2]; tft.setTextSize(3);

      switch (pid) {
        case 0x0C: { // RPM
          raw_rpm_A = rx_msg.data[3]; raw_rpm_B = rx_msg.data[4];
          uint16_t rpm_val = ((raw_rpm_A * 256) + raw_rpm_B) / 4;
          current_rpm_state = getNormalState(rpm_val, 2200, 2400); 
          if(rpm_val != last_rpm) {
            tft.fillRect(VLINE_1 + 2, Y_ROW[0], 75, 25, TFT_BLACK);
            tft.setTextColor(stateToColor(current_rpm_state), TFT_BLACK); tft.drawNumber(rpm_val, VLINE_1 + 5, Y_ROW[0]);
            drawDynamicStairBar(VLINE_2 + 8, Y_ROW[0] - 2, rpm_val, 3000, 2200, 2400);
            last_rpm = rpm_val; forceIconUpdate = true;
          } break;
        }
        case 0x04: { // LOAD
          raw_load = rx_msg.data[3]; uint8_t load_pct = (raw_load * 100) / 255;
          current_load_state = getNormalState(load_pct, 95, 100); 
          if(load_pct != last_load) {
            tft.fillRect(VLINE_1 + 2, Y_ROW[1], 75, 25, TFT_BLACK);
            tft.setTextColor(stateToColor(current_load_state), TFT_BLACK); tft.drawNumber(load_pct, VLINE_1 + 5, Y_ROW[1]);
            drawDynamicStairBar(VLINE_2 + 8, Y_ROW[1] - 2, load_pct, 100, 95, 100);
            last_load = load_pct; forceIconUpdate = true;
          } break;
        }
        case 0x05: { // TEMP
          raw_eng = rx_msg.data[3]; uint8_t eng_c = raw_eng - 40;
          // Logika state tetap 105 agar warna angka suhu akurat
          current_eng_state = getTempState(eng_c, 71, 105, 108); 
          if(eng_c != last_eng) {
            tft.fillRect(VLINE_1 + 2, Y_ROW[2], 75, 25, TFT_BLACK);
            tft.setTextColor(stateToColor(current_eng_state), TFT_BLACK); tft.drawNumber(eng_c, VLINE_1 + 5, Y_ROW[2]);
            
            // Perbaikan Resolusi Visual: Nilai warning visual diturunkan ke 102 
            // agar tepat mengenai titik tengah segmen matematika TFT.
            drawDynamicStairBar(VLINE_2 + 8, Y_ROW[2] - 2, eng_c, 120, 102, 108, 71); 
            
            last_eng = eng_c; forceIconUpdate = true;
          } break;
        }
        case 0x0B: { // MAP BOOST
          raw_map = rx_msg.data[3]; 
          current_map_state = getNormalState(raw_map, 165, 200); 
          if(raw_map != last_map) {
            tft.fillRect(VLINE_1 + 2, Y_ROW[3], 75, 25, TFT_BLACK);
            tft.setTextColor(stateToColor(current_map_state), TFT_BLACK); tft.drawNumber(raw_map, VLINE_1 + 5, Y_ROW[3]);
            drawDynamicStairBar(VLINE_2 + 8, Y_ROW[3] - 2, raw_map, 255, 165, 200);
            last_map = raw_map; forceIconUpdate = true;
          } break;
        }
        case 0x0D: { // SPEED
          raw_speed = rx_msg.data[3]; 
          current_speed_state = getNormalState(raw_speed, 39, 42); 
          if(raw_speed != last_speed) {
            tft.fillRect(VLINE_1 + 2, Y_ROW[4], 75, 25, TFT_BLACK);
            tft.setTextColor(stateToColor(current_speed_state), TFT_BLACK); tft.drawNumber(raw_speed, VLINE_1 + 5, Y_ROW[4]);
            drawDynamicStairBar(VLINE_2 + 8, Y_ROW[4] - 2, raw_speed, 60, 39, 42);
            last_speed = raw_speed; forceIconUpdate = true;
          } break;
        }
        case 0x42: { // VOLTAGE
          raw_volt_A = rx_msg.data[3]; raw_volt_B = rx_msg.data[4];
          float volt_val = ((raw_volt_A * 256.0) + raw_volt_B) / 1000.0;
          current_volt_state = getVoltState(volt_val); 
          
          if(abs(volt_val - last_volt) > 0.05) {
            tft.fillRect(VLINE_1 + 2, Y_ROW[5], 75, 25, TFT_BLACK);
            tft.setTextColor(stateToColor(current_volt_state), TFT_BLACK);
            tft.drawFloat(volt_val, 1, VLINE_1 + 5, Y_ROW[5]);
            
            drawVoltStairBar(VLINE_2 + 8, Y_ROW[5] - 2, volt_val);
            last_volt = volt_val; forceIconUpdate = true;
          } break;
        }
      }
    }
  }

  if (forceIconUpdate) {
    drawAlertIcon(0, current_rpm_state); drawAlertIcon(1, current_load_state); drawAlertIcon(2, current_eng_state);
    drawAlertIcon(3, current_map_state); drawAlertIcon(4, current_speed_state); drawAlertIcon(5, current_volt_state);
  }

  bool alarm_red = false; bool warning_orange = false;
  if (current_rpm_state == 3 || current_load_state == 3 || current_eng_state == 3 || current_map_state == 3 || current_speed_state == 3 || current_volt_state == 3) {
    alarm_red = true;
  } else if (current_rpm_state == 2 || current_load_state == 2 || current_eng_state == 2 || current_map_state == 2 || current_speed_state == 2 || current_volt_state == 2) {
    warning_orange = true;
  }

  // ==========================================================
  // PERBAIKAN FINAL: LOGIKA BUZZER ACTIVE LOW (LOW LEVEL TRIGGER)
  // ==========================================================
  if (alarm_red) {
    // Failure: Bunyi terus-menerus
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW); 
  } 
  else if (warning_orange) {
    // Warning: Bunyi putus-putus mengikuti kedipan layar
    if (blinkState) {
      pinMode(PIN_BUZZER, OUTPUT);
      digitalWrite(PIN_BUZZER, LOW); // Menyala saat blinkState TRUE
    } else {
      pinMode(PIN_BUZZER, INPUT);   // Mati total saat blinkState FALSE
    }
  } 
  else {
    // Normal: Pastikan mati total dan tidak menguras arus
    pinMode(PIN_BUZZER, INPUT); 
  }
}