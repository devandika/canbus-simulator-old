#include "driver/twai.h"

#define PIN_RPM     32
#define PIN_LOAD    33
#define PIN_ENG_TMP 34
#define PIN_MAP     35
#define PIN_SPEED   25
#define PIN_VOLT    26

#define TX_PIN 21
#define RX_PIN 22

void setup() {
  Serial.begin(115200);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  twai_driver_install(&g_config, &t_config, &f_config);
  twai_start();
  Serial.println("ECU Simulator CAT 950L Aktif.");
}

void loop() {
  twai_message_t rx_msg;
  if (twai_receive(&rx_msg, 0) == ESP_OK) {
    if (rx_msg.identifier == 0x7DF && rx_msg.data[1] == 0x01) {
      sendOBDResponse(rx_msg.data[2]);
    }
  }
  delay(1);
}

void sendOBDResponse(uint8_t pid) {
  twai_message_t tx_msg;
  tx_msg.identifier = 0x7E8; tx_msg.extd = 0; tx_msg.rtr = 0; tx_msg.data_length_code = 8;
  tx_msg.data[1] = 0x41; tx_msg.data[2] = pid;
  for(int i = 3; i < 8; i++) tx_msg.data[i] = 0x55;

  switch (pid) {
    case 0x0C: { // RPM (0 - 3000)
      int valRPM = analogRead(PIN_RPM);
      uint32_t obd_rpm = map(valRPM, 0, 4095, 0, 3000) * 4;
      tx_msg.data[0] = 4; tx_msg.data[3] = (obd_rpm >> 8) & 0xFF; tx_msg.data[4] = obd_rpm & 0xFF;        
      break;
    }
    case 0x04: { // LOAD (0 - 100%)
      int valLoad = analogRead(PIN_LOAD);
      tx_msg.data[0] = 3; tx_msg.data[3] = map(valLoad, 0, 4095, 0, 255);
      break;
    }
    case 0x05: { // ENG TEMP (Simulasi 0 - 120°C)
      int valEng = analogRead(PIN_ENG_TMP);
      tx_msg.data[0] = 3; tx_msg.data[3] = map(valEng, 0, 4095, 0, 120) + 40;
      break;
    }
    case 0x0B: { // MAP BOOST (0 - 255 kPa) - Diturunkan ke 0 untuk mensimulasikan KOEO
      int valMap = analogRead(PIN_MAP);
      tx_msg.data[0] = 3; tx_msg.data[3] = map(valMap, 0, 4095, 0, 255);
      break;
    }
    case 0x0D: { // SPEED (0 - 60 KM/H) - Dinaikkan agar bisa masuk ke kondisi failure > 42 km/h
      int valSpeed = analogRead(PIN_SPEED);
      tx_msg.data[0] = 3; tx_msg.data[3] = map(valSpeed, 0, 4095, 0, 60);
      break;
    }
    case 0x42: { // VOLTAGE (0V - 35.0V) - Dinaikkan agar bisa masuk ke kondisi failure > 32V
      int valVolt = analogRead(PIN_VOLT);
      uint16_t volt_mv = map(valVolt, 0, 4095, 0, 35000);
      tx_msg.data[0] = 4; tx_msg.data[3] = (volt_mv >> 8) & 0xFF; tx_msg.data[4] = volt_mv & 0xFF;        
      break;
    }
    default: return;
  }
  twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
}