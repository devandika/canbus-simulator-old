# Simulator Dual-Node CAN-Bus ECU Caterpillar 950L

> **Judul Proyek:**  
> *"Rancang Bangun Simulator Dual-Node CAN-Bus ECU Caterpillar 950L Berbasis ESP32, Transceiver SN65HVD230, dan Gateway ELM327 Menggunakan Pengendali Tampilan TFT ILI9488 Terintegrasi."*

---

## 📌 Deskripsi Proyek
Proyek ini mengimplementasikan simulator sistem kontrol elektronik dan komunikasi bus data otomotif/alat berat standar CAN-Bus (ISO 11898 / ISO 15765-4 OBD-II) untuk mesin alat berat **Caterpillar 950L Wheel Loader**. 

Sistem menggunakan arsitektur **Dual-Node ESP32**:
1. **Node 1 (Transmitter / ECU Simulator):** Membaca 6 input potensiometer 10k sebagai representasi parameter operasional mesin, lalu mengemasnya menjadi respons frame CAN standar OBD-II (ID `0x7E8`).
2. **Node 2 (Receiver / Cluster Dashboard & Gateway):** Mengirim query PID berkala (ID `0x7DF`), mengurai data respons, menampilkan antarmuka visual dinamis pada layar **TFT 3.5" ILI9488**, memicu alarm multi-level pada **Active Buzzer 5V**, serta mengemulasi gateway **ELM327 v1.5** via Serial USB untuk integrasi perangkat lunak diagnostik PC (seperti OBD Auto Doctor).

---

## 🏗️ Arsitektur Jaringan & Aliran Data
![Arsitektur Jaringan](Arsitektur%20jaringan.png)

1. **Input Layer:** 6 unit potensiometer analog 10k menghasilkan variasi tegangan 0 - 3.3V ke pin ADC ESP32 Node 1.
2. **ECU Simulator:** ESP32 Node 1 mengonversi nilai ADC menjadi data parameter telemetri OBD-II standar.
3. **Physical Layer Transceiver:** Chip SN65HVD230 mentranslasikan level tegangan logika 3.3V (CTX/CRX) menjadi sinyal tegangan diferensial (`CAN_H` dan `CAN_L`).
4. **CAN Bus:** Bekerja pada kecepatan **500 kbps (TWAI ESP32)**.
5. **Receiver & Instrument Cluster:** ESP32 Node 2 memproses frame CAN, mengontrol display TFT SPI dan buzzer alarm.
6. **Diagnostic Layer:** 
   - Soket fisik **OBD-II 16-pin** untuk alat diagnostik eksternal.
   - Penganalisis bus USB-to-CAN untuk pemantauan paket raw melalui **SavvyCAN**.
   - Emulasi protokol AT-Command ELM327 via serial USB ke software **OBD Auto Doctor**.

---

## 🔌 Diagram Pengkabelan (Wiring Diagram)
![Wiring Diagram](Wiring%20diagram.jpeg)

### Pemetaan Pin Node 1 (ECU Simulator):
| Komponen / Sinyal | Pin ESP32 | Keterangan |
| :--- | :--- | :--- |
| Potensiometer RPM | GPIO 32 (ADC) | Engine RPM (0 - 3000 RPM) |
| Potensiometer Load | GPIO 33 (ADC) | Engine Load (0 - 100 %) |
| Potensiometer Suhu | GPIO 34 (ADC) | Engine Temperature (0 - 120 °C) |
| Potensiometer MAP | GPIO 35 (ADC) | Manifold Absolute Pressure (0 - 255 kPa) |
| Potensiometer Speed | GPIO 25 (ADC) | Vehicle Speed (0 - 60 km/h) |
| Potensiometer Volt | GPIO 26 (ADC) | System Voltage (0.0 - 35.0 V) |
| SN65HVD230 CTX | GPIO 21 (TWAI TX) | Jalur Transmit CAN |
| SN65HVD230 CRX | GPIO 22 (TWAI RX) | Jalur Receive CAN |

### Pemetaan Pin Node 2 (Receiver & Dashboard):
| Komponen / Sinyal | Pin ESP32 | Keterangan |
| :--- | :--- | :--- |
| SN65HVD230 CTX | GPIO 21 (TWAI TX) | Jalur Transmit CAN |
| SN65HVD230 CRX | GPIO 22 (TWAI RX) | Jalur Receive CAN |
| TFT 3.5" ILI9488 | Hardware SPI | CS, DC, RST, SCK, MOSI, MISO |
| Active Buzzer 5V | GPIO 13 | Kontrol Active-LOW (Trik Hi-Z) |
| Serial USB | TX0 / RX0 | 38400 bps (Emulasi ELM327 USB) |

---

## 📊 Parameter Telemetri & Ambang Batas (Thresholds)
| Parameter | PID OBD-II | Skala | Kondisi Normal | Peringatan (Warning) | Bahaya (Alarm/Failure) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Engine RPM** | `0x0C` | 0 - 3000 RPM | < 2200 RPM | $\ge$ 2200 RPM | $\ge$ 2400 RPM |
| **Engine Load** | `0x04` | 0 - 100 % | < 95 % | $\ge$ 95 % | $\ge$ 100 % |
| **Coolant Temp** | `0x05` | 0 - 120 °C | 71 - 104 °C | $\ge$ 105 °C (Cold: <71°C) | $\ge$ 108 °C |
| **MAP Boost** | `0x0B` | 0 - 255 kPa | < 165 kPa | $\ge$ 165 kPa | $\ge$ 200 kPa |
| **Vehicle Speed**| `0x0D` | 0 - 60 km/h | < 39 km/h | $\ge$ 39 km/h | $\ge$ 42 km/h |
| **System Voltage**| `0x42` | 0 - 35.0 V | 23.0 - 29.4 V | < 23.0V atau $\ge$ 29.5V | < 18.0V atau $\ge$ 32.0V |

---

## 🔔 Logika Alarm & Indikator
- **Normal (Hijau):** Buzzer nonaktif (High-Z mode).
- **Warning (Oranye):** Ikon peringatan menyala oranye, buzzer berbunyi putus-putus (*intermittent beep*) setiap 500 ms sinkron dengan animasi layar.
- **Alarm / Critical (Merah):** Ikon peringatan berkedip merah/hitam, buzzer berbunyi panjang terus-menerus (*continuous beep*).

---

## 💻 Cara Menggunakan
1. Buka Arduino IDE, pastikan core **ESP32** terpasang.
2. Pasang library **TFT_eSPI** dan konfigurasikan `User_Setup.h` untuk driver **ILI9488 SPI**.
3. Upload `ProgramFinalNode1.ino` ke ESP32 Node 1.
4. Upload `ProgramFinalNode2Revision.ino` ke ESP32 Node 2.
5. Sambungkan kedua node melalui jalur `CAN_H` dan `CAN_L` pada transceiver SN65HVD230 (pastikan resistor terminasi 120 $\Omega$ aktif jika diperlukan).
6. Sambungkan kabel USB Node 2 ke PC pada baudrate **38400** untuk integrasi dengan software diagnostik **OBD Auto Doctor**.
