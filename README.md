# ESP32-S3 RTOS — Percobaan Peripheral (2 Core) — Wokwi Simulator

Proyek ini menggunakan **FreeRTOS multitasking** pada **ESP32-S3** untuk mengendalikan berbagai peripheral secara paralel.  
Simulasi dilakukan menggunakan **Wokwi Simulator** dan dikembangkan dengan **PlatformIO (VSCode)**.

---

# Perangkat yang digunakan
- LED
- Push button
- Buzzer
- OLED (I2C, 128x64)
- Potensiometer
- Rotary encoder
- Motor stepper
- Servo motor

---

# Diagram Simulasi

<img src="assets/Screenshot 2025-11-11 015944.png" width="500">

*Koneksi pin ESP32-S3:*
- LED1 → GPIO2  
- LED2 → GPIO4  
- LED3 → GPIO5  
- BUTTON1 → GPIO0 (INPUT_PULLUP)  
- BUTTON2 → GPIO15 (INPUT_PULLUP)  
- BUZZER (PWM) → GPIO13  
- OLED (I2C) → SDA = GPIO21, SCL = GPIO22  
- POT (ADC) → GPIO36 (ADC1 channel)  
- ENCODER A → GPIO18  
- ENCODER B → GPIO19  
- ENCODER BTN → GPIO23  
- STEPPER STEP → GPIO26  
- STEPPER DIR → GPIO25  
- STEPPER ENABLE → GPIO27  
- SERVO (PWM) → GPIO14  

---

# Pembagian Tugas FreeRTOS antar Core

Berikut contoh pembagian task antar core ESP32-S3:

| Core | Tugas utama | Deskripsi |
|------|--------------|-----------|
| **Core 0** | Input & Komunikasi | Membaca tombol, rotary encoder, potensiometer, dan komunikasi I2C ke OLED |
| **Core 1** | Output & Kontrol Aktuator | Mengendalikan LED, Buzzer, Servo, dan Stepper |


Hasil Video Demo
[[<video src="assets/Screen Recording 2025-11-11.mp4" width="500" controls></video>](https://github.com/user-attachments/assets/42b3ebd8-0092-445e-a724-a47d020f65a6)](https://github.com/user-attachments/assets/301858d7-ffb5-41cc-b437-a77c131aa282)

(file: `assets/Screen Recording 2025-11-11.mp4`)


Simulasi dijalankan di Wokwi menggunakan PlatformIO pada VSCode

Board: ESP32-S3
