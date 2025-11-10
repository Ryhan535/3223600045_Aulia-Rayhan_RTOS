#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ESP32Servo.h>

#define LED1_PIN   2
#define LED2_PIN   3  
#define LED3_PIN   4

#define BTN_PIN    5

#define BUZZER_PIN 7

#define I2C_SDA    42
#define I2C_SCL    41
#define OLED_ADDR  0x3C
Adafruit_SSD1306 display(128, 64, &Wire);

#define POT_PIN    1

#define ENC_A_PIN  8
#define ENC_B_PIN  9
#define ENC_SW_PIN 10

#define ST_A_N     11
#define ST_A_P     12
#define ST_B_P     13
#define ST_B_N     14

#define SERVO_PIN  15

Servo servo1;

volatile long encCount = 0;
volatile uint8_t lastA = 0, lastB = 0;
volatile bool encBtnPressed = false;

TaskHandle_t taskLEDsHandle;
TaskHandle_t taskBtnBuzzHandle;
TaskHandle_t taskServoOLEDHandle;
TaskHandle_t taskStepperHandle;

void setStepperCoils(uint8_t a_n, uint8_t a_p, uint8_t b_p, uint8_t b_n) {
  digitalWrite(ST_A_N, a_n);
  digitalWrite(ST_A_P, a_p);
  digitalWrite(ST_B_P, b_p);
  digitalWrite(ST_B_N, b_n);
}

const uint8_t STEP_TABLE[8][4] = {
  {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
  {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};

// Buzzer control
const int BUZZR_CH = 2;
void beepStart(int freq) {
  ledcAttachPin(BUZZER_PIN, BUZZR_CH);
  ledcWriteTone(BUZZR_CH, freq);
}
void beepStop() {
  ledcWrite(BUZZR_CH, 0);
  ledcDetachPin(BUZZER_PIN);
}

// Encoder interrupt handlers
void IRAM_ATTR encISR_A() {
  uint8_t a = digitalRead(ENC_A_PIN);
  uint8_t b = digitalRead(ENC_B_PIN);
  if (a != lastA) {
    (a == b) ? encCount++ : encCount--;
    lastA = a;
    lastB = b;
  }
}
void IRAM_ATTR encISR_B() {
  uint8_t a = digitalRead(ENC_A_PIN);
  uint8_t b = digitalRead(ENC_B_PIN);
  if (b != lastB) {
    (a != b) ? encCount++ : encCount--;
    lastA = a;
    lastB = b;
  }
}
void IRAM_ATTR encBtnISR() {
  if (digitalRead(ENC_SW_PIN) == LOW) encBtnPressed = true;
}

//  RTOS TASKS 
void taskLEDs(void* pv) {
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);

  for (;;) {
    //Blink
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, HIGH);
    digitalWrite(LED3_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(500)); // 500ms ON
    
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, LOW);
    digitalWrite(LED3_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(500)); // 500ms OFF
  }
}

// Task Button & Buzzer
void taskButtonsBuzzer(void* pv) {
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);

  bool buzOn = false;
  for (;;) {
    // Single Button: toggle buzzer
    if (digitalRead(BTN_PIN) == LOW) {
      vTaskDelay(pdMS_TO_TICKS(20)); // debounce
      if (digitalRead(BTN_PIN) == LOW) {
        buzOn = !buzOn;
        if (buzOn) {
          beepStart(1760); // Frekuensi 1760Hz
        } else {
          beepStop();
        }
        while (digitalRead(BTN_PIN) == LOW) vTaskDelay(pdMS_TO_TICKS(10));
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Task Servo + OLED + Potentiometer pada Core 1
void taskServoOLED(void* pv) {
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  servo1.attach(SERVO_PIN);

  for (;;) {
    int raw = analogRead(POT_PIN);
    int angle = map(raw, 0, 4095, 0, 180);
    servo1.write(angle);

    long enc = encCount;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(F("ESP32-S3 RTOS Demo"));
    display.print(F("Pot: ")); display.print(raw);
    display.print(F("  Ang: ")); display.println(angle);
    display.print(F("Enc: ")); display.println(enc);
    display.print(F("Dir Btn: "));
    display.println(encBtnPressed ? "TOGGLE" : "-");
    display.display();

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// Task Stepper Motor pada Core 1
void taskStepper(void* pv) {
  pinMode(ST_A_N, OUTPUT);
  pinMode(ST_A_P, OUTPUT);
  pinMode(ST_B_P, OUTPUT);
  pinMode(ST_B_N, OUTPUT);

  int stepIndex = 0;
  bool dirCW = true;

  for (;;) {
    if (encBtnPressed) {
      dirCW = !dirCW;
      encBtnPressed = false;
    }

    long speedSteps = abs(encCount); 
    speedSteps = constrain(speedSteps, 0, 400);
    uint16_t delayMs = 4 + (400 - speedSteps) / 2;

    const uint8_t* s = STEP_TABLE[stepIndex];
    setStepperCoils(s[0], s[1], s[2], s[3]);

    stepIndex = (dirCW) ? (stepIndex + 1) : (stepIndex + 7);
    stepIndex &= 7;

    vTaskDelay(pdMS_TO_TICKS(delayMs));
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(200);

  // Encoder setup dengan interrupt
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  pinMode(ENC_SW_PIN, INPUT_PULLUP);
  lastA = digitalRead(ENC_A_PIN);
  lastB = digitalRead(ENC_B_PIN);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), encISR_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B_PIN), encISR_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_SW_PIN), encBtnISR, FALLING);

  // Buat RTOS tasks
  xTaskCreatePinnedToCore(taskLEDs,        "LEDs",        4096, NULL, 1, &taskLEDsHandle,       0);
  xTaskCreatePinnedToCore(taskButtonsBuzzer,"ButtonBuzz",  4096, NULL, 2, &taskBtnBuzzHandle,    0);
  xTaskCreatePinnedToCore(taskServoOLED,   "ServoOLED",   6144, NULL, 1, &taskServoOLEDHandle,   1);
  xTaskCreatePinnedToCore(taskStepper,     "Stepper",     4096, NULL, 2, &taskStepperHandle,     1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}