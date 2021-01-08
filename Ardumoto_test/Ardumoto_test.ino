#include <Wire.h>

// распиновка портов
// кнопка режима
#define BTN_MODE 2
// светик режима
#define LED_MODE 4
// светики наклона вперед и назад
#define LED_FRONT 5
#define LED_BACK 6
// делитель напряжения батареи
#define ADC_BATTERY A0
// потенциометры настройки коэффициентов PID
#define ADC_P A1
#define ADC_I A2
#define ADC_D A3

// адреса I2C
// акселерометр/гироскоп
#define ACC_GYRO_ADDR 0x68
// обработчик энкодеров
#define ENC_HANDLER_ADDR 0x73

// управление моторами
// пины управления
#define R_PWM 3
#define L_PWM 11
#define R_DIR 12
#define L_DIR 13
// направления моторов
#define R_FORW HIGH
#define R_BACK LOW
#define L_FORW LOW
#define L_BACK HIGH

byte bBtnMode;


void setup() {

  // устанавливаем режимы пинов  
  // кнопка режима
  pinMode(BTN_MODE, INPUT_PULLUP);
  // индикаторы режима и наклона
  pinMode(LED_MODE, OUTPUT);
  pinMode(LED_FRONT, OUTPUT);  
  pinMode(LED_BACK, OUTPUT);
  // управление моторами
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  
  // инициализируемся
  // включаем все светодиоды на 0.5 сек
  digitalWrite(LED_MODE, HIGH);
  digitalWrite(LED_FRONT, HIGH);
  digitalWrite(LED_BACK, HIGH);
  delay(500);
  digitalWrite(LED_MODE, LOW);
  digitalWrite(LED_FRONT, LOW);
  digitalWrite(LED_BACK, LOW);
  
  // проверяем состояние кнопки режима
  bBtnMode = digitalRead(BTN_MODE);
  
}

void loop() {
  
  // направление вперед
  digitalWrite(R_DIR, R_FORW);
  digitalWrite(L_DIR, L_FORW);
  
  for (int i = 0; i <= 255; i++) {
    analogWrite(R_PWM, i);
    analogWrite(L_PWM, i);
    delay(20);
  }  
  for (int i = 255; i >=0; i--) {
    analogWrite(R_PWM, i);
    analogWrite(L_PWM, i);
    delay(20);
  }
  
  // направление назад
  digitalWrite(R_DIR, R_BACK);
  digitalWrite(L_DIR, L_BACK);
  
  for (int i = 0; i <= 255; i++) {
    analogWrite(R_PWM, i);
    analogWrite(L_PWM, i);
    delay(20);
  }  
  for (int i = 255; i >=0; i--) {
    analogWrite(R_PWM, i);
    analogWrite(L_PWM, i);
    delay(20);
  }  
}
