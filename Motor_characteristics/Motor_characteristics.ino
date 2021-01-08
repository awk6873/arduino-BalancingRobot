// Управляющая программа для снятия зависимости кол-ва оборотов моторов в секунду от PWM
// Основной контроллер - Arduino Uno
// Плата управления моторами - Ardumoto L298
// Контроллер энкодеров - ATTiny85
//
// Дата последнего изменения 28.08.2020

#include <Wire.h>

// распиновка портов
// управление моторами
#define MOTOR_PWM_R 3
#define MOTOR_PWM_L 11
#define MOTOR_DIR_R 12
#define MOTOR_DIR_L 13

// адреса I2C
// контроллер энкодеров
#define ENCODER_I2C_ADDR 0x73

// константы
// направления вращения моторов
#define MOTOR_FORWARD_R LOW
#define MOTOR_BACKWARD_R HIGH
#define MOTOR_FORWARD_L HIGH
#define MOTOR_BACKWARD_L LOW

// датчик напряжения батареи
#define VBAT_ADC A0
// коэффициент пересчета значения ADC VBAT в В * 100
#define VBAT_FACTOR 2.0755

#define SERIAL_BAUD_RATE 115200

// полиномы для линеаризации зависимости RPM моторов от PWM
#define motorLinearizeR(x) x
#define motorLinearizeL(x) x

int16_t VBat;            // напряжение батареи, В * 100

// функция чтения данных из контроллера энкодеров
int Encoder_read(int numBytes, byte data[]) {
  int i = 0, r;

  Wire.requestFrom(ENCODER_I2C_ADDR, numBytes);  // устанавливаем кол-во читаемых регистров
  int numBytesAvailable = Wire.available();
  for (i = 0; i < numBytesAvailable; i++) {
    data[i] = Wire.read();
  }
  if (numBytes != numBytesAvailable) {
    Serial.print("Bytes read: ");
    Serial.println(numBytesAvailable);
    return (-2);
  }
  return (0);
}

void setup() {
  int rc;
  uint8_t data[10];
 
  // инициализация UART и I2C
  Serial.begin(SERIAL_BAUD_RATE);
  Wire.begin();
  
  // инициализация портов
  // управление моторами
  pinMode(MOTOR_PWM_R, OUTPUT);
  pinMode(MOTOR_PWM_L, OUTPUT);
  pinMode(MOTOR_DIR_R, OUTPUT);
  pinMode(MOTOR_DIR_L, OUTPUT);

  // датчик напряжения батареи
  pinMode(VBAT_ADC, INPUT);
  
  // проверяем контроллер энкодеров
  rc = Encoder_read(10, data);
  if (rc != 0) {
    Serial.print("Encoder controller error: ");
    Serial.println(rc);
   // while(1);
  }
}

// съем зависимости RPM от PWM
void loop() {
  int rc, err, motorPWM, motorPWM_R, motorPWM_L;
  byte data[10];
  int32_t encLstart, encRstart, encL, encR;
  int32_t motorRPM_R, motorRPM_L;

  // устанавливаем направление моторов "вперед"
  digitalWrite(MOTOR_DIR_R, MOTOR_FORWARD_R);
  digitalWrite(MOTOR_DIR_L, MOTOR_FORWARD_L);

  // цикл с увеличением ШИМ с шагом 5 до макс.значения
  for (motorPWM = 0; motorPWM <= 255; motorPWM += 5) {
    // устанавливаем значение ШИМ    
    analogWrite(MOTOR_PWM_R, motorPWM);
    analogWrite(MOTOR_PWM_L, motorPWM);

    // считываем текущие значения энкодеров
    do {
      rc = Encoder_read(10, data);
      if (rc != 0) {
        Serial.print("Encoder controller error: ");
        Serial.println(rc);
        delay(1);
      }
    } while (rc != 0);

    encRstart = data[0];
    encRstart = (encRstart << 8) | data[1];
    encRstart = (encRstart << 8) | data[2];
    encRstart = (encRstart << 8) | data[3];
    encLstart = data[4];
    encLstart = (encLstart << 8) | data[5];
    encLstart = (encLstart << 8) | data[6];
    encLstart = (encLstart << 8) | data[7];
    err = data[8];
    err = (err << 8) | data[9];
    
    // ждем 1 с
    delay(1000);
    
    // считываем значения энкодеров
    do {
      rc = Encoder_read(10, data);
      if (rc != 0) {
        Serial.print("Encoder controller error: ");
        Serial.println(rc);
        delay(1);
      }
    } while (rc != 0);
    encR = data[0];
    encR = (encR << 8) | data[1];
    encR = (encR << 8) | data[2];
    encR = (encR << 8) | data[3];
    encL = data[4];
    encL = (encL << 8) | data[5];
    encL = (encL << 8) | data[6];
    encL = (encL << 8) | data[7];
    err = data[8];
    err = (err << 8) | data[9];

    // вычисляем скорость вращения моторов в импульсах энкодера за секунду
    motorRPM_R = (encR - encRstart);
    motorRPM_L = (encL - encLstart);

    // датчик напряжения батареи
    VBat = analogRead(VBAT_ADC) * VBAT_FACTOR;
    
    // вывод значений
    Serial.print(motorPWM);Serial.print("\t");
    Serial.print(motorRPM_R);Serial.print("\t");
    Serial.print(motorRPM_L);Serial.print("\t");
    Serial.print(encR);Serial.print("\t");
    Serial.print(encL);Serial.print("\t");
    Serial.print(err);Serial.print("\t");
    //Serial.print(VBat);Serial.print("\t");
      
    Serial.print("\n");
  }
  // выключаем моторы
  analogWrite(MOTOR_PWM_R, 0);
  analogWrite(MOTOR_PWM_L, 0);
  // пауза 
  delay(3000);
}


