// Управляющая программа для балансирующего робота
// Основной контроллер - Arduino Uno
// IMU (акселерометр/гироскоп) - MPU6050 (GY-521)
// Плата управления моторами - Ardumoto L298
// Контроллер энкодеров - ATTiny85
// Модуль Bluetooth - HC-06
//
// Дата последнего изменения 28.08.2020

// отладочные установки 
#define DEBUG       // выводить отладочные сообщения в Serial - пока не используется
//#define MOTOR_OFF   // не крутить моторы
#define PID_ADJUST  // подстройка коэффициентов PID потенциометрами

#include <Wire.h>
#include <EEPROM.h>

// коэффициенты для PID
double Kp = 200, Ki = 80, Kd = 1;

// фильтр Калмана
#include "kalman.h"
Kalman kalmanX;
Kalman kalmanY;

// распиновка портов
// кнопка режима
#define BUTTON_MODE 2
// потенциометры настройки PID
#define PID_ADJ_P A3  // P
#define PID_ADJ_I A2  // I
#define PID_ADJ_D A1  // D
// датчик напряжения батареи
#define VBAT_ADC A0 
// светодиоды наклона вперед и назад
#define LED_FORWARD 6
#define LED_BACKWARD 5
// светодиод режима
#define LED_MODE 4 
// управление моторами
#define MOTOR_PWM_R 3
#define MOTOR_PWM_L 11
#define MOTOR_DIR_R 12
#define MOTOR_DIR_L 13

// адреса I2C
// акселерометр/гироскоп
#define MPU6050_I2C_ADDR 0x68
// контроллер энкодеров
#define ENCODER_I2C_ADDR 0x73

// константы
// коэффициент пересчета значения ADC VBAT в В * 100
#define VBAT_FACTOR 2.0755
// макс.коэффициент подстройки PID (* или /) в крайних положениях потенциометров
#define PID_ADJ_FACTOR_MAX 5
// направления вращения моторов
#define MOTOR_FORWARD_R LOW
#define MOTOR_BACKWARD_R HIGH
#define MOTOR_FORWARD_L HIGH
#define MOTOR_BACKWARD_L LOW
// коэффициенты для уравнивания характеристик моторов 
#define MOTOR_FACTOR_R 1
#define MOTOR_FACTOR_L 1
// значение PWM, при котором моторы начинают вращаться
#define MOTOR_PWM_OFFSET 80
// макс.разброс значений гироскопа для определения неподвижного положения
#define MPU6050_GYRO_STAB_RANGE 100
// кол-во замеров для калибровки MPU6050
#define MPU6050_CALIBRATE_SAMPLES_NUM 50
// задержка между чтением из MPU6050 для калибровки, мс
#define MPU6050_CALIBRATE_DELAY 100

#define SERIAL_BAUD_RATE 115200

// текущие значения сенсоров
int8_t buttonModeStatus; // состояние кнопки режима
int16_t VBat;            // напряжение батареи, В * 100

// значения ADC потенциометров подстройки PID
int16_t adjPValue;       
int16_t adjIValue;
int16_t adjDValue;

// положения энкодеров и кол-во ошибок
int32_t encoderR, encoderL;
int16_t encoderErrors;

// "сырые" значения из IMU 
int16_t accX, accY, accZ;
int16_t gyroX, gyroY, gyroZ;
int16_t tempRaw;

// калибровочные "сырые" значения из IMU 
int16_t accX_base, accY_base, accZ_base;
int16_t gyroX_base, gyroY_base, gyroZ_base;

// вычисленные значения по IMU
double accXangle;      // углы по акселю
double accYangle;
double gyroXangle = 0; // углы по гиро
double gyroYangle = 0;
double compAngleX = 0; // углы через комплементарный фильр 
double compAngleY = 0; 
double kalAngleX;      // углы через фильтр Калмана
double kalAngleY;
double temp;           // температура IMU в град.C

// текущие значения коэфф.подстройки PID
double adjFactorKp, adjFactorKi, adjFactorKd;

// значения воздействия на моторы (PWM)
uint16_t motorPWMR;
uint16_t motorPWML;

// таймер для вычислений дельт
uint32_t timer;
// таймер для обработки энкодеров 100 мс
#define ENCODER_TIMER_DELAY 100
uint32_t encoder_timer;
// таймер вывода в Serial 200 мс
#define SERIAL_TIMER_DELAY 200
uint32_t serial_timer;

void setup() {
  uint8_t data[14];
  int rc;
 
  // инициализация UART и I2C
  Serial.begin(SERIAL_BAUD_RATE);
  Wire.begin();
  
  // инициализация портов
  // кнопка режима
  pinMode(BUTTON_MODE, INPUT_PULLUP);
  // светодиод режима
  pinMode(LED_MODE, OUTPUT);
  // светодиоды наклона вперед и назад
  pinMode(LED_FORWARD, OUTPUT);
  pinMode(LED_BACKWARD, OUTPUT);
  // потенциометры настройки PID
  pinMode(PID_ADJ_P, INPUT);
  pinMode(PID_ADJ_I, INPUT);
  pinMode(PID_ADJ_D, INPUT);
  // датчик напряжения батареи
  pinMode(VBAT_ADC, INPUT);
  // управление моторами
  pinMode(MOTOR_PWM_R, OUTPUT);
  pinMode(MOTOR_PWM_L, OUTPUT);
  pinMode(MOTOR_DIR_R, OUTPUT);
  pinMode(MOTOR_DIR_L, OUTPUT);

  // считываем состояние кнопки режима
  buttonModeStatus = digitalRead(BUTTON_MODE);
  
  // тест периферии
    
  // проверяем MPU6050
  rc = MPU6050_read(0x75, 1, data); // читаем регистр WHO_AM_I
  if (rc != 0) {
    Serial.print("MPU6050 error: ");
    Serial.println(rc);
    show_error_code(1);
    while(1);
  }
  if (data[0] != MPU6050_I2C_ADDR) {
    Serial.println("MPU6050 is not connected");
    show_error_code(1);
    while(1);
  }
  // отключаем режим SLEEP в MPU6050
  MPU6050_write_reg(0x6B, 0);
  // включаем ФНЧ (2 - 94 Гц, 3 - 44 Гб, 4 - 21 Гц, 5 - 10 Гц, 6 - 5 Гц)
  MPU6050_read(0x1A, 1, data);
  MPU6050_write_reg(0x1A, data[0] | 6);
  
  // проверяем контроллер энкодеров
  rc = Encoder_read(10, data);
  if (rc != 0) {
    Serial.print("Encoder controller error: ");
    Serial.println(rc);
    show_error_code(2);
    while(1);
  }

  // моргаем светиками 
  digitalWrite(LED_FORWARD, HIGH);
  delay(200);
  digitalWrite(LED_FORWARD, LOW);
  digitalWrite(LED_BACKWARD, HIGH);
  delay(200);
  digitalWrite(LED_BACKWARD, LOW);
  digitalWrite(LED_MODE, HIGH);
  delay(200);
  digitalWrite(LED_MODE, LOW);

  if (buttonModeStatus == 0)
    // включение было с нажатой кнопкой, переходим в режим калибровки
    MPU6050_calibrate();

  // считываем калибровочные значения из EEPROM
  Serial.println("Reading calibration values from EEPROM");
  accX_base = (EEPROM.read(0) << 8) | EEPROM.read(1);
  accY_base = (EEPROM.read(2) << 8) | EEPROM.read(3);
  accZ_base = (EEPROM.read(4) << 8) | EEPROM.read(5);
  gyroX_base = (EEPROM.read(6) << 8) | EEPROM.read(7);
  gyroY_base = (EEPROM.read(8) << 8) | EEPROM.read(9);
  gyroZ_base = (EEPROM.read(10) << 8) | EEPROM.read(11);
  Serial.print(accX_base);Serial.print('\t');
  Serial.print(accY_base);Serial.print('\t');
  Serial.print(accZ_base);Serial.print('\t');
  Serial.print(gyroX_base);Serial.print('\t');
  Serial.print(gyroY_base);Serial.print('\t');
  Serial.print(gyroZ_base);Serial.print('\t');
  Serial.println();

  // установка начальных значений фильтра Калмана 
  kalmanX.setAngle(0);
  kalmanY.setAngle(0);
  
  // заcекаем начальную отметку времени для таймеров
  timer = micros();
  serial_timer = millis();
  encoder_timer = serial_timer;
}

void loop() {
  int rc;
  uint8_t data[14];
  double controlAngleY = 0, controlAngleYintegr = 0;
  
  // считываем значения датчиков

  // датчик напряжения батареи
  VBat = analogRead(VBAT_ADC) * VBAT_FACTOR;

  #ifdef PID_ADJUST
  // положение потенциометров подстройки PID
  adjPValue = analogRead(PID_ADJ_P) - 511;
  adjIValue = analogRead(PID_ADJ_I) - 511;
  adjDValue = analogRead(PID_ADJ_D) - 511;

  // вычисляем коэфф.подстройки PID
  adjFactorKp = 1.0 + ((PID_ADJ_FACTOR_MAX - 1) * abs(adjPValue) / 512.0);
  if (adjPValue < 0) adjFactorKp = 1.0 / adjFactorKp;
  adjFactorKi = 1.0 + ((PID_ADJ_FACTOR_MAX - 1) * abs(adjIValue) / 512.0);
  if (adjIValue < 0) adjFactorKi = 1.0 / adjFactorKi;
  adjFactorKd = 1.0 + ((PID_ADJ_FACTOR_MAX - 1) * abs(adjDValue) / 512.0);
  if (adjDValue < 0) adjFactorKd = 1.0 / adjFactorKd;
   
  /*
  Serial.print(adjFactorKp);Serial.print("\t");
  Serial.print(adjFactorKi);Serial.print("\t");
  Serial.print(adjFactorKd);Serial.print("\t");
  Serial.println();
  */
  #endif

  // получаем текущие значения энкодеров каждые N мс
  if (millis() - encoder_timer > ENCODER_TIMER_DELAY) {
    encoder_timer = millis();

    if (Encoder_read(10, data)) {
      // если не было ошибки чтения
      encoderR = data[0];
      encoderR = (encoderR << 8) | data[1];
      encoderR = (encoderR << 8) | data[2];
      encoderR = (encoderR << 8) | data[3];
      encoderL = data[4];
      encoderL = (encoderL << 8) | data[5];
      encoderL = (encoderL << 8) | data[6];
      encoderL = (encoderL << 8) | data[7];
      encoderErrors = data[8];
      encoderErrors = (encoderErrors << 8) | data[9];
    }
  }

  // получаем значения из MPU6050
  MPU6050_read(0x3B, 14, data);
  accX = ((data[0] << 8) | data[1]);
  accY = ((data[2] << 8) | data[3]);
  accZ = ((data[4] << 8) | data[5]);  
  tempRaw = ((data[6] << 8) | data[7]);  
  gyroX = ((data[8] << 8) | data[9]);
  gyroY = ((data[10] << 8) | data[11]);
  gyroZ = ((data[12] << 8) | data[13]);   

  // вычитаем калибровочные значения, аксель по Z не компенсируем
  accX -= accX_base;
  accY -= accY_base;
  gyroX -= gyroX_base;
  gyroY -= gyroY_base;
  gyroZ -= gyroZ_base;

  // вычисляем углы по акселю
  //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
  accYangle = atan(-1*accX/sqrt(pow(accY,2) + pow(accZ,2)))*RAD_TO_DEG;
  accXangle = atan(accY/sqrt(pow(accX,2) + pow(accZ,2)))*RAD_TO_DEG;

  // вычисляем углы по гиро
  // угловые скорости по гиро 
  double gyroXrate = (double)gyroX/131.0;
  double gyroYrate = (double)gyroY/131.0;
  // вариант вычисления углов по гиро без фильтра 
  gyroXangle += gyroXrate*((double)(micros()-timer)/1000000); 
  gyroYangle += gyroYrate*((double)(micros()-timer)/1000000);
  // вариант вычисления углов по гиро через unbiased rate
  //gyroXangle += kalmanX.getRate()*((double)(micros()-timer)/1000000);
  //gyroYangle += kalmanY.getRate()*((double)(micros()-timer)/1000000);

  // вычисляем общие значения углов через комплементарный фильтр
  compAngleX = (0.93*(compAngleX+(gyroXrate*(double)(micros()-timer)/1000000)))+(0.07*accXangle);
  compAngleY = (0.93*(compAngleY+(gyroYrate*(double)(micros()-timer)/1000000)))+(0.07*accYangle);  

  // вычисляем общие значения углов через фильтр Калмана
  kalAngleX = kalmanX.getAngle(accXangle, gyroXrate, (double)(micros()-timer)/1000000); 
  kalAngleY = kalmanY.getAngle(accYangle, gyroYrate, (double)(micros()-timer)/1000000);

  // используемое для управления положением по углу Y значение
  controlAngleY = kalAngleY;
  controlAngleYintegr = controlAngleYintegr + controlAngleY * (double)(micros()-timer)/1000000; 
  if (controlAngleY > 0) {
    // наклон вперед

    // индикация наклона
    digitalWrite(LED_FORWARD, HIGH);
    digitalWrite(LED_BACKWARD, LOW);

    // устанавливаем направление моторов "вперед"
    digitalWrite(MOTOR_DIR_R, MOTOR_FORWARD_R);
    digitalWrite(MOTOR_DIR_L, MOTOR_FORWARD_L);
  }
  if (controlAngleY < 0) {
    // наклон назад

    // индикация наклона
    digitalWrite(LED_FORWARD, LOW);
    digitalWrite(LED_BACKWARD, HIGH);

    // устанавливаем направление моторов "назад"
    digitalWrite(MOTOR_DIR_R, MOTOR_BACKWARD_R);
    digitalWrite(MOTOR_DIR_L, MOTOR_BACKWARD_L);
  }

  // управление моторами
  double controlPID = abs(controlAngleY) * Kp * adjFactorKp + abs(controlAngleYintegr) * Ki * adjFactorKi;
  int controlPWM = round(controlPID);
  // получаем значения PWM для моторов с учетом коэффициентов и сдвига PWM
  motorPWMR = controlPWM * MOTOR_FACTOR_R + MOTOR_PWM_OFFSET;
  motorPWML = controlPWM * MOTOR_FACTOR_L + MOTOR_PWM_OFFSET;
  digitalWrite(LED_MODE, LOW);
  if (motorPWMR > 255) {
    motorPWMR = 255;
    digitalWrite(LED_MODE, HIGH);
  }
  if (motorPWML > 255) {
    motorPWML = 255;
    digitalWrite(LED_MODE, HIGH);
  }

  #ifndef MOTOR_OFF   
  analogWrite(MOTOR_PWM_R, motorPWMR);
  analogWrite(MOTOR_PWM_L, motorPWML);
  #endif

  // значение температуры
  temp = ((double)tempRaw + 12412.0) / 340.0;

  if (millis() - serial_timer > SERIAL_TIMER_DELAY) {
    // вывод значений каждые N мс
    serial_timer = millis();
    /*
    Serial.print(accX);Serial.print("\t");
    Serial.print(accY);Serial.print("\t");  
    Serial.print(accZ);Serial.print("\t");    
      
    Serial.print(gyroX);Serial.print("\t");  
    Serial.print(gyroY); Serial.print("\t");   
    Serial.print(gyroZ);Serial.print("\t");  
    */
    
    //Serial.print(accXangle);Serial.print("\t");
    Serial.print(accYangle);Serial.print("\t"); 
       
    //Serial.print(gyroXangle);Serial.print("\t");
    Serial.print(gyroYangle);Serial.print("\t");
      
    //Serial.print(compAngleX);Serial.print("\t");
    Serial.print(compAngleY); Serial.print("\t");
      
    //Serial.print(kalAngleX);Serial.print("\t");
    Serial.print(kalAngleY);Serial.print("\t");
    
    Serial.print(controlPID);Serial.print("\t");
    Serial.print(controlPWM);Serial.print("\t");

    Serial.print(VBat);Serial.print("\t");
    
    //Serial.print(temp);Serial.print("\t");
     
    Serial.print("\n");
  }
  
  delay(20);

  // отметка времени
  timer = micros();
}

// процедура калибровки MPU6050
void MPU6050_calibrate() {
  uint8_t data[14];
  int32_t accX_sum = 0;
  int32_t accY_sum = 0;
  int32_t accZ_sum = 0;  
  int16_t accX_avg;
  int16_t accY_avg;
  int16_t accZ_avg;

  int32_t gyroX_sum = 0;
  int32_t gyroY_sum = 0;
  int32_t gyroZ_sum = 0;  
  int16_t gyroX_avg;
  int16_t gyroY_avg;
  int16_t gyroZ_avg;
  uint32_t stab_timer;

  // ждем успокоения колебаний
  Serial.println("MPU6050 calibration, waiting for immobility");
  // критерии успокоения - отсутствие отклонений в данных гиро по осям X и Y 
  // от среднего больше MPU6050_GYRO_STAB_RANGE в течение 1 секунды,
  // средние пересчитываются после каждого замера
  // засекаем начальную отметку времени
  stab_timer = millis();
  for (int16_t i = 1; ; i++) {
    // мигаем светиком режима
    if (i % 2 == 0)
      digitalWrite(LED_MODE, HIGH);
    else
      digitalWrite(LED_MODE, LOW);
    
    // получаем значения из MPU6050
    MPU6050_read(0x3B, 14, data);
    gyroX = ((data[8] << 8) | data[9]);
    gyroY = ((data[10] << 8) | data[11]);

    // вычисляем средние значения
    gyroX_sum += gyroX;
    gyroY_sum += gyroY;
    gyroX_avg = gyroX_sum / i;
    gyroY_avg = gyroY_sum / i;

    Serial.print(i);Serial.print('\t');
    Serial.print(gyroX);Serial.print('\t');
    Serial.print(gyroX_sum);Serial.print('\t');
    Serial.print(gyroX_avg);Serial.print('\t');
    Serial.print(abs(gyroX - gyroX_avg));Serial.print('\t');
    Serial.print(gyroY);Serial.print('\t');
    Serial.print(gyroY_sum);Serial.print('\t');
    Serial.print(gyroY_avg);Serial.print('\t');
    Serial.print(abs(gyroY - gyroY_avg));Serial.print('\t');
    Serial.println();

    // индикация колебаний по Y
    if (gyroY - gyroY_avg > MPU6050_GYRO_STAB_RANGE) 
      digitalWrite(LED_FORWARD, HIGH);      
    else 
      digitalWrite(LED_FORWARD, LOW);
    if (gyroY - gyroY_avg < (-1) * MPU6050_GYRO_STAB_RANGE)
      digitalWrite(LED_BACKWARD, HIGH);      
    else 
      digitalWrite(LED_BACKWARD, LOW);

    if (abs(gyroX - gyroX_avg) > MPU6050_GYRO_STAB_RANGE || abs(gyroY - gyroY_avg) > MPU6050_GYRO_STAB_RANGE)
      // порог колебаний пока превышен, запоминаем отметку времени
      stab_timer = millis();

    if (millis() - stab_timer > 1000)
      // превышения порога колебаний не было больше 1 с,
      // выходим на калибровку
      break;
    
    delay(MPU6050_CALIBRATE_DELAY);
  }
  
  // выполняем калибровку
  Serial.println("MPU6050 calibration started");
  // включаем светик режима
  digitalWrite(LED_MODE, HIGH);
  for (int16_t i = 1; i <= MPU6050_CALIBRATE_SAMPLES_NUM; i++) {
    // получаем значения из MPU6050
    MPU6050_read(0x3B, 14, data);
    accX = ((data[0] << 8) | data[1]);
    accY = ((data[2] << 8) | data[3]);
    accZ = ((data[4] << 8) | data[5]);   
    gyroX = ((data[8] << 8) | data[9]);
    gyroY = ((data[10] << 8) | data[11]);
    gyroZ = ((data[12] << 8) | data[13]);

    // вычисляем накопленные значения
    accX_sum += accX;
    accY_sum += accY;
    accZ_sum += accZ;
    gyroX_sum += gyroX;
    gyroY_sum += gyroY;
    gyroZ_sum += gyroZ;
   
    delay(MPU6050_CALIBRATE_DELAY);
  }
  // вычисляем средние значения
  accX_avg = accX_sum / MPU6050_CALIBRATE_SAMPLES_NUM;
  accY_avg = accY_sum / MPU6050_CALIBRATE_SAMPLES_NUM;
  accZ_avg = accZ_sum / MPU6050_CALIBRATE_SAMPLES_NUM;
  gyroX_avg = gyroX_sum / MPU6050_CALIBRATE_SAMPLES_NUM;
  gyroY_avg = gyroY_sum / MPU6050_CALIBRATE_SAMPLES_NUM;
  gyroZ_avg = gyroZ_sum / MPU6050_CALIBRATE_SAMPLES_NUM;

  if (accZ_avg < 0) {
    // калибровка выполнялась "вверх ногами", значения для акселя инвертируем
    accX_avg = -accX_avg;
    accY_avg = -accY_avg;
    accZ_avg = -accZ_avg;
  }

  Serial.println("MPU6050 calibration complete");
  Serial.print(accX_avg);Serial.print('\t');
  Serial.print(accY_avg);Serial.print('\t');
  Serial.print(accZ_avg);Serial.print('\t');
  Serial.print(gyroX_avg);Serial.print('\t');
  Serial.print(gyroY_avg);Serial.print('\t');
  Serial.print(gyroZ_avg);Serial.print('\t');
  Serial.println();

  // сохраняем калибровочные значения в EEPROM
  EEPROM.write(0, (accX_avg >> 8) & 0xFF);
  EEPROM.write(1, accX_avg & 0xFF);
  EEPROM.write(2, (accY_avg >> 8) & 0xFF);
  EEPROM.write(3, accY_avg & 0xFF);
  EEPROM.write(4, (accZ_avg >> 8) & 0xFF);
  EEPROM.write(5, accZ_avg & 0xFF);

  EEPROM.write(6, (gyroX_avg >> 8) & 0xFF);
  EEPROM.write(7, gyroX_avg & 0xFF);
  EEPROM.write(8, (gyroY_avg >> 8) & 0xFF);
  EEPROM.write(9, gyroY_avg & 0xFF);
  EEPROM.write(10, (gyroZ_avg >> 8) & 0xFF);
  EEPROM.write(11, gyroZ_avg & 0xFF);
  
  // включаем на 2 с все светики
  digitalWrite(LED_FORWARD, HIGH);
  digitalWrite(LED_BACKWARD, HIGH);
  delay(2000);
  digitalWrite(LED_MODE, LOW);
  digitalWrite(LED_FORWARD, LOW);
  digitalWrite(LED_BACKWARD, LOW);

  // останов
  while(1);
}

// функция чтения регистров из MPU6050 
int MPU6050_read(byte registerAddress, int numBytes, byte data[]) {
  int i, r;

  Wire.beginTransmission(MPU6050_I2C_ADDR);
  
  r = Wire.write(registerAddress); // устанавливаем начальный адрес, с которого будем читать
  if (r != 1) return (-1);
  
  r = Wire.endTransmission(false); // не отпускаем шину
  if (r != 0) return r;

  Wire.requestFrom(MPU6050_I2C_ADDR, numBytes, true);  // устанавливаем кол-во читаемых регистров с освобождением шины
  while(Wire.available() && i < numBytes) {
    data[i++] = Wire.read();
  }
  if (i != numBytes) return (-2);
  return (0);
}

// функция записи в регистр MPU6050 
int MPU6050_write_reg(int registerAddress, const byte data)
{
  int r;

  Wire.beginTransmission(MPU6050_I2C_ADDR);
  r = Wire.write(registerAddress);  // устанавливаем начальный адрес
  if (r != 1) return (-1);

  r = Wire.write(data);   // записываем данные
  if (r != 1) return (-2);

  r = Wire.endTransmission(true);   // освобождаем шину
  if (r != 0) return (r);

  return (0);      
}

// функция чтения данных из контроллера энкодеров
int Encoder_read(int numBytes, byte data[]) {

  Wire.requestFrom(ENCODER_I2C_ADDR, numBytes);  // устанавливаем кол-во читаемых регистров
  int numBytesAvailable = Wire.available();
  if (numBytes != numBytesAvailable)
    // кол-во возвращенных слов отличается от кол-ва запрошенных
    return (-1);
  
  // читаем данные 
  for (int i = 0; i < numBytesAvailable; i++) {
    data[i] = Wire.read();
    Serial.println(data[i]);
  }
  return (0);
}

// сигнализация кода ошибки светодиодами
void show_error_code(byte code) {
  switch (code) {
    case 1:
      digitalWrite(LED_FORWARD, HIGH);
      digitalWrite(LED_BACKWARD, LOW);
      break;
    case 2:
      digitalWrite(LED_FORWARD, LOW);
      digitalWrite(LED_BACKWARD, HIGH);
      break;
    case 3:
      digitalWrite(LED_FORWARD, HIGH);
      digitalWrite(LED_BACKWARD, HIGH);
      break;
  }
}

