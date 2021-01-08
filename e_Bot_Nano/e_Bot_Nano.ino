#include <Wire.h>

// светодиоды наклона вперед и назад
#define FRONT_LED 5
#define BACK_LED 6
// светодиод на плате
#define LED13 13

// направления моторов
#define R_FORW LOW
#define R_BACK HIGH
#define L_FORW HIGH
#define L_BACK LOW
// порты управления моторами
#define R_PWM 9
#define L_PWM 10
#define R_DIR 11
#define L_DIR 12
// коэффициенты моторов 
#define R_FACTOR 1.05
#define L_FACTOR 1

/* Kalman filter variables and constants */
const double Q_angleX = 0.0001; // Process noise covariance for the accelerometer - Sw (0.001)
const double Q_gyroX = 0.2; // Process noise covariance for the gyro - Sw           (0.003)
const double R_angleX = 0.03; // Measurement noise covariance - Sv
double angleX = 0; // The angle output from the Kalman filter
double biasX = 0; // The gyro bias calculated by the Kalman filter
double PX_00 = 0, PX_01 = 0, PX_10 = 0, PX_11 = 0;
double dtX, yX, SX;
double KX_0, KX_1;  

int gyroResult[3], accelResult[3];

float timeStep = 0.02;          //20ms. Need a time step value for integration of gyro angle from angle/sec
float biasGyroX, biasGyroY, biasGyroZ, biasAccelX, biasAccelY, biasAccelZ;
float pitchGyro = 0;
float pitchRateGyro = 0;
float pitchAccel = 0;

float pitchPrediction = 0; //Output of Kalman filter
float rollGyro = 0;
float rollAccel = 0;
float rollPrediction = 0;  //Output of Kalman filter
float giroVar = 0.1;
float deltaGiroVar = 0.1;
float accelVar = 5;
float Pxx = 0.1; // angle variance
float Pvv = 0.1; // angle change rate variance
float Pxv = 0.1; // angle and angle change rate covariance
float kx, kv;
float pitch;
float pitchPrev;
float pitchDerivative;
float pitchIntegral = 0;
float m_control, r_control, l_control;
unsigned long timer;

void writeTo(byte device, byte toAddress, byte val) {
  Wire.beginTransmission(device);  
  Wire.write(toAddress);        
  Wire.write(val);        
  Wire.endTransmission();
}

void readFrom(byte device, byte fromAddress, int num, byte result[]) {
  Wire.beginTransmission(device);
  Wire.write(fromAddress);
  Wire.endTransmission();
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
}

void getGyroscopeReadings(int gyroResult[]) {
  byte buffer[6];
  readFrom(0x68,0x1D,6,buffer);
  gyroResult[0] = (((int)buffer[0]) << 8 ) | buffer[1];
  gyroResult[1] = (((int)buffer[2]) << 8 ) | buffer[3];
  gyroResult[2] = (((int)buffer[4]) << 8 ) | buffer[5];
} 

void getAccelerometerReadings(int accelResult[]) {
  byte buffer[6];
  readFrom(0x53,0x32,6,buffer);
  accelResult[0] = (((int)buffer[1]) << 8 ) | buffer[0];
  accelResult[1] = (((int)buffer[3]) << 8 ) | buffer[2];
  accelResult[2] = (((int)buffer[5]) << 8 ) | buffer[4];
}


double kalmanX(double newAngle, double newRate, double dtime) {

  // KasBot V2  -  Kalman filter module - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418
  // See also http://www.x-firm.com/?page_id=145
  // with slightly modifications by Kristian Lauszus
  // See http://academic.csuohio.edu/simond/courses/eec644/kalman.pdf and
  // http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf for more information
  dtX = dtime;  // уже в секундах   / 1000000; // Convert from microseconds to seconds
  // Discrete Kalman filter time update equations - Time Update ("Predict")
  // Update xhat - Project the state ahead  angleX += dtX * (newRate - biasX);
  // Update estimation error covariance - Project the error covariance ahead
  PX_00 += -dtX * (PX_10 + PX_01) + Q_angleX * dtX;
  PX_01 += -dtX * PX_11;
  PX_10 += -dtX * PX_11;
  PX_11 += +Q_gyroX * dtX;
  // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
  // Calculate Kalman gain - Compute the Kalman gain
  SX = PX_00 + R_angleX;
  KX_0 = PX_00 / SX;
  KX_1 = PX_10 / SX;
  // Calculate angle and resting rate - Update estimate with measurement zk
  yX = newAngle - angleX;
  angleX += KX_0 * yX;
  biasX += KX_1 * yX;
  // Calculate estimation error covariance - Update the error covariance
  PX_00 -= KX_0 * PX_00;
  PX_01 -= KX_0 * PX_01;
  PX_10 -= KX_1 * PX_00;
  PX_11 -= KX_1 * PX_01;
  return angleX;
} 
 

void setup() {
  int totalGyroXValues = 0;
  int totalGyroYValues = 0;
  int totalGyroZValues = 0;
  int totalAccelXValues = 0;
  int totalAccelYValues = 0;
  int totalAccelZValues = 0;
  int i;
  
  // индикаторы наклона и общий индикатор
  pinMode(FRONT_LED, OUTPUT);  
  pinMode(BACK_LED, OUTPUT);
  pinMode(LED13, OUTPUT);
  // порты моторов
  pinMode(R_PWM, OUTPUT);  
  pinMode(L_PWM, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  
  // шина I2C и COM-порт
  Wire.begin(); 
  Serial.begin(38400);
  
  writeTo(0x53,0x31,0x09); //Set accelerometer to 11bit, +/-4g
  writeTo(0x53,0x2D,0x08); //Set accelerometer to measure mode
  writeTo(0x68,0x16,0x1A); //Set gyro to +/-2000deg/sec and 98Hz low pass filter
  writeTo(0x68,0x15,0x09); //Set gyro to 100Hz sample rate
  delay(100); //wait for gyro to "spin" up
  
  // калибровка акселя и гиро по 50 измерениям
  // включаем индикаторы на 2 секунды
  analogWrite(FRONT_LED, 100);
  analogWrite(BACK_LED, 100);
  delay(2000);
  
  for (i = 0; i < 50; i += 1) {
    getGyroscopeReadings(gyroResult);
    getAccelerometerReadings(accelResult);
    totalGyroXValues += gyroResult[0];
    totalGyroYValues += gyroResult[1];
    totalGyroZValues += gyroResult[2];
    totalAccelXValues += accelResult[0];
    totalAccelYValues += accelResult[1];
    totalAccelZValues += accelResult[2];
    
    // мигаем индикаторами
    analogWrite(FRONT_LED, 50 * (i % 2));
    analogWrite(BACK_LED, 50 * ((i + 1) % 2));
    
    delay(50);
  }
  // выключаем индикаторы
  digitalWrite(FRONT_LED, LOW);
  digitalWrite(BACK_LED, LOW);
  
  biasGyroX = totalGyroXValues / 50;
  biasGyroY = totalGyroYValues / 50;
  biasGyroZ = totalGyroZValues / 50;
  biasAccelX = totalAccelXValues / 50;
  biasAccelY = totalAccelYValues / 50;
  // biasAccelZ = -((totalAccelZValues / 50) + 256); //Don't compensate gravity away! We would all (float)! // калибровка вверх ногами
  biasAccelZ = (totalAccelZValues / 50) - 256; //Don't compensate gravity away! We would all (float)! 
  Serial.println(totalAccelZValues /50);
  delay(2000);
}

void loop() {
  timer = millis(); //get a start value to determine the time the loop takes
  getGyroscopeReadings(gyroResult);
  getAccelerometerReadings(accelResult);
  
  //Calculate the pitch in degrees as measured by the accelerometers. Note that 8g into 11 bits gives 256 bits/g
  pitchAccel = -atan2((accelResult[1] - biasAccelY) / 256, (accelResult[2] - biasAccelZ) / 256) * 180.0 / PI;
  // изменение угла от гиро
  pitchRateGyro = (gyroResult[1] - biasGyroY) / 14.375;
  
  //Calculate the pitch in degrees as measured by the gyros, angle = angle + gyro reading * time step.
  pitchGyro = pitchGyro + ((gyroResult[1] - biasGyroY) / 14.375) * timeStep;
    
  // запоминаем результат прошлой итерации
  pitchPrev = pitch;
  
  // результат текущей итерации
  pitch = kalmanX(pitchAccel, pitchRateGyro, timeStep);
  
  // интеграл
  pitchIntegral = pitchIntegral + pitch * timeStep;
  // ограничиваем интеграл, иначе не вылезем из наклонного положения
  if (abs(pitch) > 10)
    pitchIntegral = 0;
  
  // производная
  pitchDerivative = (pitch - pitchPrev) / timeStep;
    
  // индикация наклона
  // максимум на +-10 градусов
  if (pitch >= 0) {
    analogWrite(FRONT_LED, constrain(pitch, 0, 10) * 25); 
    analogWrite(BACK_LED, 0);
  }
  else {
    analogWrite(BACK_LED, constrain(-pitch, 0, 10) * 25); 
    analogWrite(FRONT_LED, 0);
  }
  
  // воздействие
  if (abs(pitch) > 10) 
    // больше чем можем исправить, выключаем моторы
    m_control = 0;
  else
    m_control = 50 * pitch + 10 * pitchIntegral /* + 0.3 * pitchDerivative */;
  // моторы - направление
  digitalWrite(R_DIR, (m_control > 0)?R_FORW:R_BACK); 
  digitalWrite(L_DIR, (m_control > 0)?L_FORW:L_BACK);
  // моторы - скорость
  r_control = round(abs(m_control) * R_FACTOR);
  if (r_control > 255) r_control = 255;
  l_control = round(abs(m_control) * L_FACTOR);
  if (l_control > 255) l_control = 255;
  analogWrite(R_PWM, r_control);
  analogWrite(L_PWM, l_control);
  
  // индикация максимума PWM моторов 
  digitalWrite(LED13, (r_control == 255 or l_control == 255)?HIGH:LOW);
  
  Serial.print(round(pitchAccel * 100));
  Serial.print(";");
  //Serial.print(round(pitchGyro * 100));
  //Serial.print(";");
  Serial.print(round(pitch * 100));
  //Serial.print(pitchIntegral);
  //Serial.print("\t");  
  //Serial.print(pitchDerivative);
  //Serial.print("\t");
  //Serial.print(m_control);
  Serial.println();
  
  /*
  Serial.print(pitchAccel);
  Serial.print("\t");
  Serial.print(pitchRateGyro);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  //Serial.print(pitchIntegral);
  //Serial.print("\t");  
  //Serial.print(pitchDerivative);
  //Serial.print("\t");
  //Serial.print(m_control);
  Serial.println();
  */
  
  timer = millis() - timer;          //how long did the loop take?
  if (timer >= timeStep * 1000)
    timer = 0;
  else 
    timer = (timeStep * 1000) - timer; //how much time to add to the loop to make it last time step msec
  delay(timer);                        //make one loop last time step msec
  
  //Serial.println(timer);
}
