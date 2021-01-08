#include <Wire.h>

// адреса I2C
// акселерометр/гироскоп
#define MPU6050_I2C_ADDR 0x68
// контроллер энкодеров
#define ENCODER_I2C_ADDR 0x73


// функция чтения данных из MPU6050 
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
  int i, r;

  Wire.requestFrom(ENCODER_I2C_ADDR, numBytes);  // устанавливаем кол-во читаемых регистров с освобождением шины
  while(Wire.available() && i < numBytes) {
    data[i++] = Wire.read();
  }
  if (i != numBytes) return (-2);
  return (0);
}

void setup() {
  
  // шина I2C и COM-порт
  Wire.begin(); 
  Serial.begin(115200);

  // отключаем режим SLEEP в MPU6050 (регистр PWR_MGMT_1)
  MPU6050_write_reg(0x6B, 0);
}

void loop() {
  
  uint8_t res[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  int32_t enc1, enc2;
  int8_t err;

  uint8_t res_MPU6050[14];
  int16_t accX;
  int16_t accY;
  int16_t accZ;
  int16_t tempRaw;
  int16_t gyroX;
  int16_t gyroY;
  int16_t gyroZ;

  double accXangle; // Angle calculate using the accelerometer
  double accYangle;
  
  delay(500);

  Serial.println(Encoder_read(10, res));
  //if (i2cRead(0x73, 0, 10, res)) {
    // enc1 = (long)(((uint32_t)res[0])<<24 | ((uint32_t)res[1])<<16 | ((uint32_t)res[2])<<8 | (uint32_t)res[3]);
    enc1 = (res[0]<<24) | (res[1]<<16) | (res[2]<<8) | res[3];
    enc2 = (res[4]<<24) | (res[5]<<16) | (res[6]<<8) | res[7];
    err =  (res[8]<<8) | res[9];
  //}

  Serial.print(enc1);
  Serial.print(' ');
  Serial.print(enc2);
  Serial.print(' ');
  Serial.print(err);
  Serial.println();
  
  Serial.println(MPU6050_read(0x3B, 14, res_MPU6050));
  accX = ((res_MPU6050[0] << 8) | res_MPU6050[1]);
  accY = ((res_MPU6050[2] << 8) | res_MPU6050[3]);
  accZ = ((res_MPU6050[4] << 8) | res_MPU6050[5]);  
  tempRaw = ((res_MPU6050[6] << 8) | res_MPU6050[7]);  
  gyroX = ((res_MPU6050[8] << 8) | res_MPU6050[9]);
  gyroY = ((res_MPU6050[10] << 8) | res_MPU6050[11]);
  gyroZ = ((res_MPU6050[12] << 8) | res_MPU6050[13]);

  //accYangle = atan(-1*accX/sqrt(pow(accY,2) + pow(accZ,2)))*RAD_TO_DEG;
  //accXangle = atan(accY/sqrt(pow(accX,2) + pow(accZ,2)))*RAD_TO_DEG;

  Serial.print(accY);
  Serial.print(' ');
  Serial.println(accX);
}



