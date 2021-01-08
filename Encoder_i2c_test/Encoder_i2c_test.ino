#include <Wire.h>


void readFrom(byte device, int num, byte result[]) {
  Wire.requestFrom((int)device, num);
  int i = 0;
  while(Wire.available()) {
    result[i] = Wire.read();
    i++;
  }
}

void setup() {
  
  // шина I2C и COM-порт
  Wire.begin(); 
  Serial.begin(115200);
}

void loop() {
  
  byte res[10] = {0};
  long enc1, enc2;
  int err;
  
  delay(100);
  readFrom(0x73, 10, res);
  enc1 = (res[0]<<24) | (res[1]<<16) | (res[2]<<8) | res[3];
  enc2 = (res[4]<<24) | (res[5]<<16) | (res[6]<<8) | res[7];
  err =  (res[8]<<8) | res[9];
  
  Serial.print(enc1);
  Serial.print(' ');
  Serial.print(enc2);
  Serial.print(' ');
  Serial.print(err);
  Serial.println();

}



