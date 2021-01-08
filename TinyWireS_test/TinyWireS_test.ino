#define I2C_SLAVE_ADDRESS 0x73 // 7-bit адрес
#include <TinyWireS.h>

// регистры для i2c
volatile byte i2cRegs[5] = {0xFA, 0xFB, 0xFC, 0xFD, 0xFE};
// текущий адрес регистра
volatile int regIndex = 0;

// обработчик записи от i2c Master
void receiveEvent(byte bytesReceived)
{
  if (bytesReceived != 1)
    // допустима только запись адреса текущего регистра 
    return;
 
  // сохраняем полученный адрес регистра
  regIndex = TinyWireS.receive();
  if (regIndex >= sizeof(i2cRegs))
    regIndex = 0;
}

// обработчик чтения от i2c Master
void requestEvent()
{  

  // передаем значение из очередного регистра
  TinyWireS.send(i2cRegs[regIndex]);

  // инкрементируем адрес регистра
  regIndex++;
  if (regIndex >= sizeof(i2cRegs))
    regIndex = 0;
}


void setup() {
  
  // инициализируем i2c
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onReceive(receiveEvent);
  TinyWireS.onRequest(requestEvent);
}

void loop(){
  TinyWireS_stop_check();
}


