// Прошивка для обработчика сигналов от 2-х квадратурных энкодеров для балансирующего робота
// Обмен с основным контроллером по i2c
// МК ATTiny85, 16 МГц internal
// Подключение:
// 2 (PB3), 3 (PB4) - энкодер 1
// 1 (PB5), 6 (PB1) - энкодер 2
// 5 - SDA, 7 - SCL
// Замечания:
// 1.Используется вариант библиотеки TinyWireS от 1/23/2011 BroHogan - brohoganx10 at gmail dot com
// 2.Контакт 1 МК используется как PB5 вместо Reset, необходимо установить RSTDISBL в 0 (старший бит hfuse) 
//
// Дата модификации 09.08.2020

#define I2C_SLAVE_ADDRESS 0x73 // 7-bit адрес
#include "TinyWireS.h"

// входы от энкодеров
const int enc1PinA = 3; 
const int enc1PinB = 4;
const int enc2PinA = 5; 
const int enc2PinB = 1;

// предыдущие состояния энкодеров
byte enc1Prev;
byte enc2Prev;

// числовые значения положений
volatile int32_t enc1Value = 0;
volatile int32_t enc2Value = 0;
// счетчик ошибок
volatile int16_t encErrors = 0;

// регистры для i2c
volatile byte i2cRegs[10];
// текущий адрес регистра
volatile byte regIndex = 0;

// обработчик прерываний от энкодеров
ISR(PCINT0_vect){
  byte enc1, enc2;
  byte enc1State, enc2State;

  // получаем состояния энкодеров
  enc1 = (digitalRead(enc1PinA) << 1) | digitalRead(enc1PinB);
  enc2 = (digitalRead(enc2PinA) << 1) | digitalRead(enc2PinB);
  
  // объединяем с предыдущими состояниями
  enc1State = (enc1Prev << 2) | enc1;
  enc2State = (enc2Prev << 2) | enc2;
  
  // определяем направления вращения
  switch (enc1State) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000:
      // вперед
      enc1Value++;
      break;
      
    case 0b0010: case 0b1011: case 0b1101: case 0b0100:
      // назад 
      enc1Value--;
      break;
      
    case 0b0011: case 0b0110: case 0b1100: case 0b1001:
      // запрещенный переход, ошибка
      encErrors++;
      break;
  }
  switch (enc2State) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000:
      // вперед
      enc2Value++;
      break;
      
    case 0b0010: case 0b1011: case 0b1101: case 0b0100:
      // назад 
      enc2Value--;
      break;
      
    case 0b0011: case 0b0110: case 0b1100: case 0b1001:
      // запрещенный переход, ошибка
      encErrors++;
      break;
  }
  
  // сохраняем состояния
  enc1Prev = enc1;
  enc2Prev = enc2;
}


// обработчик чтения от i2c Master
void requestEvent()
{  
  // фиксируем значения в регистрах i2с
  if (regIndex == 0) {
    i2cRegs[0] = (byte)((enc1Value >> 24) & 0xFF);
    i2cRegs[1] = (byte)((enc1Value >> 16) & 0xFF);
    i2cRegs[2] = (byte)((enc1Value >> 8) & 0xFF);
    i2cRegs[3] = (byte)(enc1Value & 0xFF);
  }
  if (regIndex == 4 ) {
    i2cRegs[4] = (byte)((enc2Value >> 24) & 0xFF);
    i2cRegs[5] = (byte)((enc2Value >> 16) & 0xFF);
    i2cRegs[6] = (byte)((enc2Value >> 8) & 0xFF);
    i2cRegs[7] = (byte)(enc2Value & 0xFF);
  }
  if (regIndex == 8) {
    i2cRegs[8] = (byte)((encErrors >> 8) & 0xFF);
    i2cRegs[9] = (byte)(encErrors & 0xFF);
  }

  // передаем значение из очередного регистра
  TinyWireS.send(i2cRegs[regIndex]);

  // инкрементируем адрес регистра
  regIndex++;
  if (regIndex == sizeof(i2cRegs))
    regIndex = 0;
}


void setup() {
  
  // инициализируем входы для энкодеров
  pinMode(enc1PinA, INPUT_PULLUP);
  pinMode(enc1PinB, INPUT_PULLUP);
  pinMode(enc2PinA, INPUT_PULLUP);
  pinMode(enc2PinB, INPUT_PULLUP);

  // временно запрещаем прерывания
  noInterrupts();

  // разрешаем Pin Change прерывания для входов от энкодеров
  GIMSK = 0b00100000;
  PCMSK |= 1 << enc1PinA | 1 << enc1PinB | 1 << enc2PinA | 1 << enc2PinB; // прерывания от пинов
  
  // считываем состояния входов
  enc1Prev = digitalRead(enc1PinA) << 1 | digitalRead(enc1PinB);
  enc1Prev = digitalRead(enc2PinA) << 1 | digitalRead(enc2PinB);
  
  // разрешаем прерывания 
  interrupts();

  // инициализируем i2c
  TinyWireS.begin(I2C_SLAVE_ADDRESS);
  TinyWireS.onRequest(requestEvent);
}


void loop(){
  TinyWireS_stop_check();  
}


