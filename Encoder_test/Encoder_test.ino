// Тест энкодера для Arduino UNO
// входы от энкодеров
const int enc1PinA = 2; 
const int enc1PinB = 3;
const int enc2PinA = 4; 
const int enc2PinB = 5;

// предыдущие состояния энкодеров
volatile byte enc1Prev;
volatile byte enc2Prev;

// значения энкодеров
volatile long enc1Value = 0;
volatile long enc2Value = 0;
volatile long encErrors = 0;

// обработчик прерываний от энкодеров
ISR(PCINT2_vect){
  byte enc1, enc2;
  byte enc1State, enc2State;

  // получаем состояния энкодеров
  enc1 = digitalRead(enc1PinA) << 1 | digitalRead(enc1PinB);
  enc2 = digitalRead(enc2PinA) << 1 | digitalRead(enc2PinB);
  
  // объединяем с предыдущими состояниями
  enc1State = enc1Prev << 2 | enc1;
  enc2State = enc2Prev << 2 | enc2;
  
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
      // запрещенные переходы, ошибка
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
      // запрещенные переходы, ошибка
      encErrors++;
      break;
  }
  
  // сохраняем состояния
  enc1Prev = enc1;
  enc2Prev = enc2;
}

void setup() {
  
  // инициализируем пины
  pinMode(enc1PinA, INPUT_PULLUP);      
  pinMode(enc1PinB, INPUT_PULLUP); 
  pinMode(enc2PinA, INPUT_PULLUP);      
  pinMode(enc2PinB, INPUT_PULLUP); 

  // временно запрещаем прерывания
  noInterrupts();

  // разрешаем Pin Change прерывания для входов от энкодеров
  PCICR |= 0b00000100;    // прерывания от порта D
  PCMSK2 |= 1 << enc1PinA | 1 << enc1PinB | 1 << enc2PinA | 1 << enc2PinB; // прерывания от пинов
  
  // считываем состояния входов
  enc1Prev = digitalRead(enc1PinA) << 1 | digitalRead(enc1PinB);
  enc1Prev = digitalRead(enc2PinA) << 1 | digitalRead(enc2PinB);
  
  // разрешаем прерывания 
  interrupts();
  
  Serial.begin(115200);
}

void loop(){

  delay(1000);
  Serial.println(enc1Value);
  Serial.println(encErrors);
  
}


