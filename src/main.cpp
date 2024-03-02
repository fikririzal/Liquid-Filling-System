#include <Arduino.h>
#include <digitalWriteFast.h>
#include <Keypad.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display

const byte ROWS = 4; // four rows
const byte COLS = 4; // three columns
char keys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};

byte rowPins[ROWS] = {12, 10, 9, 8}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {7, 6, 5, 4};   // connect to the column pinouts of the keypad

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
#define ledPin 13

// Type Variable
#define charLenght 5
uint8_t typeIndex = 0;
char typeArray[charLenght] = "0";
char emptytypeArray[charLenght] = "";
const char empty[charLenght] = "";
uint32_t typeInteger = 0;

// PWM
#define PWMpin 11
uint16_t topPWM = 0xFF;

// Pulse Variable
volatile uint16_t pulses = 0;

// Liter per Minute Calculation Variable
volatile uint32_t LpM_calc_time = 0;
volatile uint32_t last_LpM_calc_time = 0;
volatile float LperMinute = 0;
volatile float CallibrationFactor = 7.5;

// Milliliter per 500ms Calculation Variable
volatile uint32_t mLp500_calc_time = 0;
volatile uint32_t last_mLp500_calc_time = 0;
volatile float mLp500 = 0;

// Water Volume Variable
double offsetVolume = 50.00;
volatile double targetVolume = 0;
volatile double currentVolume = 0;
volatile double dsp_currentVolume = 0;

// Filling System Variable
bool Filling = false;
volatile bool Pumping = false;
uint8_t maxPower = 100;

// Program Selector Variabel
#define IDLE 0xA1
#define FILLING 0xA2
#define PUMPING 0xA3
volatile uint8_t Status = IDLE;

// ISR Variable
volatile uint32_t last_time = 0;
volatile uint8_t runIndex = 0;

// fungtion declaration
void writePWM(uint8_t, boolean);
void Sens();
void setupPWM(uint16_t);
void ControlSystem();
void Calculation();
void clearTypeArray();
void input2typeArray(char);
void typeArray2uint32(uint32_t *);
void typeHandler();
void EmergencyStop();
void Status2Idle();
void Status2filling(uint32_t);
void Status2pumping();
void startTimer1();
void stopTimer1();
void display();
uint8_t isTypeArrayEmpty();

void setup()
{
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();

  setupPWM(topPWM);
  pinModeFast(ledPin, OUTPUT); // Sets the digital pin as output.

  // startTimer1();
  // DDRD = 1 << PORTD4;
  // detachInterrupt(0);
  attachInterrupt(0, Sens, FALLING); //  function for creating external interrupts at pin2 on Rising (LOW to HIGH)
  Serial.println("Pertanimi Basic Test");
}

ISR(TIMER1_COMPA_vect)
{ // timer1 interrupt 1Hz toggles pin 13 (LED)
  // generates pulse wave of frequency 1Hz/2 = 0.5kHz (takes two cycles for full wave- toggle high then toggle low)
  digitalWriteFast(ledPin, !digitalReadFast(ledPin));

  /* mLp500_calc_time = millis(); */
  mLp500 = /* ((mLp500_calc_time - last_mLp500_calc_time) / 50) *  */ ((LperMinute / 60) * 1000) / 20;
  /* last_mLp500_calc_time = mLp500_calc_time; */
  currentVolume += mLp500;
  dsp_currentVolume = currentVolume;

  if (currentVolume > targetVolume && Status == FILLING)
  {
    dsp_currentVolume = targetVolume;
  }

  /* Serial.print(String(mLp500) + String("mL/500, "));
  Serial.print(String(currentVolume) + String("mL, "));
  Serial.println(String(LperMinute) + String("L/m")); */

  runIndex++;
  if (runIndex >= 5)
  {
    runIndex = 0;
    /* LpM_calc_time = millis(); */
    detachInterrupt(0);
    LperMinute = /* ((LpM_calc_time - last_LpM_calc_time) / 250.0) *  */ (pulses / CallibrationFactor * 4);
    /* last_LpM_calc_time = LpM_calc_time; */
    pulses = 0;
    attachInterrupt(0, Sens, FALLING);
  }

  if (currentVolume >= targetVolume && Status == FILLING)
  {
    Status2Idle();
  }
}

void loop()
{
  switch (Status)
  {
  case IDLE:
    typeHandler();
    writePWM(0, true);

    break;
  case FILLING:
    EmergencyStop();
    ControlSystem();
    break;
  case PUMPING:
    EmergencyStop();
    ControlSystem();
    break;
  }

  display();

  static uint32_t last = 0;
  if (millis() - last >= 100 && Status != IDLE)
  {
    last = millis();
    Serial.print(String(currentVolume) + String("mL, "));
    Serial.println(String(LperMinute) + String("L/m"));
  }

  //_bitWrite(&PORTD, PORTD4, 1);
  //_bitWrite(&PORTD, PORTD4, 0);
}

void typeHandler()
{
  char key = keypad.getKey();
  switch (key)
  {
  case '*':
    if (isTypeArrayEmpty())
    {
      // Serial.println("typeArray empty, procced pumping");
      Status2pumping();
      break;
    }
    typeArray2uint32(&typeInteger);
    if (typeInteger < 100)
    {
      clearTypeArray();
      // Serial.println("typeInteger is to low below 100");
    }

    clearTypeArray();
    /* Serial.print("typeInteger[mL]: ");
    Serial.println(typeInteger); */
    Status2filling(typeInteger);
    // Serial.println("typeArray cleared");
    break;

  case '#':
    if (isTypeArrayEmpty())
    {
      // Serial.println("typeArray empty");
      break;
    }
    clearTypeArray();
    // Serial.println("typeArray cleared");
    break;
  case 'A':
    CallibrationFactor = CallibrationFactor + 0.1;
    break;

  case 'B':
    CallibrationFactor = CallibrationFactor - 0.1;
    break;

  default:
    if (key != '\0')
    {
      input2typeArray(key);
      Serial.println(key);
      /*Serial.println(" pressed"); */
    }
    break;
  }
}

void display()
{
  static char flow_buffer[8] = "";
  static char call_buffer[8] = "";
  static char f_current_buffer[20] = "";
  static char f_target_buffer[30] = "";
  static char f_flow_buffer[20] = "";

  dtostrf(LperMinute, 2, 2, flow_buffer);
  dtostrf(CallibrationFactor, 2, 2, call_buffer);

  if (Status == IDLE)
  {
    sprintf(f_target_buffer, "%smL     ", typeArray);
  }
  else
  {
    sprintf(f_target_buffer, "%dmL     ", uint16_t(targetVolume));
  }

  sprintf(f_current_buffer, "%dmL     ", uint16_t(dsp_currentVolume));
  // sprintf(f_target_buffer, "%dmL     ", uint16_t(targetVolume));
  sprintf(f_flow_buffer, "%sL/m     ", flow_buffer);

  lcd.setCursor(0, 0);
  lcd.print(f_current_buffer);

  lcd.setCursor(0, 1);
  lcd.print(f_target_buffer);

  lcd.setCursor(16 - (strlen(f_flow_buffer) - 5), 1);
  lcd.print(f_flow_buffer);

  lcd.setCursor(16 - (strlen(call_buffer)), 0);
  lcd.print(call_buffer);
}

void EmergencyStop()
{
  if (keypad.getKey() == '#')
  {
    Status2Idle();
  }
}

void Status2Idle()
{
  writePWM(0, true);
  Pumping = false;
  Status = IDLE;
  targetVolume = 0;
  stopTimer1();
}

void Status2filling(uint32_t inputValue)
{
  targetVolume = inputValue;
  Status = FILLING;
  currentVolume = 0.0;
  LperMinute = 0.0;
  last_LpM_calc_time = millis();
  last_mLp500_calc_time = millis() - 50;
  pulses = 0;
  runIndex = 0;
  startTimer1();
  attachInterrupt(0, Sens, FALLING);
}

void Status2pumping()
{
  targetVolume = 0;
  Status = PUMPING;
  Pumping = true;
  currentVolume = 0.0;
  LperMinute = 0.0;
  last_LpM_calc_time = millis();
  last_mLp500_calc_time = millis() - 50;
  pulses = 0;
  runIndex = 0;
  startTimer1();
  attachInterrupt(0, Sens, FALLING);
}

void ControlSystem()
{
  if (Status == PUMPING)
  {
    if (Pumping)
    {
      writePWM(maxPower, true);
    }
    else
    {
      Status2Idle();
    }
  }

  if (Status == FILLING)
  {
    /* if (currentVolume > targetVolume - offsetVolume)
    {
      writePWM(maxPower, true);
    }
    else */
    if (currentVolume < offsetVolume)
    {
      writePWM(70, true);
    }
    else
    {
      writePWM(maxPower, true);
    }
  }
}

void clearTypeArray()
{
  typeIndex = 0;
  memset(typeArray, '\0', sizeof(typeArray));
  typeArray[0] = '0';
}

void input2typeArray(char input)
{
  char wer = '0';
  if (typeIndex == 0 && input == wer)
    return;

  if (typeIndex > charLenght - 1)
    return;
  typeArray[typeIndex] = input;
  typeIndex++;
}

void typeArray2uint32(uint32_t *variabel)
{
  *variabel = atol(typeArray);
}

uint8_t isTypeArrayEmpty()
{
  return strcmp(empty, typeArray) == 0 ? 1 : 0;
}

void writePWM(uint8_t percentage, boolean smooth)
{
  uint16_t j = percentage > 100 ? topPWM : map(percentage, 0, 100, 0, topPWM);
  OCR2A = j;
  /* digitalWriteFast(PWMpin, percentage > 0 ? 0 : 1); */
}

void Sens() // ISR function excutes when push button at pinD2 is pressed
{
  pulses++;
}

/* Configure digital pins 9 and 10 as 16-bit PWM outputs. */
void setupPWM(uint16_t ICR)
{
  pinModeFast(PWMpin, OUTPUT);
  // digitalWriteFast(PWMpin, 1);

  TCCR2A = 0; // Clear TCCR1A register
  TCCR2B = 0; // Clear TCCR1B register

  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20);
  // TCCR1A = 0b10100010;

  TCCR2B = _BV(CS21);
  OCR2A = 0;
  // TCCR1B = 0b00011001;
}

void startTimer1()
{
  cli();
  // set timer1 interrupt at 1Hz
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 16000000 / (20 * 256) - 1; // = (16*10^6) / (40*64) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 and CS10 bits for 1024 prescaler
  TCCR1B |= (1 << CS12);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();
}

void stopTimer1()
{
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 0;
  TIMSK1 = 0;
  sei();
}