#include <Arduino.h>
#include <digitalWriteFast.h>
#include <Keypad.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

const byte ROWS = 4; // four rows
const byte COLS = 3; // three columns
char keys[ROWS][COLS] = {
    {'1', '2', '3'},
    {'4', '5', '6'},
    {'7', '8', '9'},
    {'*', '0', '#'}};

byte rowPins[ROWS] = {11, 9, 8, 7}; // connect to the row pinouts of the keypad
byte colPins[COLS] = {6, 5, 4};     // connect to the column pinouts of the keypad

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
#define ledPin 13

// Type Variable
#define charLenght 5
uint8_t typeIndex = 0;
char typeArray[charLenght] = "";
const char empty[charLenght] = "";
uint32_t typeInteger = 0;

// PWM
#define PWMpin 10
uint16_t topPWM = 0x7fe;

// Pulse Variable
volatile uint16_t pulses = 0;

// Liter per Minute Calculation Variable
uint32_t LpM_calc_time = 0;
uint32_t last_LpM_calc_time = 0;
float LperMinute = 0;
float CallibrationFactor = 7.5;

// Milliliter per 500ms Calculation Variable
uint32_t mLp500_calc_time = 0;
uint32_t last_mLp500_calc_time = 0;
float mLp500 = 0;

// Water Volume Variable
double offsetVolume = 50.00;
double targetVolume = 0;
double currentVolume = 0;

// Filling System Variable
bool Filling = false;
bool Pumping = false;
uint8_t maxPower = 100;

// Parser Variabel
/* const char delimiter = ','; */

// Program Selector Variabel
#define IDLE 0xA1
#define FILLING 0xA2
#define PUMPING 0xA3
uint8_t Status = IDLE;

// fungtion declaration
void writePWM(uint8_t, boolean);
void Sens();
void setupPWM(uint16_t);
void _bitWrite();
void ControlSystem();
void Calculation();
void clearTypeArray();
void input2typeArray(char);
void typeArray2uint32(uint32_t *);
void typeHandler();
void display();
void EmergencyStop();
void Status2Idle();
void Status2filling(uint32_t);
void Status2pumping();
uint8_t isTypeArrayEmpty();
/* uint16_t *parser(String *, const char *); */

void setup()
{
    /* Serial.begin(115200);
    Serial.println("Pertanimi Basic Test"); */

    setupPWM(topPWM);
    pinModeFast(ledPin, OUTPUT);    // Sets the digital pin as output.
    digitalWriteFast(ledPin, HIGH); // Turn the LED on.
    u8g2.begin();

    // DDRD = 1 << PORTD4;

    attachInterrupt(0, Sens, FALLING); //  function for creating external interrupts at pin2 on Rising (LOW to HIGH)
}

void loop()
{
    /* while (Serial.available() > 0)
    {
      char recieved = Serial.read();
      static String inData = "";
      inData += recieved;

      // Process message when new line character is recieved
      if (recieved == '=')
      {
        inData.remove(inData.length() - 1);
        Serial.println(String("Nano Recieve = ") + String(inData));

        if (inData.startsWith("STOP"))
        {
          inData.remove(0, 4);
          Status2Idle();
        }

        if (inData.startsWith("F"))
        {
          inData.remove(0, 1);
          uint16_t *parsedData = parser(&inData, &delimiter);

          if (parsedData[0] > 0 && parsedData[1] > 0)
          {
            Status2filling(parsedData[0]);
            Serial.println(String("Filling Start with target ") + String(targetVolume) + String("ml"));
          }

          if (parsedData[1] > 0)
          {
            // Status2pumping();
            maxPower = parsedData[1];
            Serial.println("Pumping Start");
          }
        }

        inData = ""; // Clear recieved buffer
      }
    } */

    switch (Status)
    {
    case IDLE:
        typeHandler();

        break;
    case FILLING:
        EmergencyStop();
        ControlSystem();
        Calculation();
        break;
    case PUMPING:
        EmergencyStop();
        ControlSystem();
        Calculation();
        break;
    }

    display();

    //_bitWrite(&PORTD, PORTD4, 1);
    //_bitWrite(&PORTD, PORTD4, 0);
}

void display()
{
    static char flow_buffer[8] = "";
    static char f_current_buffer[8] = "";
    static char f_target_buffer[8] = "";
    static char f_flow_buffer[8] = "";
    //static char CurrentVolume[15] = "Current Volume";
    //static char TargetVolume[14] = "Target Volume";
    //static char CurrentFlow[6] = "Flow:";
    dtostrf(LperMinute, 2, 2, flow_buffer);

    sprintf(f_current_buffer, "%dmL", uint16_t(currentVolume));
    sprintf(f_target_buffer, "%dmL", uint16_t(targetVolume));
    sprintf(f_flow_buffer, "%sL/m", flow_buffer);

    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_profont12_tr);
        //u8g2.drawStr(64 - u8g2.getStrWidth(CurrentVolume) / 2, 8, CurrentVolume);
        //u8g2.drawStr(64 - u8g2.getStrWidth(TargetVolume) / 2, 34, TargetVolume);

        u8g2.setFont(u8g2_font_profont22_tr);
        u8g2.drawStr(64 - u8g2.getStrWidth(f_current_buffer) / 2, 24, f_current_buffer);
        u8g2.drawStr(64 - u8g2.getStrWidth(f_target_buffer) / 2, 52, f_target_buffer);

        u8g2.setFont(u8g2_font_profont15_tr);
        //u8g2.drawStr(0, 64, CurrentFlow);
        u8g2.drawStr(128 - u8g2.getStrWidth(f_flow_buffer), 64, f_flow_buffer);
    } while (u8g2.nextPage());
}

void typeHandler()
{
    char key = keypad.getKey();
    switch (key)
    {
    case '*':
        if (isTypeArrayEmpty())
        {
            /* Serial.println("typeArray empty, procced pumping"); */
            Status2pumping();
            break;
        }
        typeArray2uint32(&typeInteger);
        if (typeInteger < 100)
        {
            clearTypeArray();
            /* Serial.println("typeInteger is to low below 100"); */
        }

        clearTypeArray();
        /* Serial.print("typeInteger[mL]: ");
        Serial.println(typeInteger); */
        Status2filling(typeInteger);
        /* Serial.println("typeArray cleared"); */
        break;

    case '#':
        if (isTypeArrayEmpty())
        {
            /* Serial.println("typeArray empty"); */
            break;
        }

        clearTypeArray();
        /* Serial.println("typeArray cleared"); */
        break;

    default:
        if (key != '\0')
        {
            input2typeArray(key);
            /* Serial.print(key);
            Serial.println(" pressed"); */
        }
        break;
    }
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
}

void Status2filling(uint32_t inputValue)
{
    targetVolume = inputValue;
    Status = FILLING;
    currentVolume = 0.0;
    LperMinute = 0.0;
    last_LpM_calc_time = millis();
    last_mLp500_calc_time = millis();
    last_LpM_calc_time = LpM_calc_time - 250;
    last_mLp500_calc_time = mLp500_calc_time - 250;
    pulses = 0;
}

void Status2pumping()
{
    targetVolume = 0;
    Status = PUMPING;
    Pumping = true;
    currentVolume = 0.0;
    LperMinute = 0.0;
    LpM_calc_time = millis();
    mLp500_calc_time = millis();
    last_LpM_calc_time = LpM_calc_time - 250;
    last_mLp500_calc_time = mLp500_calc_time - 250;
    pulses = 0;
}

/* uint16_t *parser(String *data, const char *delimiter)
{
  static uint16_t parsedData[10] = {};
  String subString = "";
  uint8_t index = 0;
  uint8_t i = 0;

  while (*data != NULL)
  {
    index = data->indexOf(',');
    subString = data->substring(0, index);
    parsedData[i] = subString.toInt();
    data->remove(0, index + 1);
    Serial.println(parsedData[i]);
    i++;
  }
  return parsedData;
} */

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

        if (currentVolume >= targetVolume)
        {
            Status2Idle();
        }
    }
}

void Calculation()
{
    LpM_calc_time = millis();
    if (LpM_calc_time - last_LpM_calc_time >= 250)
    {
        noInterrupts();
        LperMinute = ((LpM_calc_time - last_LpM_calc_time) / 250.0) * (pulses / CallibrationFactor * 4);
        last_LpM_calc_time = LpM_calc_time;
        pulses = 0;
        interrupts();
    }

    mLp500_calc_time = millis();
    if (mLp500_calc_time - last_mLp500_calc_time >= 50)
    {
        mLp500 = ((mLp500_calc_time - last_mLp500_calc_time) / 50) * ((LperMinute / 60) * 1000) / 20;
        last_mLp500_calc_time = mLp500_calc_time;

        currentVolume += mLp500;
        /* Serial.print("LperMinute[L/m]: ");
        Serial.print(LperMinute, 2);
        Serial.print(" -- Volume[mL]: ");
        Serial.println(currentVolume, 2); */
    }
}

void clearTypeArray()
{
    typeIndex = 0;
    memset(typeArray, '\0', sizeof(typeArray));
}

void input2typeArray(char input)
{
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

    OCR1A = j;
    OCR1B = j;
}

void Sens() // ISR function excutes when push button at pinD2 is pressed
{
    pulses++;
}

void _bitWrite(volatile uint8_t *var, uint8_t bitPosition, uint8_t value)
{
    if (bitPosition >= 0 && bitPosition < 8)
    { // Assuming a 16-bit variable
        uint8_t mask = 1 << bitPosition;
        if (value)
        {
            *var |= mask;
        }
        else
        {
            *var &= ~mask;
        }
    }
}

/* Configure digital pins 9 and 10 as 16-bit PWM outputs. */
void setupPWM(uint16_t ICR)
{
    pinModeFast(PWMpin, OUTPUT);

    TCCR1A = 0; // Clear TCCR1A register
    TCCR1B = 0; // Clear TCCR1B register

    TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
    // TCCR1A = 0b10100010;

    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    // TCCR1B = 0b00011001;

    // Set the TOP value to determine the PWM frequency (e.g., 1000 for 1 kHz)
    ICR1 = ICR;
}