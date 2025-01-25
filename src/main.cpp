#include <Arduino.h>

// Definition of the input pins
const int PIN_CLK = 34;
const int PIN_DT = 35;
const int PIN_BTN = 32;

// Initialization of required variables
bool initialisation_complete = false;
u_int8_t ring_hour = 0;

#define DEBOUNCE_DELAY 300

unsigned modulo(int value, unsigned m)
{
  int mod = value % (int)m;
  if (mod < 0)
  {
    mod += m;
  }
  return mod;
}

void readEncoder()
{
  unsigned long static last_interrupt = 0;
  if (initialisation_complete && millis() - last_interrupt > DEBOUNCE_DELAY)
  {
    if (digitalRead(PIN_DT))
    {
      ring_hour = modulo(ring_hour + 1, 24);
      Serial.println(ring_hour);
    }
    else
    {
      ring_hour = modulo(ring_hour - 1, 24);
      Serial.println(ring_hour);
    }
    last_interrupt = millis();
  }
}

typedef enum
{
  NONE = 0,
  CLOCKWISE = 1,
  COUNTERCLOCKWISE = -1,
  INVALID = 2
} RotationDirection;

// https://daniellethurow.com/blog/2021/8/30/how-to-use-quadrature-rotary-encoders
// lookup table, first index is previous value
// second index is current value
// says if it's part of the sequence when moving
// clockwise (1) or counterclockwise (-1)
// didn't move (0) or skipped a value (2)
int lookupTable[4][4] = {{NONE, COUNTERCLOCKWISE, CLOCKWISE, INVALID},
                         {CLOCKWISE, NONE, INVALID, COUNTERCLOCKWISE},
                         {COUNTERCLOCKWISE, INVALID, NONE, CLOCKWISE},
                         {INVALID, CLOCKWISE, COUNTERCLOCKWISE, NONE}};

void readEncoderSave()
{
  static unsigned int clockState = 0;
  static unsigned int counterClockState = 0;
  static int prevVal = 3;

  int newVal = (digitalRead(PIN_CLK) << 1) + digitalRead(PIN_DT);

  int info = lookupTable[prevVal][newVal];
  if (info == CLOCKWISE)
  {
    clockState |= (1 << newVal);
  }
  else if (info == COUNTERCLOCKWISE)
  {
    counterClockState |= (1 << newVal);
  }
  else if (info == INVALID)
  {
    Serial.println("skipped a value");
  }

  if (prevVal != newVal && newVal == 3)
  {
    // changed to the non moving state, lets figure out what direction we went!

    // for each clockwise and counterclockwise, the encoder state goes through 4 distinct states
    // make sure it's gone through at least 3 of those (and assume if one is missing it's because I didn't read fast enough)
    if (clockState == 0b1011 || clockState == 0b1101 || clockState == 0b1110 || clockState == 0b1111)
    {
      ring_hour = modulo(ring_hour + 1, 24);
      Serial.println(ring_hour);
    }
    if (counterClockState == 0b1011 || counterClockState == 0b1101 || counterClockState == 0b1110 || counterClockState == 0b1111)
    {
      ring_hour = modulo(ring_hour - 1, 24);
      Serial.println(ring_hour);
    }

    clockState = 0;
    counterClockState = 0;
  }

  prevVal = newVal;
}

void readButton()
{
  unsigned long static last_interrupt = 0;
  if (initialisation_complete && millis() - last_interrupt > DEBOUNCE_DELAY)
  {
    Serial.print("Wecker um ");
    Serial.print(ring_hour);
    Serial.println(" Uhr gesetzt. ");
    ring_hour = 0;
    last_interrupt = millis();
  }
}

void setup()
{
  pinMode(PIN_CLK, INPUT_PULLUP);
  pinMode(PIN_DT, INPUT_PULLUP);
  pinMode(PIN_BTN, INPUT_PULLUP);

  // simple readEncoder
  // attachInterrupt(digitalPinToInterrupt(PIN_CLK), readEncoder, FALLING);
  // readEncoderSave
  attachInterrupt(digitalPinToInterrupt(PIN_CLK), readEncoderSave, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_DT), readEncoderSave, CHANGE);

  attachInterrupt(digitalPinToInterrupt(PIN_BTN), readButton, RISING);
  Serial.begin(115200);
  initialisation_complete = true;
}

void loop()
{
}
