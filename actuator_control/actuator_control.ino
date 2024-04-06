#include <Servo.h>
#include <InputDebounce.h>

#define BUTTON_DEBOUNCE_DELAY   20   // [ms]

Servo myservo;
const int PowerPin = 6;
const int forwardPin = 5;
const int backPin = 3;

int microseconds = 1000;

InputDebounce forwardButton;
InputDebounce backwardButton;

void setup()
{
  forwardButton.registerCallbacks([](uint8_t){microseconds = min(2000, microseconds + 100);}, NULL);
  backwardButton.registerCallbacks([](uint8_t){microseconds = max(1000, microseconds - 100);}, NULL);

  forwardButton.setup(forwardPin, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);
  backwardButton.setup(backPin, BUTTON_DEBOUNCE_DELAY, InputDebounce::PIM_EXT_PULL_DOWN_RES);

  // Serial.begin(9600);
  myservo.attach(7);
  pinMode(PowerPin, OUTPUT);
  digitalWrite(PowerPin, HIGH);

  pinMode(forwardPin, INPUT);
  pinMode(backPin, INPUT);
}

void loop()
{
  unsigned long now = millis();
  forwardButton.process(now);
  backwardButton.process(now);

  myservo.writeMicroseconds(microseconds);
}