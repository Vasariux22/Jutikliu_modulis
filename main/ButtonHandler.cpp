#include "ButtonHandler.h"


ButtonHandler::ButtonHandler(int pin) {
  _pin = pin;
  pinMode(_pin, INPUT_PULLDOWN);
  _currentState = LOW;
  _previousState = LOW; 
  _pressed = false;
  _pressTime = 0;
  _longPressTriggered = false;
}

void ButtonHandler::update() {

  pinMode(_pin, INPUT_PULLDOWN);
  _currentState = digitalRead(_pin);

  if ((_currentState == HIGH) && (_previousState == LOW)) {
    _pressed = true;
    _pressTime = millis();
    _longPressTriggered = false;
  }

  else if (_currentState == LOW && _previousState == HIGH) {
    _pressed = false;
  }

  _previousState = _currentState;
}

bool ButtonHandler::isPressed() {

  if (_pressed && !_longPressTriggered) {
    _pressed = false;
    return true;
  }
  return false;
}

bool ButtonHandler::isLongPressed(unsigned long holdTime) {
  if (_currentState == HIGH && (millis() - _pressTime >= holdTime)) {

    if (!_longPressTriggered) {
      _longPressTriggered = true; 
      return true;
    }
  }
  return false;
}