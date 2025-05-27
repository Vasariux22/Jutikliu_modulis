#ifndef BUTTONHANDLER_H
#define BUTTONHANDLER_H

#include <Arduino.h>

class ButtonHandler {
  public:

    ButtonHandler(int pin);
    void update();
    bool isPressed();
    bool isLongPressed(unsigned long holdTime);

  private:
    int _pin;
    int _currentState;
    int _previousState;
    bool _pressed;
    unsigned long _pressTime;
    bool _longPressTriggered;
};

#endif // BUTTONHANDLER_H