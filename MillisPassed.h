#ifndef MillisPassed_h
#define MillisPassed_h

#include <Arduino.h>
#include <TimePassed.h>

class MillisPassed : public TimePassed {
protected:
  inline unsigned long getNow() {
    return millis();
  }
};

#endif
