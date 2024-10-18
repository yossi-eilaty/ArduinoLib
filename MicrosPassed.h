#ifndef MicrosPassed_h
#define MicrosPassed_h

#include <Arduino.h>
#include <TimePassed.h>

class MicrosPassed : public TimePassed {
protected:
  inline unsigned long getNow() {
    return micros();
  }
};

#endif
