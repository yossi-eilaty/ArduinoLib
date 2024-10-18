#ifndef UltraSonicSensor_h
#define UltraSonicSensor_h

#include <Arduino.h>

class UltraSonicSensor {
private:
  int _trigPin;
  int _echoPin;

  static const int DISTANCES_FOR_CLEANUP = 5;
  volatile unsigned long _microsWhenSignalSent;
  volatile double _distance;
  volatile boolean _nowMeasuring = false;
  unsigned long _microsOfMeasureStart;
  volatile unsigned long _microsOfMeasureEnd;
  volatile float _distances[DISTANCES_FOR_CLEANUP];
  volatile float _total = 0;
  volatile int _newDistanceLocation = -1;

  static unsigned long microSecondsDiff(unsigned long from, unsigned long to);

  static UltraSonicSensor *sensorPointers[2];
  static void (*userFunc[])(void);
  static void isr0();
  static void isr1();

  void ultraSonicISR();
  void newDistanceFound(float distance);
public:
  UltraSonicSensor(int trigPin, int echoPin);
  void setup();
  void triggerMeasure();
  float getDistance();
  float getCleanDistance();
  unsigned long getDistMicros();
};

#endif