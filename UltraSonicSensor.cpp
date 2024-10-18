#include "Arduino.h"
#include "UltraSonicSensor.h"

#define SPEED_OF_SOUND_IN_AIR_IN_CM_PER_SECOND 34300
#define MICROS_PER_CM (1000000.0 / SPEED_OF_SOUND_IN_AIR_IN_CM_PER_SECOND)
#define ULTRA_SOUND_FREQ 40000
#define NUM_CYCLES 8
#define MICROS_FOR_PING ((1000000 / ULTRA_SOUND_FREQ) * NUM_CYCLES)
#define MAX_RANGE_CM 400
#define MIN_RANGE_CM 2
#define MIN_MICROS_BETWEEN_MEASURES 2
#define DIST_BETWEEN_TRANS_AND_RECV_CM 2.2
#define CALIBRATION_CM -0.14

// Calculates the difference in micro seconds between two calls to micros().
// We need to use this function instead of simple subtruction because it handles the case
// of the counter overflow, which happens every 70 minutes
static unsigned long UltraSonicSensor::microSecondsDiff(unsigned long from, unsigned long to) {
  if (to > from)
    return to - from;
  else
    return (((unsigned long)-1L) - from) + to + 1;
}

static UltraSonicSensor *UltraSonicSensor::sensorPointers[2];

static void UltraSonicSensor::isr0() {
  sensorPointers[0]->ultraSonicISR();
}

static void UltraSonicSensor::isr1() {
  sensorPointers[1]->ultraSonicISR();
}

static void (*UltraSonicSensor::userFunc[])(void) = { UltraSonicSensor::isr0, UltraSonicSensor::isr1 };

UltraSonicSensor::UltraSonicSensor(int trigPin, int echoPin) {
  _trigPin = trigPin;
  _echoPin = echoPin;
  for (int i = 0; i < DISTANCES_FOR_CLEANUP; ++i)
    _distances[i] = -1;
}

void UltraSonicSensor::setup() {
  _distance = -1.0;
  _microsOfMeasureEnd = micros();

  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
  digitalWrite(_trigPin, LOW);
  sensorPointers[digitalPinToInterrupt(_echoPin)] = this;
  attachInterrupt(digitalPinToInterrupt(_echoPin), userFunc[digitalPinToInterrupt(_echoPin)], CHANGE);
  triggerMeasure();
}

// This is the interrupt handler for the ultrasonic echo pin.
// It measures the time in microseconds from the time the pin goes up until it
// goes down and then uses this time to calculate a new distance value
void UltraSonicSensor::ultraSonicISR() {
  // It is important that the following line will always be the first line of the ISR
  // in order to get the most accurate measure
  unsigned long now = micros();
  if (digitalRead(_echoPin) == HIGH) {
    _microsWhenSignalSent = now;
    // Serial.println("S");
  } else {
    float potentialDistance = sqrt(sq((microSecondsDiff(_microsWhenSignalSent, now) >> 1) / MICROS_PER_CM) - sq(DIST_BETWEEN_TRANS_AND_RECV_CM)) + CALIBRATION_CM;
    _nowMeasuring = false;
    _microsOfMeasureEnd = now;
    // Serial.print(now);
    // Serial.print(": ");
    // Serial.println("F");
    if (potentialDistance >= MIN_RANGE_CM && potentialDistance <= MAX_RANGE_CM)
      newDistanceFound(potentialDistance);
  }
}

void UltraSonicSensor::newDistanceFound(float distance) {
  if (_newDistanceLocation > -1) {
    _distance = distance;
    _total = _total - _distances[_newDistanceLocation] + distance;
    _distances[_newDistanceLocation] = distance;
    _newDistanceLocation = (_newDistanceLocation + 1) % DISTANCES_FOR_CLEANUP;
    // Serial.println(_newDistanceLocation);
  } else {
    _newDistanceLocation = 0;
    for (int i = 0; i < DISTANCES_FOR_CLEANUP; ++i)
      _distances[i] = distance;
    _total = distance * DISTANCES_FOR_CLEANUP;
  }
}


// This action efficiently triggers a new ultrasonic sensor measurement, introducing a delay so
// small it's practically irrelevant (about 12 microseconds at most).
void UltraSonicSensor::triggerMeasure() {
  if (microSecondsDiff(_microsOfMeasureEnd, micros()) < MIN_MICROS_BETWEEN_MEASURES)
    return;
  if (!_nowMeasuring || microSecondsDiff(_microsOfMeasureStart, micros()) > MICROS_PER_CM * MAX_RANGE_CM * 2.2 + MICROS_FOR_PING) {
    _nowMeasuring = true;
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
    _microsOfMeasureStart = micros();
    // Serial.print(_microsOfMeasureStart);
    // Serial.print(": ");
    // Serial.println("T");
  }
}

float UltraSonicSensor::getDistance() {
  return _distance;
}

float UltraSonicSensor::getCleanDistance() {
  // Serial.print("total=");
  // Serial.print(_total);
  // Serial.print(" [");
  // for (int i = 0; i < DISTANCES_FOR_CLEANUP - 1; ++i) {
  //   Serial.print(_distances[i]);
  //   Serial.print(",");
  // }
  // Serial.print(_distances[DISTANCES_FOR_CLEANUP - 1]);
  // Serial.print("] => ");
  float avg = _total / ((float)DISTANCES_FOR_CLEANUP);
  float maxDiff = avg;
  for (int i = 0; i < DISTANCES_FOR_CLEANUP; ++i)
    if (abs(_distances[i] - avg) > abs(maxDiff - avg))
      maxDiff = _distances[i];
  float rv = (_total - maxDiff) / (DISTANCES_FOR_CLEANUP - 1.0);
  // Serial.println(rv);
  return rv;
}