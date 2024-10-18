#include "TimePassed.h"
#include "ULongDiff.h"
#include <Arduino.h>

TimePassed::TimePassed() {
}

bool TimePassed::passedSinceLastCycle(unsigned long timeTocheck, bool resetIfPassed) {
	unsigned long now = getNow();
	bool passed = ulongDiff(lastMeasure, now) > timeTocheck;
	if (resetIfPassed && passed)
		reset();
	return passed;
}

void TimePassed::setup() {
  reset();
}

void TimePassed::reset() {
	lastMeasure = getNow();
}