#ifndef TimePassed_h
#define TimePassed_h

class TimePassed {
private:
	unsigned long lastMeasure;
protected:
	virtual unsigned long getNow() = 0;
public:
	TimePassed();
	bool passedSinceLastCycle(unsigned long timeTocheck, bool resetIfPassed);
  void setup();
	void reset();
};

#endif
