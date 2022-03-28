#ifndef ExtIntFreqCount_h
#define ExtIntFreqCount_h

#include <Arduino.h>

class ExtIntFreqCountClass {
public:
	#if(!defined(__IMXRT1062__))
		static void begin(uint16_t msec);
	#else
		static void begin(int checkPin, int pulseDir);
	#endif
	static uint8_t available(void);
	static uint32_t read(void);
	static void end(void);
};

extern ExtIntFreqCountClass ExtIntFreqCount;

#endif
