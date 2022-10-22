#include "LowPassFilter.h"

#define ERROR_CHECK (true)

#if ERROR_CHECK
#include <iostream>
#endif

	static int n;
LowPassFilter::LowPassFilter():
	output(0),
	ePow(){}

LowPassFilter::LowPassFilter(float iCutOffFrequency, float iDeltaTime):
	output(0),
	ePow(1-exp(-iDeltaTime * 44/7 * iCutOffFrequency))
	{
		#if ERROR_CHECK
		if(iDeltaTime <= 0){
			std::cout << " Warning : A LowPassFilter instance has been configured with 0 s as delta time.";
			ePow = 0;

		}
		if(iCutOffFrequency <= 0){
			std::cout <<" Warning : A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
			ePow = 0;
		}
		#endif 
	}

float LowPassFilter::update(float input){
	n++;
	return output += (input - output) * ePow;
}

float LowPassFilter::update(float input, float deltaTime, float cutoffFrequency){
	reconfigureFilter(deltaTime, cutoffFrequency); //changes ePow accordingly.
	return output += (input - output) * ePow;
}

void LowPassFilter::reconfigureFilter(float deltaTime, float cutoffFrequency){
	#if ERROR_CHECK
	if (deltaTime <= 0 ){
		std::cout << " Warning : A LowPassFilter instance has been configured with 0 s as delta time.";
		ePow = 0;
	}
	if (cutoffFrequency <= 0 ){
		std::cout << " Warning : A LowPassFilter instance has been configured with 0 Hz as cut-off frequency.";
		ePow = 0;
	}
	#endif
	ePow = 1-exp(-deltaTime * 44/7 * cutoffFrequency);
}

