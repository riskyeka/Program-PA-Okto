#pragma once

#ifndef _LowPassFilter_h_
#define _LowPassFilter_h_ 

#include <cmath>

class LowPassFilter
{
public:
	LowPassFilter();
	LowPassFilter(float iCutOffFrequency, float iDeltaTime);
	//function
	float update(float input);
	float update(float input, float deltaTime, float cutoffFrequency);
	//get and configure functions
	float getOutput() const { return output;}
	void reconfigureFilter (float deltaTime, float cutoffFrequency);
private:
	float output;
	float ePow;
	
};
#endif