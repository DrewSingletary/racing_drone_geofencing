#ifndef __LOW_PASS_FILTER_H_INCLUDED__
#define __LOW_PASS_FILTER_H_INCLUDED__

#include "Arduino.h"

namespace Cyberpod
{

class LowPassFilter
{
public:
	struct DATA
	{
		float yRaw = 0.0F;
		float y = 0.0F;
		float yDotRaw = 0.0F;
		float yDot = 0.0F;
	};

	// Methods
	LowPassFilter(const float &tau,
	              const float &tauDer,
	              const float &dtMin = 0.001);

	void update(const int &new_time,
	            const float &new_val);

	// Attributes
	int tNm1_;
	float tau_;
	float tauDer_;
	float dtMin_;
	DATA data_;
    bool vel_init = 0;
};

}

#endif
