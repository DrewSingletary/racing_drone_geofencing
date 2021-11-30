#include "LowPassFilter.h"

namespace Cyberpod
{
	// Default Constructor
	LowPassFilter::LowPassFilter(const float &tau,
	                             const float &tauDer,
	                             const float &dtMin):
	tNm1_(-1.),
	tau_(tau),
	tauDer_(tauDer),
	dtMin_(dtMin),
	data_()
	{
		if(tau_<dtMin_)
			tau_ = dtMin_;

		if(tauDer<dtMin_)
			tauDer_ = dtMin_;
	}

	// Update the time and value vectors
	void LowPassFilter::update(const int &new_time,
	                           const float &new_val)
	{
		if(tNm1_<0.)
		{
      data_.y = new_val;
			data_.yRaw = new_val;
			tNm1_ = new_time;
		}
		else
		{
			const float dtNow = (float)(new_time-tNm1_)*1e-6;
			if(dtNow>dtMin_)
			{
				data_.yDotRaw = -(data_.yRaw - new_val)/dtNow;
                if (!vel_init)
                {
                    vel_init = 1;
                    data_.yDot = data_.yDotRaw;
                }
				data_.yRaw = new_val;
				tNm1_ = new_time;

				if(dtNow>tau_)
					data_.y = data_.yRaw;
				else
					data_.y += dtNow*(data_.yRaw-data_.y)/tau_;

				if(dtNow>tauDer_)
					data_.yDot = data_.yDotRaw;
				else
					data_.yDot += dtNow*(data_.yDotRaw-data_.yDot)/tauDer_;
			}
		}
	}
}
