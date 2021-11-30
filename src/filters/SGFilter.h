#ifndef __SG_FILTER_H_INCLUDED__
#define __SG_FILTER_H_INCLUDED__

#include <Eigen.h>
#include <Eigen/Dense>
#include <Eigen/LU>

#include "Arduino.h"

namespace Cyberpod
{

	class SGFilter
	{
	public:
        struct DATA 
		{
			float yRaw = 0.0F;
            float y = 0.0F;
            float yDot = 0.0F;
		};

        // Methods
        SGFilter(uint32_t poly_order, uint32_t filter_size);

        void update(float new_time_float, float new_val);
        void debug_print_A(void);
        float powFast(float x, uint32_t n);    

        // Attributes
        uint32_t poly_order_;
        uint32_t filter_size_;

        uint32_t filter_iter_;
        uint32_t current_line_;

        float t0_, tRange_;

        Eigen::VectorXf f_; // Vector of function values 
        Eigen::VectorXf t_; // Vector of time slots
        Eigen::VectorXf tNorm_; // Vector of normalizedtime slots
        Eigen::MatrixXf A_; // Matrix of Least-Squares coefficients 
        Eigen::VectorXf c_; // Vector of Polynomial coefficients
        DATA data_;
    };
}

#endif