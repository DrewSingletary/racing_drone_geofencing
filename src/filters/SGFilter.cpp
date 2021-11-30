#include "SGFilter.h"

namespace Cyberpod
{
    // Default Constructor
    SGFilter::SGFilter(uint32_t poly_order, uint32_t filter_size) {

        // Always guarantee unique solution by imposing this condition
        if (poly_order + 1 > filter_size) {
            while(1) {
                Serial.println("SGFilter could not be initialized. Order should be "
                    "smaller than number of filtered points");
                delay(1000);
            }
        }

        // Assign attribute variables 
        poly_order_ = poly_order;
        filter_size_ = filter_size;

        current_line_ = 0;
        filter_iter_ = 0;

        A_.resize(filter_size_, poly_order_ + 1);

        // Initialize all vectors to zeros
        f_.resize(filter_size_);
        t_.resize(filter_size_);
        tNorm_.resize(filter_size_);
        c_.resize(poly_order_ + 1);

        for (unsigned int i = 0; i < filter_size_; i++) {
            f_(i) = 0.0f;
            t_(i) = 0.0f;
            tNorm_(i) = 0.0f;
        }

        //construct_A();
        
    }

    // Update the time and value vectors
    void SGFilter::update(float new_time_float, float new_val) {

        f_(current_line_) = new_val;
        t_(current_line_) = new_time_float;


        uint32_t line_np1 = current_line_;
        line_np1++;
        line_np1 = line_np1 % filter_size_;

        t0_ = t_(line_np1);
        tRange_ = new_time_float - t0_;

        for (unsigned int i = 0; i < filter_size_; i++) {
            tNorm_(i) = t_(i) - t0_;
            // tNorm_(i) = tNorm_(i)/tRange_;
        }

        // Push one slot forward
        for (uint32_t i = 0; i < filter_size_; i++)
        {
            for (uint32_t j = 0; j < poly_order_ + 1; j++) {           
                A_(i,j) = powFast(tNorm_(i), j);
            }
        }

        if(filter_iter_ > filter_size_) {
            // Update the matrix and solve the least-squares prolem 
            // c_ = (A_.transpose() * A_).ldlt().solve(A_.transpose() * f_);   
            c_ = A_.fullPivHouseholderQr().solve(f_);

            // Reinitialize data 
            data_.yRaw = new_val;
            data_.y = 0.0f;
            data_.yDot = 0.0f;

            float tEnd = tNorm_(current_line_);
            
            // Update data filtered
            for (unsigned int i = 0; i <= poly_order_; i++) {
                data_.y += c_(i) * powFast(tEnd, i);
            }
            // Update data derivative
            for (unsigned int i = 1; i <= poly_order_; i++) {
                data_.yDot += i * c_(i) * powFast(tEnd, i - 1);
            }
        }

        filter_iter_++;
        current_line_++;
        current_line_ = current_line_ % filter_size_;
    }

    // Debugging Function: Print out Coefficient Matrix
    void SGFilter::debug_print_A(void) {

        unsigned int nrow = A_.rows();
        unsigned int ncol = A_.cols();

        // Serial.print("nrow: "); Serial.println(nrow);
        // Serial.print("ncol: "); Serial.println(ncol);
        // Serial.println();

        for (unsigned int i = 0; i<nrow; i++)
        {
            for (unsigned int j=0; j<ncol; j++)
            {
                Serial.print(A_(i,j), 6);   // print 6 decimal places
                Serial.print(", ");
            }
            Serial.println();
        }
        Serial.println();
    }

    float SGFilter::powFast(float x, uint32_t n)
    {
        if(n==0)
        {
            return 1.0F;
        }
        else if(n==1)
        {
            return x;
        }
        else
        {
            float out = x;
            for(uint32_t i = 1; i<n; i++)
            {
                out*=x;
            }
            return out;
        }
    }
}