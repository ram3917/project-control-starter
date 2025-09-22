/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * Initialize PID coefficients (and errors, if needed)
   **/
   Kp_ = Kpi;
   Ki_ = Kii;
   Kd_ = Kdi;
   output_lim_max_ = output_lim_maxi;
   output_lim_min_ = output_lim_mini;
   
   p_error_ = 0.0;
   i_error_ = 0.0;
   d_error_ = 0.0;
   delta_time_ = 0.0;
}


/**
 * Updates the PID controller error terms based on the current cross-track error.
 * 
 * This method calculates and updates the three error components used in PID control:
 * - Proportional error (p_error_): Set to the current cross-track error
 * - Derivative error (d_error_): Calculated as the rate of change of error over time
 * - Integral error (i_error_): Accumulated sum of errors over time
 * 
 * @param cte The current cross-track error (difference between desired and actual position)
 */
void PID::UpdateError(double cte) {
   /**
   * Update PID errors based on cte.
   **/
   d_error_ = (cte - p_error_) / delta_time_;
   p_error_ = cte;
   i_error_ += cte * delta_time_;
}

double PID::TotalError() {
   /**
   *  Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
   double total_error = -Kp_ * p_error_ - Ki_ * i_error_ - Kd_ * d_error_;
      
   if (total_error > output_lim_max_) {
      return output_lim_max_;
   }
   if (total_error < output_lim_min_) {
      return output_lim_min_;
   }
   return total_error;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * Update the delta time with new value
   */
  delta_time_ = new_delta_time;
}