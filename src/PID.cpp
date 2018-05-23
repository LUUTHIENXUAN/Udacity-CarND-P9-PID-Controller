#include "PID.h"
#include <uWS/uWS.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	p_error = 0;
	i_error = 0;
	d_error = 0;

  //run() function
	max_time_step  = 3; //5
	err            = 0;
	best_error     = 0;

  // Twiddle status
	twiddle_intialized = false;
  twiddle_completed  = false;

	// Set size params & dparams for Twiddle
	int n_params = 3;
  dparams.resize(n_params);
  params.resize(n_params);

  // Initialize params & dparams variable.
	dparams[0]  = 0.001;
	dparams[1]  = 0.000000001;
	dparams[2]  = 1.1;

	params[0]  = Kp;
	params[1]  = Ki;
	params[2]  = Kd;

	current_stage  = 0;
	index          = 0;

}

void PID::UpdateError(double cte) {

	///* diff_crosstrack_error
	d_error  = cte - p_error;
	///* crosstrack_error
	p_error  = cte;
	///* int_crosstrack_error
	i_error += cte;

}

double PID::TotalError() {

  std::cout << "Kp, Ki, Kd: " << Kp << ", "
	                            << Ki << ", "
															<< Kd << std::endl;

	return -Kp*p_error - Ki*i_error - Kd*d_error;
}

// return average average cross track error just like python run()
double PID::run(double cte, int current_time_step){

	err += cte*cte;

	if (current_time_step == max_time_step) {

		double average_err = err/max_time_step; //average cross track error
		err = 0;

		return average_err;
	}
	else return err;
}


// Instead of do run() in for loop we divide it into 4 stages
// Input err taken from run function

void PID::twiddle(double tol, double err){

  if (!twiddle_intialized) {

		best_error = err;
		std::cout << "Initializing best_error..." << best_error <<std::endl;
		// done initializing, no need to update uptate Twiddle
    twiddle_intialized = true;
    return;
	}

	if ((dparams[0] + dparams[1] + dparams[2]) < tol) {

		std::cout << "Twiddle completed! "
		          << (dparams[0] + dparams[1] + dparams[2]) << " < " << tol
							<< std::endl;
    std::cout << " Final Result:" << " Kp = " << params[0]
		                              << " Ki = " << params[1]
																	<< " Kd = " << params[2]
																	<< std::endl;
		twiddle_completed = true;

	}

	switch (current_stage) {

		case 0: {

			params[index]  += dparams[index];
			current_stage  += 1 ;

		}break;

		case 1: {

			double error_1 = err;
			if (error_1 < best_error){
				best_error      = error_1;
				dparams[index] *= 1.1;
			}
			else{
				params[index]  -= 2.0*dparams[index];
			}

			current_stage  += 1;

		}break;

		case 2: {

			double error_2 = err;
			if (error_2 < best_error){
				best_error      = error_2;
				dparams[index] *= 1.1;
			}
			else{
				params[index]  += dparams[index];
				dparams[index] *= 0.9;
			}

			//return to case 0
			current_stage = 0;
			//increase index
			index = (index < 2) ? index += 1 : 0;
		}break;

	}

	///* Update coefficients
	Kp = params[0];
	Ki = params[1];
	Kd = params[2];

}
