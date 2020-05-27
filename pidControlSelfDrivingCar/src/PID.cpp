#include "PID.h"

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
	
}

void PID::UpdateError(double cte) {

	this->p_error = -(this->Kp)*cte;
	
	// Derivative error -> Need to find the gradient here.
	double diff_cte = cte - (this->prev_cte);
	this->prev_cte = cte;
	this->d_error = -(this->Kd)*diff_cte;
	
	// integral Error -> Need to find the sum here. 
	this->sum_cte += cte;
	this->i_error = -(this->Ki)*sum_cte;
	
}

double PID::TotalError() {

    return (this->p_error) + (this->i_error) + (this->d_error);
    
}

