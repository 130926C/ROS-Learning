
#include "Pid.h"
#include <math.h>
#include <iostream>
Pid::Pid() {
	ctrl_period_ = 0.05;
	derivative_ = 0;
	frequency_ = 100;
	integ_ = 0;
	integ_limit_ = 10;
	kd_ = 0;
	ki_ = 0;
	kp_ = 1;
	error_pre_ = 0;
	error_ = 0;
	output_ = 0;
	output_limit_ = 10;
	//tolerance_ = 10;
}

Pid::~Pid() {
}

void Pid::init(double ctrl_period){
	ctrl_period_ = ctrl_period;
	frequency_ = 1.0 / ctrl_period;
	integ_limit_ = 1;
	kd_ = 0;
	ki_ = 0;
	kp_ = 1;
	error_ = 0;
	output_ = 0;
	output_limit_ = 1;
	//tolerance_ = 0.01;
	// filter_error_.resize(3);
	// unfilter_error_.resize(3);
	// filter_error_.clear();
	// unfilter_error_.clear();

	derivative_ = 0;
	integ_ = 0;
	error_pre_ = 0;

}
void Pid::clear(){
	derivative_ = 0;
	integ_ = 0;
	error_pre_ = 0;
}

double Pid::calc(double set_point, double feedback){
	error_ = set_point - feedback;

	integ_ += ki_ * error_;
	if(integ_ > integ_limit_){
		integ_ = integ_limit_;
	}else if(integ_ < -integ_limit_){
		integ_ = -integ_limit_;
	}else{}
	
	derivative_ = kd_ * (error_ - error_pre_) ;
	
	output_ = kp_ * error_ + integ_ + derivative_;
	error_pre_  = error_;
	return output_;
}
double Pid::calc2(double set_point, double feedback){
	error_ = set_point - feedback;

	integ_ += ki_ * error_ ;
	if(integ_ > integ_limit_){
		integ_ = integ_limit_;
	}else if(integ_ < -integ_limit_){
		integ_ = -integ_limit_;
	}else{}
	
	derivative_ = kd_ * (error_ - error_pre_) ;
	
	output_ = kp_ * error_ * fabs(error_)+ integ_ + derivative_;
	error_pre_  = error_;
	return output_;
}