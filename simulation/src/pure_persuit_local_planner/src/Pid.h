/*
 * Pid.h
 *
 *  Created on: Oct 17, 2016
 *      Author: joey
 */

#ifndef SRC_PID_H_
#define SRC_PID_H_

#include <vector>

class Pid {
public:
	Pid();
	virtual ~Pid();
	void init(double ctrl_period);
	double calc(double set_point, double feedback);
	double calc2(double set_point, double feedback);
	void clear();

	double kp_;
	double  ki_;
	double kd_;
	double tolerance_;
	double output_limit_;
//private:
	double error_;
	double output_;
	double integ_;
	double integ_limit_;
	double frequency_;
	double error_pre_;
	double derivative_;
	double ctrl_period_;	//in seconds

	std::vector<double> filter_error_;
	std::vector<double> unfilter_error_;

};

#endif /* SRC_PID_H_ */