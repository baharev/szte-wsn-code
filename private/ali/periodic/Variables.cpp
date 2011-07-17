/* Copyright (c) 2011, University of Szeged
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* - Neither the name of University of Szeged nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
* OF THE POSSIBILITY OF SUCH DAMAGE.
*
*      Author: Ali Baharev
*/

#include "Variables.hpp"

namespace {

//---
// Calibration from April
//double accel_gain[] = {
//		0.0211877,    0.000196498, -0.000102888,
//		-0.000392152, 0.0217456,   -3.38839e-05,
//		0.000161889,  1.20452e-06,  0.0221908
//};
//
//double accel_offset[] = { -49.7793, -50.9205, -53.8249 };
//---

// Calibration in July
double accel_gain[] = {
		0.0212822,		-1.00586e-05,	-9.25613e-05,
		-0.000304954,	0.0217671,		-4.35519e-05,
		0.000313866,	-5.65814e-05,	0.0222094
};

double accel_offset[] = { -50.498, -51.0964, -54.3594 };

// Gyro is left-handed: y -> -y to make it right handed!!!
double gyro_gain[] = {
		0.00592978,  -2.96852e-05, -0.000268771,
	   -9.88625e-05, -0.00617284,  0.000253044,
		0.000914252, -0.00142428,   0.00620373
};

// Gyro is left-handed: y -> -y to make it right handed!!!

double gyro_offset[] = {-13.2207, 18.3094, -14.7302 }; // TODO Check bound inflation!

double v0[] = { 0.0, 0.0, 0.0 };


//double accel_gain[] = {
//		 1.0,   0.0,  0.0,
//		 0.0,   1.0,  0.0,
//		 0.0,   0.0,  1.0
//};
//
//double accel_offset[] = { 0.0, 0.0, 0.0 };
//
//double gyro_gain[] = {
//		1.0, 0.0, 0.0,
//	    0.0, 1.0, 0.0,
//		0.0, 0.0,-1.0
//};
//
//double gyro_offset[] = { 0.0, 0.0, 0.0 }; // TODO Check bound inflation!
//
//double v0[] = { 0, 0, 0 };

}

namespace gyro {

std::vector<int> Variables::current_periods = std::vector<int>();

void Variables::set_current_periods(const std::vector<int>& p, size_t i) {

	current_periods.assign(&p.at(i), &p.at(i+N_PERIODS)+1);

	int offset = current_periods.at(0);

	for (size_t k=0; k<current_periods.size(); ++k) {

		current_periods.at(k) -= offset;
	}
}

Variables::Variables() : PERIOD_END(current_periods) {

	ASSERT(PERIOD_END.size()==N_PERIODS+1);

	reset_period_position();

	set_intial_points();

	set_bounds();

	check_feasibility();
}

void Variables::set_intial_points() {

	push_back_3d_vector(x_0, v0);

	for (int i=0; i<N_PERIODS+1; ++i) {

		push_back_3d_vector(x_0, gyro_offset);
	}
}

void Variables::push_back_3d_vector(std::vector<double>& v, const double x[3]) {

	for (int i=0; i<3; ++i) {

		v.push_back( x[i] );
	}
}

void Variables::set_bounds() {

	for (int i=0; i<3; ++i) {

		x_L.push_back(x_0.at(i) - 5); // velocity [-5, 5] m/s^2
		x_U.push_back(x_0.at(i) + 5);
	}

	for (int i=3; i<N_VARS; ++i) {

		x_L.push_back(x_0.at(i) - 30); // gyro_offsets
		x_U.push_back(x_0.at(i) + 30);
	}
}

void Variables::check_feasibility() const {

	ASSERT(static_cast<int>(x_L.size())==N_VARS);
	ASSERT(static_cast<int>(x_U.size())==N_VARS);
	ASSERT(static_cast<int>(x_0.size())==N_VARS);

	for (int i=0; i<N_VARS; ++i) {

		ASSERT(x_L.at(i) < x_U.at(i));

		ASSERT(x_L.at(i) <= x_0.at(i) && x_0.at(i) <= x_U.at(i));
	}
}

const matrix3 Variables::accel_gain() const {

	return matrix3(::accel_gain);
}

const vector3 Variables::accel_offset() const {

	return vector3(::accel_offset);
}

const matrix3 Variables::gyro_gain() const {

	return matrix3(::gyro_gain);
}

}
