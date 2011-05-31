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

#include <limits>
#include <stdexcept>
#include "VarEstimates.hpp"

namespace {

double accel_gain[] = {
		0.0211877,    0.000196498, -0.000102888,
		-0.000392152, 0.0217456,   -3.38839e-05,
		0.000161889,  1.20452e-06,  0.0221908
};

double accel_offset[] = { -49.7793, -50.9205, -53.8249 };

// Gyro is left-handed: y -> -y to make it right handed!!!
double gyro_gain[] = {
		0.00592978,  -2.96852e-05, -0.000268771,
	   -9.88625e-05, -0.00617284,  0.000253044,
		0.000914252, -0.00142428,   0.00620373
};

// Gyro is left-handed: y -> -y to make it right handed!!!
double gyro_offset[] = {-13.2952, 17.8362, -14.1019 };

double initial_orientation[] = { 0.0, 0.0, 0.0 };

double v0[] = { 0.0, 0.0, 0.0 };

}

namespace gyro {

VarEstimates::VarEstimates() {

	set_everything_incorrect();

	set_intial_points();

	set_bounds();

	check_feasibility();
}

void VarEstimates::set_everything_incorrect() {

	const double DOUBLE_MAX = std::numeric_limits<double>::max();

	for (int i=0; i<N_VARS; ++i) {
		x_L[i] = DOUBLE_MAX;
		x_U[i] =-DOUBLE_MAX;
		x_0[i] = std::numeric_limits<double>::quiet_NaN();
	}
}

void VarEstimates::copy_to_initial_point(const double array[], const VarEnum begin, const VarEnum end) {

	for (int i=begin; i<=end; ++i) {

		x_0[i] = array[i-begin];
	}
}

void VarEstimates::set_intial_points() {

	copy_to_initial_point(accel_gain, A11, A33);

	copy_to_initial_point(accel_offset, B1, B3);

	copy_to_initial_point(gyro_gain, C11, C33);

	copy_to_initial_point(gyro_offset, D1, D3);

	copy_to_initial_point(initial_orientation, EULER_X, EULER_Z);

	copy_to_initial_point(v0, VX, VZ);
}

void VarEstimates::set_bounds_by_abs_inflation(const VarEnum begin, const VarEnum end, double amount) {

	for (int i=begin; i<=end; ++i) {

		x_L[i] = x_0[i] - amount;
		x_U[i] = x_0[i] + amount;
	}
}

void VarEstimates::set_bounds() {

	set_bounds_by_abs_inflation(A11, A33, 0.01);

	set_bounds_by_abs_inflation(B1, B3, 20.0);

	set_bounds_by_abs_inflation(C11, C33, 0.003);

	set_bounds_by_abs_inflation(D1, D3, 30.0);

	set_bounds_by_abs_inflation(EULER_X, EULER_Z, 1.0);

	set_bounds_by_abs_inflation(VX, VZ, 1.0);
}

void VarEstimates::check_feasibility() {

	for (int i=0; i<N_VARS; ++i) {

		if (x_L[i] >= x_U[i]) {

			throw std::logic_error("incorrect variable bounds");
		}

		if (!(x_L[i] <= x_0[i] && x_0[i] <= x_U[i])) {

			throw std::logic_error("initial estimate is out of range");
		}
	}
}

}
