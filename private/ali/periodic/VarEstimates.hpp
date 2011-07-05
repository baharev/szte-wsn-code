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

#ifndef VARESTIMATES_HPP_
#define VARESTIMATES_HPP_

#include <cassert>
#include <vector>
#include "MatrixVector.hpp"

namespace gyro {

const int PERIOD_LENGTH = 200;

const int N_PERIODS = 6;

const int N_SAMPLES = PERIOD_LENGTH*N_PERIODS; // TODO Check samples.size() == N_SAMPLES

const int N_VARS = 3*(1+(N_PERIODS+1)); // (3 coordinates)*(v_initial + gyro_offsets)

const int N_CONS = 3*N_PERIODS;

class VarEstimates {

public:

	VarEstimates();

	const double* lower_bounds()  const { return &x_L.at(0); }
	const double* upper_bounds()  const { return &x_U.at(0); }
	const double* initial_point() const { return &x_0.at(0); }

	const matrix3 accel_gain() const;
	const vector3 accel_offset() const;
	const matrix3 gyro_gain() const;

	template <typename T>
	const Vector<T> initial_velocity(const T* x) {

		return Vector<T>(x);
	}

	void reset_period_position() { period = 1; }

	template <typename T>
	const Vector<T> next_gyro_offset(const T* x) {

		const int index = 3*period;

		assert(index<N_VARS);

		++period;

		return Vector<T>(x+index);
	}

private:

	void set_intial_points();

	void push_back_3d_vector(std::vector<double>& v, const double x[3]);

	void set_bounds();

	void check_feasibility() const;

	std::vector<double> x_L;
	std::vector<double> x_U;
	std::vector<double> x_0;

	const std::vector<int> PERIOD_END;

	int period;
};

}

#endif // VARESTIMATES_HPP_
