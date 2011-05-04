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

#ifndef ELLIPSOIDOBJECTIVE_HPP_
#define ELLIPSOIDOBJECTIVE_HPP_

#include <cmath>
#include <vector>
#include "MatrixVector.hpp"
#include "StaticSample.hpp"

namespace gyro {

template <typename T>
class EllipsoidObjective {

public:

	EllipsoidObjective(const std::vector<StaticSample>& samples)
	: samples(samples)
	{
#ifdef ACCEL_CALIB
		sample_at = &EllipsoidObjective::accel_at;
#elif defined MAGNETO_CALIB
		sample_at = &EllipsoidObjective::magn_at;
#else
#error Either accelerometer / megnetometer calibration should be chosen!
#endif
	}

	T f(const T* const x);

	double max_error(const double* const x);

private:

	void set_A_b(const T* const x) {

		A = Matrix<T> (	  x[0],   x[1], x[2],
						T(0.0),   x[3], x[4],
						T(0.0), T(0.0), x[5]);

		b = Vector<T> (x+6);
	}

	int size() const { return static_cast<int>(samples.size()); }

	const vector3& accel_at(int i) const { return samples.at(i).accel; }

	const vector3& magn_at(int i) const { return samples.at(i).magn; }

	const T error_at(int i) const {

		const Vector<T> x((this->*sample_at)(i));

		const Vector<T> y = A*(x-b); // TODO operator-(double, GradType) would be needed

		return y.length()-T(1.0); // Detto
	}

	typedef const vector3& (EllipsoidObjective::*MemFun)(int i) const;

	const std::vector<StaticSample> samples;

	MemFun sample_at;

	Matrix<T> A;

	Vector<T> b;
};

template <typename T>
T EllipsoidObjective<T>::f(const T* const v) {

	set_A_b(v);

	T sum(0.0);

	for (int i=0; i<size(); ++i) {

		const T err = error_at(i);

		sum += (err*err);
	}

	return sum;
}

// FIXME Separate interface and implementation
template<>
inline double EllipsoidObjective<double>::max_error(const double* const v) {

	set_A_b(v);

	double max_err(0.0);

	for (int i=0; i<size(); ++i) {

		const double err = error_at(i);

		if (std::fabs(err) > std::fabs(max_err)) {

			max_err = err;
		}
	}

	return max_err;
}

}

#endif // ELLIPSOIDOBJECTIVE_HPP_
