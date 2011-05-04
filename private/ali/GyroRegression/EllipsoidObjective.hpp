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

#include <vector>
#include "MatrixVector.hpp"
#include "StaticSample.hpp"

namespace gyro {

template <typename T>
class EllipsoidObjective {

public:

	EllipsoidObjective(const std::vector<StaticSample>& samples) : samples(samples) { }

	T accel(const T* const x) {

		return accumulate(x, &EllipsoidObjective::accel_at);
	}

	T magn(const T* const x) {

		return accumulate(x, &EllipsoidObjective::magn_at);
	}

private:

	void set_A_b(const T* const x) {

		A = Matrix<T> (	  x[0],   x[1], x[2],
						T(0.0),   x[3], x[4],
						T(0.0), T(0.0), x[5]);

		b = Vector<T> (x+6);
	}

	template <typename MemFun> T accumulate(const T* const x, MemFun at);

	int size() const { return static_cast<int>(samples.size()); }

	const vector3& accel_at(int i) const { return samples.at(i).accel; }

	const vector3& magn_at(int i) const { return samples.at(i).magn; }

	const std::vector<StaticSample> samples;

	Matrix<T> A;

	Vector<T> b;
};

template <typename T>
template <typename MemFun>
T EllipsoidObjective<T>::accumulate(const T* const v, MemFun at) {

	set_A_b(v);

	T sum(0.0);

	for (int i=0; i<size(); ++i) {

		const Vector<T> x((this->*at)(i));

		Vector<T> y = A*(x-b); // TODO operator-(double, GradType) would be needed

		T err = y.length()-T(1.0); // Detto

		sum += (err*err);
	}

	return sum;
}

}

#endif // ELLIPSOIDOBJECTIVE_HPP_
