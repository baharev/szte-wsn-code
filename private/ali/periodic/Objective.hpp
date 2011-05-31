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

#ifndef OBJECTIVE_HPP_
#define OBJECTIVE_HPP_

#include <vector>
#include "CompileTimeConstants.hpp"
#include "MatrixVector.hpp"
#include "Sample.hpp"
#include "VarEstimates.hpp"

namespace gyro {

template <typename T>
class Objective {

public:

	Objective(const std::vector<Sample>& samples) : samples(samples) { }

	T f(const T* const x)  {

		init_vars(x);

	}

private:

	void init_vars(const T* const x) {

		A = Matrix<T>(x+A11);
		b = Vector<T>(x+B1);

		C = Matrix<T>(x+C11);
		d = Vector<T>(x+D1);

		Euler_XYZ = Vector<T>(x+Euler_X);

		v0 = Vector<T>(x+VX);
	}

	void set_initial_orientation() {

		R = euler2rotmat(Euler_XYZ);
	}

	const vector3& raw_gyro(int i) const {

		return samples.at(i).gyro;
	}

	const double time_step(int i) const {

		unsigned int t2 = samples.at(i  ).timestamp;
		unsigned int t1 = samples.at(i-1).timestamp;

		return (t2-t1)/TICKS_PER_SEC;
	}

	const std::vector<Sample>& samples;

	Matrix<T> A;
	Vector<T> b;

	Matrix<T> C;
	Vector<T> d;

	Vector<T> Euler_XYZ;

	Vector<T> v0;

	Matrix<T> R;
	Vector<T> s;
	Matrix<T> M;

};

}

#endif // OBJECTIVE_HPP_
