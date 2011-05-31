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

	Objective(const std::vector<Sample>& samples) : samples(samples), estimates(VarEstimates()) { }

	T f(const T* const x)  {

		init_vars(x);

		return gyro_regression(x);
	}

private:

	const T minimize_rotation() {

		fix_A();
		fix_b();
		fix_C();
		//fix_d();
		fix_Euler();
		fix_v0();

		//Vector<T> angle = angular_rate(i)*time_step(i);
	}

	const T gyro_regression(const T* const x) {

		fix_A();
		fix_b();
		//fix_C();
		fix_C_but_diagonal(x);
		//fix_d();
		fix_Euler();
		fix_v0();

		set_initial_orientation();

		s = rotated_accel(0);

		for (int i=1; i<N; ++i) {

			update_R(i);

			s += rotated_accel(i);
		}

		return objective();
	}

	void init_vars(const T* const x) {

		N = static_cast<int>( samples.size() );

		A = Matrix<T>(x+A11);
		b = Vector<T>(x+B1);

		C = Matrix<T>(x+C11);
		d = Vector<T>(x+D1);

		Euler_XYZ = Vector<T>(x+EULER_X);

		v0 = Vector<T>(x+VX);
	}

	void set_initial_orientation() {

		R = euler2rotmat(Euler_XYZ);
	}

	const vector3& raw_gyro(int i) const {

		return samples.at(i).gyro;
	}

	const Vector<T> angular_rate(int i) const {

		const vector3 raw_gyro_avg = (raw_gyro(i-1)+raw_gyro(i))/2;

		return C*raw_gyro_avg+d;
	}

	const vector3& raw_accel(int i) const {

		return samples.at(i).accel;
	}

	const Vector<T> accel(int i) const {

		return A*raw_accel(i)+b;
	}

	const double time_step(int i) const {

		unsigned int t2 = samples.at(i  ).timestamp;
		unsigned int t1 = samples.at(i-1).timestamp;

		return (t2-t1)/TICKS_PER_SEC;
	}

	void update_R(const int i) {

		Vector<T> angle = angular_rate(i)*time_step(i);

		T tmp[] = {    T(1.0), -angle[Z],  angle[Y],
				     angle[Z],    T(1.0), -angle[X],
				    -angle[Y],  angle[X],    T(1.0)
		};

		const Matrix<T> G(tmp);

		R = R*G;

		normalize_R();
	}

	const T correction(const Vector<T>& v) const {

		return (3-v*v)/2;
	}

	void normalize_R() {

		const Vector<T> Rx = R[X];
		const Vector<T> Ry = R[Y];

		T half_error = (Rx*Ry)/2;

		const Vector<T> Rx_new = Rx-half_error*Ry;
		const Vector<T> Ry_new = Ry-half_error*Rx;
		const Vector<T> Rz_new = cross_product(Rx_new, Ry_new);

		T Cx = correction(Rx_new); // TODO Write *= with arg T, not double
		T Cy = correction(Ry_new);
		T Cz = correction(Rz_new);

		R = Matrix<T> (Cx*Rx_new, Cy*Ry_new, Cz*Rz_new);
	}

	const Vector<T> rotated_accel(int i) {

		return R*accel(i);
	}

	T objective() {
		return -((s[X]/N)*(s[X]/N) + (s[Y]/N)*(s[Y]/N) + (s[Z]/N)*(s[Z]/N));
	}

	void fix_A() {

		A = Matrix<T>(estimates.initial_point(A11));
	}

	void fix_b() {

		b = Vector<T>(estimates.initial_point(B1));
	}

	void fix_C() {

		C = Matrix<T>(estimates.initial_point(C11));
	}

	void fix_C_but_diagonal(const T* const x) {

		const double* const x0 = estimates.initial_point();

		T tmp[9] = {
				 x[C11], x0[C12], x0[C13],
				x0[C21],  x[C22], x0[C23],
				x0[C31], x0[C32],  x[C33]
		};

		C = Matrix<T>(tmp);
	}

	void fix_d() {

		d = Vector<T>(estimates.initial_point(D1));
	}

	void fix_Euler() {

		Euler_XYZ = Vector<T>(estimates.initial_point(EULER_X));
	}

	void fix_v0() {

		v0 = Vector<T>(estimates.initial_point(VX));
	}

	const std::vector<Sample>& samples;

	const VarEstimates estimates;

	int N;

	Matrix<T> A;
	Vector<T> b;

	Matrix<T> C;
	Vector<T> d;

	Vector<T> Euler_XYZ;

	Vector<T> v0;

	Matrix<T> R;
	Vector<T> s;

};

}

#endif // OBJECTIVE_HPP_
