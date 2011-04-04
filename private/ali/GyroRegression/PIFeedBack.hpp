/** Copyright (c) 2011, University of Szeged
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
* Author: Ali Baharev
*/

#ifndef PIFEEDBACK_HPP_
#define PIFEEDBACK_HPP_

#include <cmath>
#include <ostream>
#include <assert.h>
#include "InputData.hpp"
#include "CompileTimeConstants.hpp"
#include "MatrixVector.hpp"

typedef double NT;

namespace gyro {

template<typename T>
class PIFeedBack {

private:

	const NT* const time_stamp;

	const NT* const acc_x;
	const NT* const acc_y;
	const NT* const acc_z;

	const NT* const wx;
	const NT* const wy;
	const NT* const wz;

	const int N;

	const NT g_ref;

	std::ostream& out;

	const NT K_P;
	const NT K_I;
	Vector<NT> w_I_corr;

	T* MR;

	Matrix<T> R;
	Matrix<T> C;
	Vector<T> d;
	Vector<T> s;
	Matrix<T> M;

	void set_sum_R0_C_d(const T* const x) {

		w_I_corr = Vector<NT>(0.0, 0.0, 0.0);

		s = Vector<T>(acc_x[0], acc_y[0], acc_z[0]);

		if (MR) {
			s = M*s;
		}

		R = M;
		//R = Matrix<T>::identity();
		C = Matrix<T>::identity() + Matrix<T> (x);
		d = Vector<T>(x+9);
	}

	const Vector<NT> angular_rate(int i) const {

		assert(0<=i && i<N);

		return Vector<NT> (wx[i], wy[i], wz[i]);
	}

	const NT time_step(int i) const {

		assert(0<=i && (i+1)<N);

		return (time_stamp[i+1]-time_stamp[i])/TICKS_PER_SEC;
	}

	const Vector<NT> acceleration(const int i) const {

		assert(0<=i && i<N);

		return Vector<NT>(acc_x[i], acc_y[i], acc_z[i]);
	}

	const Vector<NT> total_correction(int i) const {

		Vector<NT> a_ref(acceleration(i));

		const NT len = a_ref.length();

		if (len != 0.0) {

			a_ref /= len;
		}

		// TODO Problems: R is of type T; M is unknown.
		return cross_product(R[Z], a_ref);
	}

	const Vector<NT> corrected_angular_rate(int i) {

		const Vector<NT> total_corr = total_correction(i);

		const Vector<NT> w_P_corr = K_P*total_corr;

		// TODO Bound w_I_corr
		w_I_corr = w_I_corr + K_I*time_step(i)*total_corr;

		const Vector<NT> w = angular_rate(i);

		// 0.03 even in the ideal case; max 1.0; w min 0.01
		out << total_corr << '\t' << total_corr.length() << '\t';
		out << w << '\t' << w.length() << '\n';

		return (C*w+d) + w_P_corr + w_I_corr;
	}

	// FIXME Change w_avg in the solver's code to w(i),  i = 0...N-1
	void update_R(const int i) {

		Vector<T> angle = corrected_angular_rate(i)*time_step(i);

		T tmp[] = {    T(1.0), -angle[Z],  angle[Y],
				     angle[Z],    T(1.0), -angle[X],
				    -angle[Y],  angle[X],    T(1.0)
		};

		const Matrix<T> G(tmp);

		R = R*G;
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

	void sum_Ri_ai(const int i) {

		const Vector<NT> a_measured(acc_x[i], acc_y[i], acc_z[i]);

		Vector<T> a = R*a_measured;

		if (MR) {

			a = M*a;

			const Matrix<T> RotMat = M*R;
			const int k = 9*i;
			RotMat.copy_to(MR+k);
		}

		s += a;
	}

	T objective() {
		return -((s[X]/N)*(s[X]/N) + (s[Y]/N)*(s[Y]/N) + (s[Z]/N)*(s[Z]/N));
	}

public:

	PIFeedBack(const Input& data, std::ostream& os, bool ) :

		time_stamp(data.time_stamp()),

		acc_x(data.acc_x()),
		acc_y(data.acc_y()),
		acc_z(data.acc_z()),

		wx(data.wx()),
		wy(data.wy()),
		wz(data.wz()),

		N(data.N()),

		g_ref(data.g_ref()),

		out(os),

		K_P(NT(0.0)), // TODO Pass control parameters
		K_I(NT(0.0)),
		w_I_corr(Vector<NT>(0.0, 0.0, 0.0)),
		MR(0),
		d(Vector<T>(0,0,0)),
		s(Vector<T>(0,0,0))
	{

	}

	T f(const T* const x)  {

		set_sum_R0_C_d(x);

		for (int i=1; i<N; ++i) {

			update_R(i-1); // R(i)=R(i-1)*G(i-1)

			normalize_R();

			sum_Ri_ai(i);

		}

		return objective();
	}

	const T s_x() const { return s[X]/N; }

	const T s_y() const { return s[Y]/N; }

	const T s_z() const { return s[Z]/N; }

	void set_M_Matrix(const double* R0) {

		M = Matrix<NT> (R0);
	}

	void set_M(double* R, double* g_error) {

		M = Matrix<NT> (R);
		MR = R;
	}

};

}

#endif
