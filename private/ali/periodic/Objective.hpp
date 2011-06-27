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

#include <iomanip>
#include <vector>
#include "CompileTimeConstants.hpp"
#include "MatrixVector.hpp"
#include "Sample.hpp"
#include "VarEstimates.hpp"

namespace gyro {

// TODO rename objective to problem representation
template <typename T>
class Model {

public:

	Model(const std::vector<Sample>& samples) :
	samples(samples), N(static_cast<int>(samples.size())), estimates(VarEstimates())
	{
		// FIXME Only for bump minimization
		fix_A();
		fix_C();
		fix_d();

		store_rotmat(); // FIXME Not appropriate for gyro regression plus C, d fixed!
	}

	T objective(const T* const x)  {

		//return minimize_bumps(x);
		return T(0.0);
	}

	const std::vector<T> constraints(const T* const x) {

		//b  = Vector<T>(x+B1);
		fix_b();

		v0 = Vector<T>(x+VX);

		rotate_back();

		store_path();

		std::vector<T> result(3);

		Vector<T> delta_r = position.at(N-1);

		result.at(X) = delta_r[X];
		result.at(Y) = delta_r[Y];
		result.at(Z) = delta_r[Z];

		return result;
	}

	void rotate_sum_downwards(const T* const x) {

		init_vars(x);

		fix_all_but_gyro_offset();

		rotate_back();
	}

	const Vector<T> get_rotated_sum() {

		s = M*rotated_accel(0);

		for (int i=1; i<N; ++i) {

			s += M*rotated_accel(i);
		}

		return s/N;
	}

	const Vector<T> get_delta_v() {

		Vector<T> dv; // = M*rotated_accel(0) - gravity;

		for (int i=1; i<N; ++i) {

			dv += velocity(i);
		}

		return dv;
	}

	const Vector<T> velocity(int i) const {

		const Vector<T> acc2 = M*rotated_accel(i  ) - gravity;

		const Vector<T> acc1 = M*rotated_accel(i-1) - gravity;

		return ((acc1 + acc2)/2)*time_step(i);
	}

	const Vector<T> get_delta_r() {

		Vector<T> r, v;

		v = v0;

		for (int i=1; i<N; ++i) {

			v += velocity(i);

			r += v*time_step(i);
		}

		return r;
	}

	void set_v0(const double* v) {

		v0 = Vector<T>(v);
	}

	// TODO dump stored positions not computed ones
	void dump_path(const T* const x, std::ostream& out) {

		b  = Vector<T>(x+B1);

		v0 = Vector<T>(x+VX);

		rotate_back();

		store_path();

		Vector<T> r, v;

		v = v0;

		out << std::setprecision(16) << std::scientific;

		out << 0 << '\t' << v << '\t' << r << '\n';

		for (int i=1; i<N; ++i) {

			v += velocity(i);

			r += v*time_step(i);

			out << i << '\t' << v << '\t' << r << '\n';
		}

		out << std::flush;
	}

	const T minimize_bumps(const T* const x) {

		// FIXME Check ctor, fixed others there

		b  = Vector<T>(x+B1);

		v0 = Vector<T>(x+VX);

		rotate_back();

		store_path();

		return integrate_bumps();
	}

	void store_rotmat() {

		rotmat.clear();

		rotmat.resize(N);

		R = Matrix<T>::identity();

		rotmat.at(0) = R;

		for (int i=1; i<N; ++i) {

			update_R(i);

			rotmat.at(i) = R;
		}
	}

	void store_path() {

		position.clear();

		position.resize(N);

		Vector<T> r, v;

		v = v0;

		position.at(0) = r;

		for (int i=1; i<N; ++i) {

			v += velocity(i);

			r += v*time_step(i);

			position.at(i) = r;
		}
	}

	const Vector<T> average_in_window(const int from, const int win_size) const {

		Vector<T> average;

		for (int i=from; i<from+win_size; ++i) {

			average += position.at(i);
		}

		average /= static_cast<double>(win_size);

		return average;
	}

	const T integrate_bumps() const {

		const int MOVING_AVG_WINDOW_SIZE = 193;

		T sum;

		for (int i=0; i<N-MOVING_AVG_WINDOW_SIZE; ++i) {

			Vector<T> average = average_in_window(i, MOVING_AVG_WINDOW_SIZE);

			sum += sqr(average);
		}

		return sum;
	}

	void dump_moving_averages(std::ostream& out) {

		out << std::setprecision(16) << std::scientific;

		const int MOVING_AVG_WINDOW_SIZE = 193;

		T sum;

		for (int i=0; i<N-MOVING_AVG_WINDOW_SIZE; ++i) {

			Vector<T> average = average_in_window(i, MOVING_AVG_WINDOW_SIZE);

			sum += sqr(average);

			out << i << '\t' << average << '\n';
		}

		out << "Integral: " << sum << '\n';
		out << "gravity: " << gravity << '\n';

		out << std::flush;
	}

private:

	const T minimize_delta_r() {

		fix_all_but_v();

		store_rotmat();

		rotate_back();

		Vector<T> r = get_delta_r();

		return r[X]*r[X] + r[Y]*r[Y] + r[Z]*r[Z];
	}

	const T minimize_rotation() {

		fix_all_but_gyro_offset();

		R = Matrix<T>::identity();

		for (int i=1; i<N; ++i) {

			update_R(i);
		}

		const T R11_err = R[X][X] - 1.0;
		const T R22_err = R[Y][Y] - 1.0;
		const T R33_err = R[Z][Z] - 1.0;

		return R11_err*R11_err + R22_err*R22_err + R33_err*R33_err;
	}

	const T gyro_regression(const T* const x) {

		fix_A();
		fix_b();
		//fix_C();
		fix_C_but_diagonal(x);
		//fix_d();
		fix_v0();

		store_rotmat();

		sum_rotated_accel();

		return gyro_regression_objective();
	}

	void sum_rotated_accel() {

		s = rotated_accel(0);

		for (int i=1; i<N; ++i) {

			s += rotated_accel(i);
		}
	}

	void rotate_back() {

		sum_rotated_accel();

		Vector<T> u = s;

		u.make_unit_length();

		// FIXME If-then-else is not good for optimization
		//Vector<T> v = (u[X]>1.0e-6 || u[Z]>1.0e-6) ? Vector<T>(-u[Z],0,u[X]) : Vector<T>(0,0,1);
		Vector<T> v = Vector<T>( -u[Z], T(0.0), u[X]);

		v.make_unit_length();

		Vector<T> w = cross_product(v, u);

		M = Matrix<T>(v, w, u*(-1));

		//gravity = M*(s/N); -9.747827
		gravity = Vector<T>( T(0.020), T(0.0), T(-9.748)); // TODO Check gravity
	}

	void init_vars(const T* const x) {

		b = Vector<T>(x+B1);

		d = Vector<T>(x+D1);

		v0 = Vector<T>(x+VX);
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

	const Vector<T> rotated_accel(int i) const {

		return (rotmat.at(i))*accel(i);
	}

	T gyro_regression_objective() const {
		return -((s[X]/N)*(s[X]/N) + (s[Y]/N)*(s[Y]/N) + (s[Z]/N)*(s[Z]/N));
	}

	void fix_A() {

		A = Matrix<T>(estimates.accel_gain());
	}

	void fix_b() {

		b = Vector<T>(estimates.initial_point(B1));
	}

	void fix_C() {

		C = Matrix<T>(estimates.gyro_gain());
	}

	void fix_d() {

		d = Vector<T>(estimates.initial_point(D1));
	}

	void fix_v0() {

		v0 = Vector<T>(estimates.initial_point(VX));
	}

	void fix_all_but_gyro_offset() {

		fix_A();
		fix_b();
		fix_C();
		//fix_d();
		fix_v0();
	}

	void fix_all_but_v() {

		fix_A();
		fix_b();
		fix_C();
		fix_d();
		//fix_v0();
	}

	const std::vector<Sample>& samples;

	const int N;

	const VarEstimates estimates;

	Matrix<T> A;
	Vector<T> b;

	Matrix<T> C;
	Vector<T> d;

	Vector<T> v0;

	Matrix<T> R;
	Vector<T> s;

	std::vector<Matrix<T> > rotmat;
	std::vector<Vector<T> > position;

	Matrix<T> M;
	Vector<T> gravity;
};

}

#endif // OBJECTIVE_HPP_
