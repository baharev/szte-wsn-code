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

#ifndef MODEL_HPP_
#define MODEL_HPP_

#include <iomanip>
#include <vector>
#include "MatrixVector.hpp"
#include "Sample.hpp"
#include "VarEstimates.hpp"

namespace gyro {

template <typename T>
class Model {

public:

	Model(const std::vector<Sample>& samples) :
	samples(samples), TICKS_PER_SEC(32768), N(static_cast<int>(samples.size())), estimates(VarEstimates())
	{
		fix_all();

		store_rotmat();
	}

	void set_used_variables(const T* x) {

		use_b(x);

		use_v(x);
	}

	T objective(const T* x)  {

		set_used_variables(x);

		return minimize_bumps();
		//return T(0.0);
	}

	const std::vector<T> constraints(const T* x) {

		set_used_variables(x);

		rotate_back();

		store_path();

		Vector<T> delta_r = position.at(N-1);

		return as_std_vector(delta_r);
	}

	void store_rotmat() {

		rotmat.clear();

		rotmat.resize(N);

		Matrix<T> R = Matrix<T>::identity();

		rotmat.at(0) = R;

		for (int i=1; i<N; ++i) {

			R = update(R, i);

			rotmat.at(i) = R;
		}
	}

	void rotate_sum_downwards(const T* x) {

		set_used_variables(x);

		rotate_back();
	}

	const Vector<T> downward_rotated_sum() const {

		Vector<T> s = M*R_accel(0);

		for (int i=1; i<N; ++i) {

			s += M*R_accel(i);
		}

		return s/N;
	}

	const Vector<T> delta_v(int i) const {

		const Vector<T> acc2 = M*R_accel(i  ) - gravity;

		const Vector<T> acc1 = M*R_accel(i-1) - gravity;

		return ((acc1 + acc2)/2)*time_step(i);
	}

	void set_v0(const double* x) {

		use_v(x);
	}

	const Vector<T> delta_r() const {

		Vector<T> r, v;

		v = v0;

		for (int i=1; i<N; ++i) {

			v += delta_v(i);

			r += v*time_step(i);
		}

		return r;
	}

	void store_path() {

		position.clear();

		position.resize(N);

		Vector<T> r, v;

		v = v0;

		position.at(0) = r;

		for (int i=1; i<N; ++i) {

			v += delta_v(i);

			r += v*time_step(i);

			position.at(i) = r;
		}
	}

	void dump_recomputed_path(const T* x, std::ostream& out) {

		set_used_variables(x);

		rotate_back();

		Vector<T> r, v;

		v = v0;

		out << std::setprecision(16) << std::scientific;

		out << 0 << '\t' << v << '\t' << r << '\n';

		for (int i=1; i<N; ++i) {

			v += delta_v(i);

			r += v*time_step(i);

			out << i << '\t' << v << '\t' << r << '\n';
		}

		out << std::flush;
	}

	const T minimize_bumps() {

		rotate_back();

		store_path();

		return integrate_bumps();
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

		T sum(0.0);

		for (int i=0; i<N-MOVING_AVG_WINDOW_SIZE; ++i) {

			Vector<T> average = average_in_window(i, MOVING_AVG_WINDOW_SIZE);

			sum += sqr(average);
		}

		return sum;
	}

	void dump_moving_averages(std::ostream& out) const {

		out << std::setprecision(16) << std::scientific;

		const int MOVING_AVG_WINDOW_SIZE = 193;

		T sum(0.0);

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

		store_rotmat();

		rotate_back();

		return sqr(delta_r());
	}

	const T minimize_rotation() {

		Matrix<T> R = Matrix<T>::identity();

		for (int i=1; i<N; ++i) {

			R = update(R, i);
		}

		Vector<T> error(R[X][X]-1.0, R[Y][Y]-1.0, R[Z][Z]-1.0);

		return sqr(error);
	}

	const T gyro_regression() {

		store_rotmat();

		return gyro_regression_objective();
	}

	const T gyro_regression_objective() const {

		const Vector<T> s = sum_R_accel();

		return -((s[X]/N)*(s[X]/N) + (s[Y]/N)*(s[Y]/N) + (s[Z]/N)*(s[Z]/N));
	}

	const Vector<T> sum_R_accel() const {

		Vector<T> s = R_accel(0);

		for (int i=1; i<N; ++i) {

			s += R_accel(i);
		}

		return s;
	}

	void rotate_back() {

		const Vector<T> s = sum_R_accel();

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

	const vector3& raw_gyro(int i) const {

		return samples.at(i).gyro;
	}

	const Vector<T> angular_rate(int i) const {

		Vector<T> offset(d);

		return offset += C*raw_gyro(i);
	}

	const vector3& raw_accel(int i) const {

		return samples.at(i).accel;
	}

	const Vector<T> calibrated_accel(int i) const {

		Vector<T> offset(b);

		return offset += A*raw_accel(i);
	}

	const Vector<T> R_accel(int i) const {

		return rotmat.at(i)*calibrated_accel(i);
	}

	const double time_step(int i) const {

		unsigned int t2 = samples.at(i  ).timestamp;
		unsigned int t1 = samples.at(i-1).timestamp;

		return (t2-t1)/TICKS_PER_SEC;
	}

	const std::vector<T> as_std_vector(const Vector<T>& r) const {

		std::vector<T> result(3);

		result.at(X) = r[X];
		result.at(Y) = r[Y];
		result.at(Z) = r[Z];

		return result;
	}

	const Matrix<T> update(const Matrix<T>& R, int i) {

		Vector<T> angle = angular_rate(i)*time_step(i);

		T tmp[] = {    T(1.0), -angle[Z],  angle[Y],
				     angle[Z],    T(1.0), -angle[X],
				    -angle[Y],  angle[X],    T(1.0)
		};

		const Matrix<T> G(tmp);

		return normalize(R*G);
	}

	const Matrix<T> normalize(const Matrix<T>& R) const {

		const Vector<T> Rx = R[X];
		const Vector<T> Ry = R[Y];

		T half_error = (Rx*Ry)/2;

		const Vector<T> Rx_new = Rx-half_error*Ry;
		const Vector<T> Ry_new = Ry-half_error*Rx;
		const Vector<T> Rz_new = cross_product(Rx_new, Ry_new);

		T Cx = correction(Rx_new);
		T Cy = correction(Ry_new);
		T Cz = correction(Rz_new);

		return Matrix<T> (Cx*Rx_new, Cy*Ry_new, Cz*Rz_new);
	}

	const T correction(const Vector<T>& v) const {

		return (3.0-sqr(v))/2.0;
	}

	void fix_A() {

		A = estimates.accel_gain();
	}

	void fix_b() {

		b = Vector<T>(estimates.initial_point(B1));
	}

	void fix_C() {

		C = estimates.gyro_gain();
	}

	void fix_d() {

		d = Vector<T>(estimates.initial_point(D1));
	}

	void fix_v0() {

		v0 = Vector<T>(estimates.initial_point(VX));
	}

	void fix_all() {

		fix_A();
		fix_b();
		fix_C();
		fix_d();
		fix_v0();
	}

	void use_b(const T* x) {

		b = Vector<T>(x+B1);
	}

	void use_d(const T* x) {

		d = Vector<T>(x+D1);
	}

	void use_v(const T* x) {

		v0 = Vector<T>(x+VX);
	}

	const std::vector<Sample>& samples;

	const double TICKS_PER_SEC;

	const int N;

	const VarEstimates estimates;

	matrix3 A;
	Vector<T> b;

	matrix3 C;
	Vector<T> d;

	Vector<T> v0;

	std::vector<Matrix<T> > rotmat;
	std::vector<Vector<T> > position;

	Matrix<T> M;
	Vector<T> gravity;
};

}

#endif // MODEL_HPP_
