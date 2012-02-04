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
#include <stdexcept>
#include <vector>
#include "MatrixVector.hpp"
#include "ModelType.hpp"
#include "Sample.hpp"
#include "Variables.hpp"

namespace gyro {

template <typename > class MinimizeBumps;
template <typename > class MinimizeRotation;
template <typename > class GyroRegression;
template <typename > class PWLOffset;

template <typename T>
class Model {

public:

	static Model<T>* newInstance(ModelType type, const std::vector<Sample>& samples) {

		Model<T>* model = 0;

		if (type==MINIMIZE_ROTATION) {

			model = new MinimizeRotation<T>(samples);
		}
		else if (type==MINIMIZE_BUMPS) {

			model = new MinimizeBumps<T>(samples);
		}
		else if (type==GYRO_REGRESSION) {

			model = new GyroRegression<T>(samples);
		}
		else if (type==PWL_GYRO_OFFSET) {

			model = new PWLOffset<T>(samples);
		}

		model->x = 0;

		model->init();

		return model;
	}

	virtual void init() = 0;

	virtual T objective(const T* x) = 0;

	virtual int number_of_constraints() const = 0;

	virtual const std::vector<T> constraints(const T* x) = 0;

	virtual ~Model() { }

	void store_rotmat() {

		rotmat.clear();

		rotmat.resize(N);

		Matrix<T> R = Matrix<T>::identity();

		rotmat.at(0) = R;

		set_period_begin_end(0);

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

	const std::vector<T> delta_r_std_vector() {

		rotate_back();

		store_path();

		Vector<T> delta_r = position.at(N-1);

		return as_std_vector(delta_r);
	}

	const std::vector<T> delta_r_periods() {

		rotate_back();

		store_path();

		std::vector<T> constraints;

		for (int i=0; i<N_PERIODS; ++i) {

			int k = variables.end_period();

			const std::vector<T> delta_r = as_std_vector(position.at(k));

			constraints.insert(constraints.end(), delta_r.begin(), delta_r.end());

			variables.increment_period_counter();
		}

		return constraints;
	}

	const std::vector<T> delta_rotation_periods() {

		std::vector<T> constraints;

		variables.reset_period_position();

		for (int i=0; i<N_PERIODS; ++i) {

			int k = variables.end_period();

			const Matrix<T>& R = rotmat.at(k);

			constraints.push_back(R[X][X]-1.0);
			constraints.push_back(R[Y][Y]-1.0);
			constraints.push_back(R[Z][Z]-1.0);

			variables.increment_period_counter();
		}

		return constraints;
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

		out << 0 << '\t' << rotation_XYZ(0) << '\t' << v << '\t' << r << "\t###\n";

		for (int i=1; i<N; ++i) {

			Vector<T> theta = rotation_XYZ(i);

			v += delta_v(i);

			r += v*time_step(i);

			out << i << '\t' << theta << '\t' << v << '\t' << r << '\n';
		}

		out << std::flush;
	}

	std::vector<T> d_beg_equals_d_end() const {

		return as_std_vector(d_beg - d_end);
	}

	const T rotation_angle_deg(const int i) const {

		const T trace = rotmat.at(i).trace();

		const T cos_theta = (trace-1)/2;

		return acos_deg(cos_theta);
	}

	const Vector<T> rotation_XYZ(const int i) const {

		using std::atan2;

		Matrix<T> R = M*rotmat.at(i);

		R = R.transponse();

		const T x = atan2(R[Z][X], R[Y][X]);
		const T y = atan2(R[Z][X], R[X][X]);
		const T z = atan2(R[Y][Y], R[X][Y]);

		Vector<T> res(x, y, z);

		const T RAD2DEG(57.2957795130823);

		return res*=RAD2DEG;
	}

	const T acos_deg(const T& x) const {

		T cos_theta(x);

		if (cos_theta > 1.0) {
			cos_theta = 1.0;
		}
		else if (cos_theta < -1.0) {
			cos_theta = -1.0;
		}

		const T RAD2DEG(57.2957795130823);

		return std::acos(cos_theta)*RAD2DEG;
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

		const Vector<T> record_avg = average_in_window(0, N);

		const int WINDOW_SIZE = variables.lpf_win_size();

		T sum(0.0);

		for (int i=0; i<N-WINDOW_SIZE; ++i) {

			Vector<T> average = average_in_window(i, WINDOW_SIZE);

			sum += sqr(average-record_avg);
		}

		return sum;
	}

	void dump_moving_averages(std::ostream& out) const {

		out << std::setprecision(16) << std::scientific;

		const Vector<T> record_avg = average_in_window(0, N);

		out << record_avg << '\n';

		const int WINDOW_SIZE = variables.lpf_win_size();

		T sum(0.0);

		for (int i=0; i<N-WINDOW_SIZE; ++i) {

			Vector<T> average = average_in_window(i, WINDOW_SIZE);

			sum += sqr(average-record_avg);

			out << i << '\t' << average << '\n';
		}

		out << "Integral: " << sum << '\n';
		out << "gravity: " << gravity << '\n';

		out << std::flush;
	}

	const T integrate_rotation() {

		const int WINDOW_SIZE = variables.lpf_win_size();

		T integral(0.0);

		for (int i=0; i<N-WINDOW_SIZE; ++i) {

			integral += distance(rotmat.at(i), rotmat.at(i+WINDOW_SIZE));
		}

		return integral;
	}

protected:

	Model(const std::vector<Sample>& samples) :
	samples(samples), TICKS_PER_SEC(32768), N(static_cast<int>(samples.size())), variables(Variables())
	{
		fix_all();
	}

	virtual void set_used_variables(const T* x) = 0;

	void use_d(const T* x) {

		//d = Vector<T>(x+D1);
		throw std::logic_error("implementation not updated properly");
	}

	void use_d_varying(const T* x) {

//		d_beg = Vector<T>(x+DX_BEG); // FIXME Ask VarEstimates

//		d_end = Vector<T>(x+DX_END); // FIXME Ask VarEstimates
	}

	void use_v(const T* x) {

		v0 = variables.initial_velocity(x);
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

		gravity = M*(s/N); //-9.747827
		//gravity = Vector<T>( T(0.0), T(0.0), T(-9.81));
	}

	void reset_period_position() {

		variables.reset_period_position();
	}

	void store_variables(const T* x) {

		this->x = x;
	}

private:

	const T minimize_delta_r() {

		store_rotmat();

		rotate_back();

		return sqr(delta_r());
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

	const vector3& raw_gyro(int i) const {

		return samples.at(i).gyro;
	}

//	const Vector<T> angular_rate(int i) const {
//
//		Vector<T> offset(d);
//
//		return offset += C*raw_gyro(i);
//	}
//
//
//	const Vector<T> angular_rate(int i) const {
//
//		double weight = i;
//
//		weight /= (N-1);
//
//		Vector<T> offset(d_beg*(1-weight)+d_end*weight);
//
//		return offset += C*raw_gyro(i);
//	}

	const Vector<T> angular_rate(int i) {

		Vector<T> offset = pwl_gyro_offset(i);

		return offset += C*raw_gyro(i);
	}

	const Vector<T> pwl_gyro_offset(int i) {

		set_period_begin_end(i);

		double weight = i - per_beg;

		weight /= (per_end-per_beg);

		return d_beg*(1-weight)+d_end*weight;
	}

	void set_period_begin_end(int i) {

		if (variables.is_period_end(i) && variables.not_last_period(i)) {

			ASSERT2(i==variables.beg_period(),"i, beg: "<<i<<", "<<variables.beg_period());

			per_beg = i;

			per_end = variables.end_period();

			d_beg = variables.beg_gyro_offset(x);;

			d_end = variables.end_gyro_offset(x);

			variables.increment_period_counter();
		}
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

	double time_step(int i) const {

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

		A = variables.accel_gain();
	}

	void fix_b() {

		b = variables.accel_offset();
	}

	void fix_C() {

		C = variables.gyro_gain();
	}

//	void fix_d() {
//
//		d = Vector<T>(estimates.initial_point(D1));
//	}

	void fix_v0() {

//		v0 = Vector<T>(estimates.initial_point(VX)); // FIXME Ask VarEstimates
	}

	void fix_all() {

		fix_A();
		fix_b();
		fix_C();
//		fix_d();
		fix_v0();
	}

	const std::vector<Sample>& samples;

	const double TICKS_PER_SEC;

	const int N;

	Variables variables;

	matrix3 A;
	vector3 b;

	matrix3 C;

	//Vector<T> d;
	Vector<T> d_beg;
	Vector<T> d_end;
	int per_beg;
	int per_end;
	const T* x;

	Vector<T> v0;

	std::vector<Matrix<T> > rotmat;
	std::vector<Vector<T> > position;

	Matrix<T> M;
	Vector<T> gravity;
};


template <typename T>
class MinimizeRotation : public Model<T> {

public:

	MinimizeRotation(const std::vector<Sample>& samples) : Model<T>(samples) { }

	virtual ~MinimizeRotation() { }

private:

	virtual void init() { }

	virtual void set_used_variables(const T* x) {

		use_d(x);
	}

	T objective(const T* x) {

		set_used_variables(x);

		this->store_rotmat();

		return this->integrate_rotation();
	}

	int number_of_constraints() const {

		return 0;
	}

	const std::vector<T> constraints(const T* x) { return std::vector<T> (); }
};

template <typename T>
class GyroRegression : public Model<T> {

public:

	GyroRegression(const std::vector<Sample>& samples) : Model<T>(samples) { }

	virtual ~GyroRegression() { }

private:

	virtual void init() { }

	virtual void set_used_variables(const T* x) {

		use_d(x);
	}

	T objective(const T* x) {

		set_used_variables(x);

		return this->gyro_regression();
	}

	int number_of_constraints() const {

		return 0;
	}

	const std::vector<T> constraints(const T* x) { return std::vector<T> (); }
};

template <typename T>
class MinimizeBumps : public Model<T> {

public:

	MinimizeBumps(const std::vector<Sample>& samples) : Model<T>(samples) { }

	virtual ~MinimizeBumps() { }

private:

	virtual void init() { }

	virtual void set_used_variables(const T* x) {

		use_d_varying(x);

		use_v(x);

		this->store_rotmat();

		this->rotate_back();
	}

	T objective(const T* x) {

		set_used_variables(x);

		return this->minimize_bumps();
	}

	int number_of_constraints() const {

		return 3;
	}

	const std::vector<T> constraints(const T* x) {

		set_used_variables(x);

		return this->delta_r_std_vector();
	}
};

template <typename T>
class PWLOffset : public Model<T> {

public:

	PWLOffset(const std::vector<Sample>& samples) : Model<T>(samples) { }

	virtual ~PWLOffset() { }

private:

	virtual void init() { }

	virtual void set_used_variables(const T* x) {

		Model<T>::reset_period_position();

		Model<T>::store_variables(x);

		Model<T>::use_d_varying(x);

		Model<T>::use_v(x);

		Model<T>::store_rotmat();

		Model<T>::reset_period_position();
	}

	T objective(const T* x) {

		return T(0.0);

		set_used_variables(x);

		return Model<T>::minimize_bumps();
	}

	int number_of_constraints() const {

		return N_CONS;
	}

	const std::vector<T> constraints(const T* x) {

		set_used_variables(x);

		std::vector<T> cons     = Model<T>::delta_r_periods();

		std::vector<T> cons_rot = Model<T>::delta_rotation_periods();

		cons.insert( cons.end(), cons_rot.begin(), cons_rot.end() );

		std::vector<T> con_offset = Model<T>::d_beg_equals_d_end();

		cons.insert( cons.end(), con_offset.begin(), con_offset.end() );

		ASSERT(number_of_constraints()==static_cast<int>(cons.size()));

		return cons;
	}
};

}

#endif // MODEL_HPP_
