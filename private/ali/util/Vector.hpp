/* Copyright (c) 2010, 2011 University of Szeged
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

#ifndef VECTOR_HPP_
#define VECTOR_HPP_

#include <cmath>
#include <iosfwd>

namespace gyro {

enum coordinate { X, Y, Z };

template <typename> class Vector;
template <typename> class Matrix;

typedef Vector<double> vector3;

inline double sqrt(double x) { return std::sqrt(x); }
inline double sqr(double x) { return std::pow(x, 2); }
template <typename T> const T sqr(const T& x) { return pow(x, 2); } // ADOL-C adouble needs this

template <typename T>
class Vector {

public:

	Vector() { v[X] = v[Y] = v[Z] = T(0.0); }

	Vector(const T& x, const T& y, const T& z) { v[X] = x; v[Y] = y; v[Z] = z; }

	template <typename U>
	explicit Vector(const U x[3]) { v[X] = T(x[X]); v[Y] = T(x[Y]); v[Z] = T(x[Z]); }

	template <typename U>
	explicit Vector(const Vector<U> x) { v[X] = x[X]; v[Y] = x[Y]; v[Z] = x[Z]; }

	void copy_to(T array[3]) const { for (int i=0; i<3; ++i) array[i] = v[i]; }

	const T length() const { return sqrt(sqr(v[X])+sqr(v[Y])+sqr(v[Z])); }

	Vector& operator+=(const Vector& x) { for (int i=0; i<3; ++i) v[i] += x.v[i]; return *this; }

	Vector& operator-=(const Vector& x) { for (int i=0; i<3; ++i) v[i] -= x.v[i]; return *this; }

	Vector& operator*=(double c) { for (int i=0; i<3; ++i) v[i] *= c; return *this; }

	template <typename U>
	Vector& operator/=(const U& x) { for (int i=0; i<3; ++i) v[i] /= x; return *this; }

	const T& operator[] (coordinate i) const { return v[i]; }

	friend const T operator*(const Vector& x, const Vector& y) { return x.v[X]*y.v[X]+x.v[Y]*y.v[Y]+x.v[Z]*y.v[Z]; }

	friend const Vector operator*(const T& c, const Vector& y) { return Vector<T> (c*y.v[X], c*y.v[Y], c*y.v[Z]); }

	friend const T sqr(const Vector& x) { return sqr(x.v[X])+sqr(x.v[Y])+sqr(x.v[Z]); }

	friend const Vector cross_product(const Vector& x, const Vector& y) {
		const T* const a = x.v;
		const T* const b = y.v;
		return Vector<T>(a[Y]*b[Z]-a[Z]*b[Y], a[Z]*b[X]-a[X]*b[Z], a[X]*b[Y]-a[Y]*b[X]);
	}

	template <typename>	friend class Matrix;

	void enforce_range_minus_pi_plus_pi(); // only for T = double

	std::ostream& print(std::ostream& os) const  { return os << v[X] << '\t' << v[Y] << '\t' <<v[Z]; }

private:

	T v[3];
};

template <typename T>
const Vector<T> operator+(const Vector<T>& x, const Vector<T>& y) { Vector<T> z(x); return z += y; }

template <typename T>
const Vector<T> operator-(const Vector<T>& x, const Vector<T>& y) { Vector<T> z(x); return z -= y; }

template <typename T>
const Vector<T> operator*(const Vector<T>& x, double y) { Vector<T> z(x); return z*=y; }

template <typename T>
const Vector<T> operator/(const Vector<T>& x, double y) { Vector<T> z(x); return z/=y; }

template <typename T>
std::ostream& operator<<(std::ostream& os, const Vector<T>& x) { return x.print(os); }

}

#endif // VECTOR_HPP_
