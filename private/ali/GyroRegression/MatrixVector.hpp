/** Copyright (c) 2010, 2011 University of Szeged
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

#ifndef MATRIXVECTOR_HPP_
#define MATRIXVECTOR_HPP_

#include <iosfwd>

namespace gyro {

enum coordinate { X, Y, Z };

template <typename> class Matrix;

template <typename T>
class Vector {

public:

	Vector(const T& x, const T& y, const T& z) { v[X] = x; v[Y] = y; v[Z] = z; }

	explicit Vector(const T x[3]) { v[X] = x[X]; v[Y] = x[Y]; v[Z] = x[Z]; }

	void copy_to(T array[3]) const;

	const T length() const;

	Vector& operator+=(const Vector& x);

	Vector& operator-=(const Vector& x);

	Vector& operator*=(double x);

	Vector& operator/=(double x);

	const T& operator[] (coordinate i) const { return v[i]; }

	void enforce_range_minus_pi_plus_pi();

	std::ostream& print(std::ostream& os) const  { return os<<v[X]<<'\t'<<v[Y]<<'\t'<<v[Z]; }

	template <typename C>
	friend const C operator*(const Vector<C>& x, const Vector<C>& y);

	template <typename C>
	friend const Vector<C> operator*(const C& c, const Vector<C>& x);

	template <typename C>
	friend const Vector<C> cross_product(const Vector<C>& x, const Vector<C>& y);

	template <typename>	friend class Matrix;

	template <typename U, typename V>
	friend const Vector<U> operator*(const Matrix<U>& M, const Vector<V>& v);

private:

	T v[3];
};

template <typename T>
class Matrix {

public:

	Matrix();

	explicit Matrix(const T array[9]);

	Matrix(const Vector<T>& row_x, const Vector<T>& row_y, const Vector<T>& row_z);

	void copy_to(T array[9]) const;

	const Matrix operator*(const Matrix& M) const;

	const Vector<T> operator[] (coordinate i) const;

	std::ostream& print(std::ostream& os) const;

	static const Matrix identity();

	Matrix& operator+=(const Matrix& M);

	template <typename U, typename V>
	friend const Vector<U> operator*(const Matrix<U>& M, const Vector<V>& v);

private:

	T m[3][3];
};

typedef Vector<double> vector3;
typedef Matrix<double> matrix3;

}

#endif
