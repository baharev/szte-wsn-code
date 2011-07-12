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

#include "Vector.hpp"

namespace gyro {

typedef Matrix<double> matrix3;

template <typename T> const T distance(const Matrix<T>& A, const Matrix<T>& B);

template <typename T>
class Matrix {

public:

	Matrix(); // Elements are initialized to zero

	template <typename U>
	explicit Matrix(const U array[9]);

	template <typename U>
	explicit Matrix(const Matrix<U>& other);

	Matrix(const T& x1, const T& x2, const T& x3,
		   const T& x4, const T& x5, const T& x6,
		   const T& x7, const T& x8, const T& x9);

	Matrix(const Vector<T>& row_x, const Vector<T>& row_y, const Vector<T>& row_z);

	static const Matrix identity();

	void copy_to(T array[9]) const;

	const Vector<T> operator[] (coordinate i) const { return Vector<T> (m[i][X], m[i][Y], m[i][Z]); }

	Matrix& operator+=(const Matrix& M);

	const Matrix operator*(const Matrix& M) const;

	template <typename U>
	const Vector<T> operator*(const Vector<U>& v) const;

	friend const T distance<>(const Matrix& A, const Matrix& B);

	const T trace() const { return m[0][0]+m[1][1]+m[2][2]; }

	const Matrix<T> transponse() const;

	template <typename>	friend class Matrix; // templated copy ctor needs this

	std::ostream& print(std::ostream& os) const;

private:

	T m[3][3];
};

template <typename T>
const Matrix<T> operator+(const Matrix<T>& A, const Matrix<T>& B) { Matrix<T> C(A); return C+=B; }

template <typename T> const Matrix<T> euler2rotmat(const Vector<T>& euler_XYZ);

template <typename T>
std::ostream& operator<<(std::ostream& os, const Matrix<T>& x) { return x.print(os); }

}

#endif
