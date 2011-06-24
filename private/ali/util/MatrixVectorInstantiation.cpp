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

#include <ostream>
#include "GradType.hpp"
#include "MatrixVector.cpp"

namespace gyro {

template <>
void Vector<double>::enforce_range_minus_pi_plus_pi() {

	const double PI(3.14159265358979323846);
	const double PI_TIMES_2(6.28318530717958647693);

	for (int i=0; i<3; ++i) {

		while (v[i] < -PI) {
			v[i] += PI_TIMES_2;
		}

		while (v[i] >  PI) {
			v[i] -= PI_TIMES_2;
		}
	}
}

template class Vector<double>;
template class Matrix<double>;

template const matrix3 euler2rotmat(const vector3& Euler_XYZ);

template const vector3 operator*(const matrix3& M, const vector3& v);

template class Vector<GradType<12> >;
template class Matrix<GradType<12> >;

template const Vector<GradType<12> > operator+(const Vector<GradType<12> >& x, const Vector<GradType<12> >& y);
template const Vector<GradType<12> > operator-(const Vector<GradType<12> >& x, const Vector<GradType<12> >& y);
template const Vector<GradType<12> > operator*(const Vector<GradType<12> >& x, double y);
template const Vector<GradType<12> > operator/(const Vector<GradType<12> >& x, double y);
template const Matrix<GradType<12> > operator+(const Matrix<GradType<12> >& A, const Matrix<GradType<12> >& B);

template std::ostream& operator<<(std::ostream& os, const Vector<GradType<12> >& x);

template const Vector<GradType<12> > operator*(const Matrix<GradType<12> >& M, const Vector<GradType<12> >& v);
template const Vector<GradType<12> > operator*(const Matrix<GradType<12> >& M, const vector3& v);

template class Vector<GradType<9> >;
template class Matrix<GradType<9> >;

template const Vector<GradType<9> > operator+(const Vector<GradType<9> >& x, const Vector<GradType<9> >& y);
template const Vector<GradType<9> > operator-(const Vector<GradType<9> >& x, const Vector<GradType<9> >& y);
template const Vector<GradType<9> > operator*(const Vector<GradType<9> >& x, double y);
template const Vector<GradType<9> > operator/(const Vector<GradType<9> >& x, double y);
template const Matrix<GradType<9> > operator+(const Matrix<GradType<9> >& A, const Matrix<GradType<9> >& B);

template std::ostream& operator<<(std::ostream& os, const Vector<GradType<9> >& x);

template const Vector<GradType<9> > operator*(const Matrix<GradType<9> >& M, const Vector<GradType<9> >& v);
template const Vector<GradType<9> > operator*(const Matrix<GradType<9> >& M, const vector3& v);

}
