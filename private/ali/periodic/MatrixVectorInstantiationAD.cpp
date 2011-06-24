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

#include <cmath>
#include <ostream>
#include "HessType.hpp"
#include "VarEnum.hpp"

// Only in tapeless forward mode, otherwise use adouble
#define NUMBER_DIRECTIONS gyro::N_VARS
#include "adouble.h"
typedef adtl::adouble adouble;
ADOLC_TAPELESS_UNIQUE_INTERNALS;

#include "MatrixVector.cpp"

namespace gyro {

template class Vector<adouble>;
template class Matrix<adouble>;

template const Vector<adouble> operator+(const Vector<adouble>& x, const Vector<adouble>& y);
template const Vector<adouble> operator-(const Vector<adouble>& x, const Vector<adouble>& y);
template const Vector<adouble> operator*(const Vector<adouble>& x, double y);
template const Vector<adouble> operator/(const Vector<adouble>& x, double y);
template const Matrix<adouble> operator+(const Matrix<adouble>& A, const Matrix<adouble>& B);

template std::ostream& operator<<(std::ostream& os, const Vector<adouble>& x);
template const adouble operator*(const Vector<adouble>& x, const Vector<adouble>& y);
template const Vector<adouble> operator*(const adouble& c, const Vector<adouble>& x);
template const Vector<adouble> cross_product(const Vector<adouble>& x, const Vector<adouble>& y);
template const Matrix<adouble> euler2rotmat(const Vector<adouble>& Euler_XYZ);

template const Vector<adouble> operator*(const Matrix<adouble>& M, const Vector<adouble>& v);
template const Vector<adouble> operator*(const Matrix<adouble>& M, const vector3& v);

template Matrix<double>::Matrix(const double array[9]);
template Matrix<adouble>::Matrix(const adouble array[9]);
template Matrix<adouble>::Matrix(const double array[9]);
template Matrix<adouble>::Matrix(const Matrix<double>& other);

template Vector<adouble>& Vector<adouble>::operator/=(const adouble& );
template Vector<adouble>& Vector<adouble>::operator/=(const double& );

template <>
const Matrix<HessType<N_VARS> > euler2rotmat(const Vector<HessType<N_VARS> >& );

template class Vector<HessType<N_VARS> >;
template class Matrix<HessType<N_VARS> >;

template const Vector<HessType<N_VARS> > operator+(const Vector<HessType<N_VARS> >& x, const Vector<HessType<N_VARS> >& y);
template const Vector<HessType<N_VARS> > operator-(const Vector<HessType<N_VARS> >& x, const Vector<HessType<N_VARS> >& y);
template const Vector<HessType<N_VARS> > operator*(const Vector<HessType<N_VARS> >& x, double y);
template const Vector<HessType<N_VARS> > operator/(const Vector<HessType<N_VARS> >& x, double y);
template const Matrix<HessType<N_VARS> > operator+(const Matrix<HessType<N_VARS> >& A, const Matrix<HessType<N_VARS> >& B);

template std::ostream& operator<<(std::ostream& os, const Vector<HessType<N_VARS> >& x);
template const HessType<N_VARS>  operator*(const Vector<HessType<N_VARS> >& x, const Vector<HessType<N_VARS> >& y);
template const Vector<HessType<N_VARS> > operator*(const HessType<N_VARS> & c, const Vector<HessType<N_VARS> >& x);
template const Vector<HessType<N_VARS> > cross_product(const Vector<HessType<N_VARS> >& x, const Vector<HessType<N_VARS> >& y);
template const Matrix<HessType<N_VARS> > euler2rotmat(const Vector<HessType<N_VARS> >& Euler_XYZ);

template const Vector<HessType<N_VARS> > operator*(const Matrix<HessType<N_VARS> >& M, const Vector<HessType<N_VARS> >& v);
template const Vector<HessType<N_VARS> > operator*(const Matrix<HessType<N_VARS> >& M, const vector3& v);

template Matrix<HessType<N_VARS> >::Matrix(const HessType<N_VARS>  array[9]);
template Matrix<HessType<N_VARS> >::Matrix(const double array[9]);
template Matrix<HessType<N_VARS> >::Matrix(const Matrix<double>& other);

template Vector<HessType<N_VARS> >& Vector<HessType<N_VARS> >::operator/=(const HessType<N_VARS> & );
template Vector<HessType<N_VARS> >& Vector<HessType<N_VARS> >::operator/=(const double& );

}
