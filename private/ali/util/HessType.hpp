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

#ifndef HESSTYPE_HPP_
#define HESSTYPE_HPP_

#include <ostream>

namespace gyro {

template <int N>
class HessType {

public:

	HessType() { }

	HessType(double x) { init(x); }

	HessType& operator=(double rhs) { init(rhs); return *this; }

	HessType& operator+=(const HessType& x);

	HessType& operator+=(double x) { f += x; return *this; }

	HessType& operator-=(const HessType& x);

	HessType& operator-=(double x) { f -= x; return *this; }

	// just a workaround, it would be messy otherwise
	HessType& operator*=(const HessType& x) { *this = (*this)*x; return *this; }

	HessType& operator*=(double x);

	// just a workaround
	HessType& operator/=(const HessType& x) { *this = (*this)/x; return *this; };

	template <int N_VAR>
	friend void init_vars(HessType<N_VAR> var[N_VAR], const double* const x);

	template <int N_VAR>
	friend const HessType<N_VAR> operator-(const HessType<N_VAR>& x);

	template <int N_VAR>
	friend const HessType<N_VAR> operator*(const HessType<N_VAR>& x, const HessType<N_VAR>& y);

	template <int N_VAR>
	friend const HessType<N_VAR> operator/(const HessType<N_VAR>& x, const HessType<N_VAR>& y);

	template <int N_VAR>
	friend const HessType<N_VAR> operator/(const double x, const HessType<N_VAR>& y);

	template <int N_VAR>
	friend const HessType<N_VAR> sqrt(const HessType<N_VAR>& x);

	template <int N_VAR>
	friend const HessType<N_VAR> sqr(const HessType<N_VAR>& x);

	std::ostream& print(std::ostream& os) const { os << this->f << std::flush; return os; }

private:

	void init(double x);

	double f;
	double g[N];
	double h[N][N];
};

template <int N>
void HessType<N>::init(double x) {
	f = x;
	for (int i=0; i<N; ++i) {
		g[i] = 0.0;
		for (int j=0; j<=i; ++j) {
			h[i][j] = 0.0;
		}
	}
}

template <int N>
void init_vars(HessType<N> var[N], const double* x) {

	for (int i=0; i<N; ++i) {
		var[i].init(x[i]);
		var[i].g[i] = 1.0;
	}
}

template <int N>
HessType<N>& HessType<N>::operator+=(const HessType<N>& x) {

	f += x.f;
	for (int i=0; i<N; ++i) {
		g[i] += x.g[i];
		for (int j=0; j<=i; ++j) {
			h[i][j] += x.h[i][j];
		}
	}

	return *this;
}

template <int N>
const HessType<N> operator+(const HessType<N>& x, const HessType<N>& y) {

	HessType<N> z(x);

	return z+=y;
}

template <int N>
const HessType<N> operator+(double x, const HessType<N>& y) {

	HessType<N> z(y);

	return z+=x;
}

template <int N>
const HessType<N> operator+(const HessType<N>& x, double y) {

	HessType<N> z(x);

	return z+=y;
}

template <int N>
const HessType<N> operator-(const HessType<N>& x) {

	HessType<N> z;

	z.f = -x.f;
	for (int i=0; i<N; ++i) {
		z.g[i] = -x.g[i];
		for (int j=0; j<=i; ++j) {
			z.h[i][j] = -x.h[i][j];
		}
	}

	return z;
}

template <int N>
HessType<N>& HessType<N>::operator-=(const HessType<N>& x) {

	f -= x.f;
	for (int i=0; i<N; ++i) {
		g[i] -= x.g[i];
		for (int j=0; j<=i; ++j) {
			h[i][j] -= x.h[i][j];
		}
	}

	return *this;
}

template <int N>
const HessType<N> operator-(const HessType<N>& x, const HessType<N>& y) {

	HessType<N> z(x);

	return z-=y;
}

template <int N>
const HessType<N> operator-(double x, const HessType<N>& y) {

	return x+(-y);
}

template <int N>
const HessType<N> operator-(const HessType<N>& x, double y) {

	HessType<N> z(x);

	return z-=y;
}

template <int N>
const HessType<N> operator*(const HessType<N>& x, const HessType<N>& y) {

	HessType<N> z;

	z.f = x.f*y.f;
	for (int i=0; i<N; ++i) {
		z.g[i] = y.f*x.g[i] + x.f*y.g[i];
		for (int j=0; j<=i; ++j) {
			z.h[i][j] = y.f*x.h[i][j]+x.g[i]*y.g[j]+y.g[i]*x.g[j]+x.f*y.h[i][j];
		}
	}

	return z;
}

template <int N>
HessType<N>& HessType<N>::operator*=(double x) {

	f *= x;
	for (int i=0; i<N; ++i) {
		g[i] *= x;
		for (int j=0; j<=i; ++j) {
			h[i][j] *= x;
		}
	}

	return *this;
}

template <int N>
const HessType<N> operator*(double x, const HessType<N>& y) {

	HessType<N> z(y);

	return z*=x;
}

template <int N>
const HessType<N> operator*(const HessType<N>& x, double y) {

	HessType<N> z(x);

	return z*=y;
}

template <int N>
const HessType<N> operator/(const HessType<N>& x, const HessType<N>& y) {

	HessType<N> z;

	z.f = x.f/y.f;
	for (int i=0; i<N; ++i) {
		z.g[i] = (x.g[i] - z.f*y.g[i]) / y.f;
		for (int j=0; j<=i; ++j) {
			z.h[i][j] = (x.h[i][j]-z.g[i]*y.g[j]-y.g[i]*z.g[j]-z.f*y.h[i][j])/y.f;
		}
	}

	return z;
}

template <int N>
const HessType<N> operator/(const double x, const HessType<N>& y) {

	HessType<N> z;

	z.f = x/y.f;
	const double p = -z.f/y.f;
	const double q = (-2.0*p)/y.f;
	for (int i=0; i<N; ++i) {
		z.g[i] = p*y.g[i];
		for (int j=0; j<=i; ++j) {
			z.h[i][j] = p*y.h[i][j] + q*y.g[i]*y.g[j];
		}
	}

	return z;
}

template <int N>
const HessType<N> operator/(const HessType<N>& x, const double y) {

	return x*(1.0/y);
}

template <int N>
const HessType<N> sqrt(const HessType<N>& x) {

	HessType<N> z;

	const double tmp0 =  std::sqrt(x.f);
	const double tmp1 =  1/(2*tmp0);
	const double tmp2 = -tmp1/(2*x.f);

	z.f = tmp0;
	for (int i=0; i<N; ++i) {
		z.g[i] = tmp1*x.g[i];
		for (int j=0; j<=i; ++j) {
			z.h[i][j] = tmp1*x.h[i][j] + tmp2*x.g[i]*x.g[j];
		}
	}

	return z;
}

template <int N>
const HessType<N> sqr(const HessType<N>& x) {

	HessType<N> z;

	z.f = std::pow(x.f, 2);
	const double tmp = 2*x.f;
	for (int i=0; i<N; ++i) {
		z.g[i] = tmp*x.g[i];
		for (int j=0; j<=i; ++j) {
			z.h[i][j] = tmp*x.h[i][j] + 2.0*x.g[i]*x.g[j];
		}
	}

	return z;
}

template <int N>
std::ostream& operator<<(std::ostream& os, const HessType<N>& x) {
	return x.print(os);
}

}

#endif
