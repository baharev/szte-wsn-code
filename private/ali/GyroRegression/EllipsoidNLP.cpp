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

#include "EllipsoidNLP.hpp"

namespace gyro {

bool EllipsoidNLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
		Index m, Number* g_l, Number* g_u)
{
	assert(n==N_VARS);

	const double* xL = estimates->lower_bounds();
	const double* xU = estimates->upper_bounds();

	for (int i=0; i<N_VARS; ++i) {

		x_l[i] = xL[i];
		x_u[i] = xU[i];
	}

	// Set the bounds for the constraints
	for (Index i=0; i<m; i++) {
		g_l[i] = 0;
		g_u[i] = 0;
	}

	return true;
}

bool EllipsoidNLP::get_starting_point(Index n, bool init_x, Number* x,
		bool init_z, Number* z_L, Number* z_U,
		Index m, bool init_lambda,
		Number* lambda)
{
	assert(init_x == true);
	assert(init_z == false);
	assert(init_lambda == false);
	assert(n==N_VARS);

	const double* x0 = estimates->initial_point();

	for (int i=0; i<N_VARS; ++i) {

		x[i] = x0[i];
	}

	return true;
}

VarEstimates::VarEstimates() {

	for (int i=0; i<N_VARS; ++i) {

		x_0[i] = 0.0;
	}
}

VarEstimates::~VarEstimates() { }

AccelVarEstimates::AccelVarEstimates() {

	for (int i=0; i<B1; ++i) {
		x_L[i] =-1.0/3000.0;;
		x_U[i] = 1.0/3000.0;;
	}

	x_L[A11] = x_L[A22] = x_L[A33] = 1.0/1500.0;
	x_U[A11] = x_U[A22] = x_U[A33] = 1.0/ 700.0;

	x_L[B1] = 2000.0;
	x_U[B1] = 2500.0;

	x_L[B2] = 2250.0;
	x_U[B2] = 2750.0;

	x_L[B3] = 2000.0;
	x_U[B3] = 2500.0;

	x_0[A11] = 1.0/1180.0;
	x_0[A22] = 1.0/1189.0;
	x_0[A33] = 1.0/1106.0;

	x_0[B1] = 2250.0;
	x_0[B2] = 2500.0;
	x_0[B3] = 2250.0;
}

MagnetoVarEstimates::MagnetoVarEstimates() {

	for (int i=0; i<B1; ++i) {
		x_L[i] =-1.0/1000.0;;
		x_U[i] = 1.0/1000.0;;
	}

	x_L[A11] = x_L[A22] = x_L[A33] = 1.0/750.0;
	x_U[A11] = x_U[A22] = x_U[A33] = 1.0/300.0;

	x_L[B1] = -200.0;
	x_U[B1] =  200.0;

	x_L[B2] = -200.0;
	x_U[B2] =  200.0;

	x_L[B3] = -200.0;
	x_U[B3] =  200.0;

	x_0[A11] = 1.0/550.0;
	x_0[A22] = 1.0/550.0;
	x_0[A33] = 1.0/550.0;

	x_0[B1] = -100.0;
	x_0[B2] = -100.0;
	x_0[B3] = -100.0;
}

}
