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

#include <algorithm>
#include "NLPForwardAD.hpp"
#include "Model.hpp"
#include "VarEnum.hpp"
#include "VarEstimates.hpp"
#define NUMBER_DIRECTIONS gyro::N_VARS
#include "adolc.h"

namespace gyro {

const int N_CONS(3);

NLPForwardAD::NLPForwardAD(const std::vector<Sample>& samples) :
		minimizer(new double[N_VARS]),
		modelDouble(new Model<double>(samples)),
		modelGradType(new Model<adouble>(samples)),
		estimates(new VarEstimates)
{

}

NLPForwardAD::~NLPForwardAD(){

	delete[] minimizer;
	delete modelDouble;
	delete modelGradType;
	delete estimates;
}

bool NLPForwardAD::get_bounds_info(Index n, Number* x_l, Number* x_u,
		Index m, Number* g_l, Number* g_u)
{
	assert(n==N_VARS);
	assert(m==N_CONS);

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

bool NLPForwardAD::get_starting_point(Index n, bool init_x, Number* x,
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

bool NLPForwardAD::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
		Index& nnz_h_lag, IndexStyleEnum& index_style)
{
	n = N_VARS;

	m = N_CONS;

	// in this example the jacobian is dense. Hence, it contains n*m nonzeros
	nnz_jac_g = n*m;

	// the hessian is also dense and has n*n total nonzeros, but we
	// only need the lower left corner (since it is symmetric)
	nnz_h_lag = n*(n-1)/2+n;

	// These were pretty much misplaced in generate_tapes
	Jac = new double*[m];
	for(Index i=0;i<m;i++)
		Jac[i] = new double[n];

	x_lam   = new double[n+m+1];

	Hess = new double*[n+m+1];
	for(Index i=0;i<n+m+1;i++)
		Hess[i] = new double[i+1];

	// use the C style indexing (0-based)
	index_style = C_STYLE;

	return true;
}

bool NLPForwardAD::eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {

	obj_value = modelDouble->objective(x);

	return true;
}

bool NLPForwardAD::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) {

	adouble vars[N_VARS];

	set_variable_vector(x, vars);

	const adouble objective_value = modelGradType->objective(vars);

	copy_derivatives(objective_value, grad_f);

	return true;
}

void NLPForwardAD::set_variable_vector(const double* x, adouble* vars) const {

	for (int i=0; i<N_VARS; ++i) {

		vars[i] = x[i];

		vars[i].setADValue(i, 1);
	}
}

void NLPForwardAD::copy_derivatives(const adouble& x, double* derivatives) const {

	for (int i=0; i<N_VARS; ++i) {

		derivatives[i] = x.getADValue(i);
	}
}

bool NLPForwardAD::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) {

	assert(m==N_CONS);

	if (N_CONS) {

		const std::vector<double> con = modelDouble->constraints(x);

		std::copy(con.begin(), con.end(), g);
	}

	return true;
}

bool NLPForwardAD::eval_jac_g(Index n, const Number* x, bool new_x,
		Index m, Index nele_jac, Index* iRow, Index *jCol,
		Number* values)
{
	assert(n==N_VARS && m==N_CONS);

	if (values == NULL) {

		fill_Jacobian_sparsity_as_dense(iRow, jCol);

		return true;
	}

	if (!N_CONS) {

		return true;
	}

	compute_Jacobian(x);

	Index idx = 0;

	for(Index i=0; i<m; i++) {

		for(Index j=0; j<n; j++) {

			values[idx++] = Jac[i][j];
		}
	}

	return true;
}

void NLPForwardAD::fill_Jacobian_sparsity_as_dense(Index* iRow, Index *jCol) const {

	Index idx = 0;

	for(Index i=0; i<N_CONS; i++) {

		for(Index j=0; j<N_VARS; j++) {

			iRow[idx] = i;

			jCol[idx++] = j;
		}
	}
}

void NLPForwardAD::compute_Jacobian(const double* x) {

	adouble vars[N_VARS];

	set_variable_vector(x, vars);

	const std::vector<adouble> con = modelGradType->constraints(vars);

	adouble constraint[N_CONS];

	std::copy(con.begin(), con.end(), constraint);

	for(int i=0; i<N_CONS; i++) {

		copy_derivatives(constraint[i], Jac[i]);
	}
}

bool NLPForwardAD::eval_h(Index n, const Number* x, bool new_x,
		Number obj_factor, Index m, const Number* lambda,
		bool new_lambda, Index nele_hess, Index* iRow,
		Index* jCol, Number* values)
{
	if (values == NULL) {
		// return the structure. This is a symmetric matrix, fill the lower left
		// triangle only.

		// the hessian for this problem is actually dense
		Index idx=0;
		for (Index row = 0; row < n; row++) {
			for (Index col = 0; col <= row; col++) {
				iRow[idx] = row;
				jCol[idx] = col;
				idx++;
			}
		}

		assert(idx == nele_hess);
	}
	else {
		// return the values. This is a symmetric matrix, fill the lower left
		// triangle only

		for(Index i = 0; i<n ; i++)
			x_lam[i] = x[i];
		for(Index i = 0; i<m ; i++)
			x_lam[n+i] = lambda[i];
		x_lam[n+m] = obj_factor;

		//hessian(tag_L,n+m+1,x_lam,Hess);

		Index idx = 0;

		for(Index i = 0; i<n ; i++)
		{
			for(Index j = 0; j<=i ; j++)
			{
				values[idx++] = Hess[i][j];
			}
		}
	}

	return true;
}

void NLPForwardAD::finalize_solution(SolverReturn status,
		Index n, const Number* x, const Number* z_L, const Number* z_U,
		Index m, const Number* g, const Number* lambda,
		Number obj_value,
		const IpoptData* ip_data,
		IpoptCalculatedQuantities* ip_cq)
{

	for (int i=0; i<n; ++i) {
		minimizer[i] = x[i];
	}

	// Memory deallocation for ADOL-C variables

	delete[] x_lam;

	for(Index i=0;i<m;i++)
		delete[] Jac[i];
	delete[] Jac;

	for(Index i=0;i<n+m+1;i++)
		delete[] Hess[i];
	delete[] Hess;
}

}
