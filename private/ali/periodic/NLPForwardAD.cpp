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
#include "VarEstimates.hpp"

namespace gyro {

NLPForwardAD::NLPForwardAD(ModelType type, const std::vector<Sample>& samples) :
		minimizer(new double[N_VARS]),
		modelDouble(Model<double>::newInstance(type, samples)),
		modelGradType(Model<GradType<N_VARS> >::newInstance(type, samples)),
		modelHessType(Model<HessType<N_VARS> >::newInstance(type, samples)),
		variables(new Variables)
{

}

NLPForwardAD::~NLPForwardAD() {

	delete[] minimizer;
	delete modelDouble;
	delete modelGradType;
	delete modelHessType;
	delete variables;
}

bool NLPForwardAD::get_bounds_info(Index n, Number* x_l, Number* x_u,
		                           Index m, Number* g_l, Number* g_u)
{

	const double* xL = variables->lower_bounds();
	const double* xU = variables->upper_bounds();

	for (int i=0; i<N_VARS; ++i) {
		x_l[i] = xL[i];
		x_u[i] = xU[i];
	}

	const int N_CONS = modelDouble->number_of_constraints();

	for (Index i=0; i<N_CONS; ++i) {
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
	const double* x0 = variables->initial_point();

	for (int i=0; i<N_VARS; ++i) {

		x[i] = x0[i];
	}

	return true;
}

bool NLPForwardAD::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
		Index& nnz_h_lag, IndexStyleEnum& index_style)
{
	n = N_VARS;

	m = modelDouble->number_of_constraints();

	// Dense Jacobian
	nnz_jac_g = n*m;

	// Dense Hessian, only lower left part is stored
	nnz_h_lag = (n*(n-1))/2+n; // In the ADOLC code, it is a bug without the parentheses

	index_style = C_STYLE;

	return true;
}

bool NLPForwardAD::eval_f(Index n, const Number* x, bool new_x, Number& obj_value) {

	obj_value = modelDouble->objective(x);

	return true;
}

bool NLPForwardAD::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f) {

	GradType<N_VARS> vars[N_VARS];

	init_vars(vars, x);

	const GradType<N_VARS> obj = modelGradType->objective(vars);

	obj.copy_gradient(grad_f);

	return true;
}

bool NLPForwardAD::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g) {

	if (modelDouble->number_of_constraints()) {

		const std::vector<double> con = modelDouble->constraints(x);

		std::copy(con.begin(), con.end(), g);
	}

	return true;
}

bool NLPForwardAD::eval_jac_g(Index n, const Number* x, bool new_x,
		Index m, Index nele_jac, Index* iRow, Index *jCol, Number* values)
{
	if (modelDouble->number_of_constraints()==0) {

		; // calling model->constraints() may crash
	}
	else if (values == 0) {

		fill_Jacobian_sparsity_as_dense(iRow, jCol);
	}
	else {

		compute_Jacobian(x, values);
	}

	return true;
}

void NLPForwardAD::fill_Jacobian_sparsity_as_dense(Index* iRow, Index *jCol) const {

	Index idx = 0;

	const int N_CONS = modelDouble->number_of_constraints();

	for (Index i=0; i<N_CONS; ++i) {

		for (Index j=0; j<N_VARS; ++j) {
			iRow[idx] = i;
			jCol[idx] = j;
			++idx;
		}
	}
}

void NLPForwardAD::compute_Jacobian(const double* x, double* values) {

	GradType<N_VARS> vars[N_VARS];

	init_vars(vars, x);

	const std::vector<GradType<N_VARS> > con = modelGradType->constraints(vars);

	const int N_CONS = modelDouble->number_of_constraints();

	for (int i=0; i<N_CONS; ++i) {

		con.at(i).copy_gradient(values+i*N_VARS); // Assumes the sparsity pattern!
	}
}

bool NLPForwardAD::eval_h(Index n, const Number* x, bool new_x,
		Number obj_factor, Index m, const Number* lambda,
		bool new_lambda, Index nele_hess, Index* iRow,
		Index* jCol, Number* values)
{
	if (values == 0) {

		fill_Hessian_sparsity_as_dense(iRow, jCol);
	}
	else {

		compute_Hessian(x, obj_factor, lambda, values);
	}

	return true;
}

void NLPForwardAD::fill_Hessian_sparsity_as_dense(Index* iRow, Index *jCol) const {

	Index idx=0;

	for (Index row = 0; row < N_VARS; ++row) {

		for (Index col = 0; col <= row; ++col) { // Only the lower left part is stored
			iRow[idx] = row;
			jCol[idx] = col;
			++idx;
		}
	}
}

void NLPForwardAD::compute_Hessian(const double* x,
		                           const double obj_factor,
		                           const double* lambda,
		                           double* values)
{
	HessType<N_VARS> vars[N_VARS];

	init_vars(vars, x);

	HessType<N_VARS> Lagrangian = obj_factor*(modelHessType->objective(vars));

	std::vector<HessType<N_VARS> > g;

	const int N_CONS = modelDouble->number_of_constraints();

	if (N_CONS) {

		g = modelHessType->constraints(vars);
	}

	for (int i=0; i<N_CONS; ++i) {

		Lagrangian += lambda[i]*g.at(i);
	}

	Lagrangian.copy_hessian(values); // Assumes the sparsity pattern!
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
}

}
