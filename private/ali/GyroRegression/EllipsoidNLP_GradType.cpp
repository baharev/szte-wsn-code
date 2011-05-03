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

#ifdef USE_GRADTYPE

#include "EllipsoidNLP_GradType.hpp"
#include "EllipsoidObjective.hpp"
#include "GradType.hpp"

namespace gyro {

const int N_CONS(0);

class ObjDouble {

public:

	ObjDouble(const std::vector<StaticSample>& samples)
	: obj(EllipsoidObjective<double> (samples))
	{ }

	double evaluate(const double* x) { return obj.accel(x); }

private:

	EllipsoidObjective<double> obj;
};

class ObjGrad {

public:

	ObjGrad(const std::vector<StaticSample>& samples)
	: obj(EllipsoidObjective<GradType<N_VARS> > (samples))
	{

	}

	void evaluate(const double* x, double* grad_f) {

		GradType<N_VARS> vars[N_VARS];

		init_vars(vars, x);

		const GradType<N_VARS> result = obj.accel(vars);

		const double* const g = result.gradient();

		for (int i=0; i<N_VARS; ++i)
			grad_f[i] = g[i];
	}

private:

	EllipsoidObjective<GradType<N_VARS> > obj;

};

EllipsoidNLP::EllipsoidNLP(const std::vector<StaticSample>& samples) :
		minimizer(new double[N_VARS]),
		obj(new ObjDouble(samples)),
		grad(new  ObjGrad(samples))
{

}

EllipsoidNLP::~EllipsoidNLP(){

	delete[] minimizer;
	delete obj;
	delete grad;
}

bool EllipsoidNLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
		Index& nnz_h_lag, IndexStyleEnum& index_style)
{
	n = N_VARS;

	m = N_CONS;

	// The dense Jacobian contains n*m nonzeros
	nnz_jac_g = n*m;

	// The dense Hessian has n*n total nonzeros, only the lower left corner is
	// stored since it is symmetric
	nnz_h_lag = n*(n-1)/2+n;

	// use the C style indexing (0-based)
	index_style = C_STYLE;

	return true;
}

bool EllipsoidNLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
		Index m, Number* g_l, Number* g_u)
{
	assert(n==N_VARS);
	const int N_ELEM(6);

	for (int i=0; i<N_ELEM; ++i) {
		x_l[i] =-0.1;
		x_u[i] = 0.1;
	}

	x_l[0] = x_l[3] = 0.0;

	for (int i=N_ELEM; i<N_VARS; ++i) {
		x_l[i] =  1000.0;
		x_u[i] =  3000.0;
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

	// set the starting point
	for (Index i=0; i<n; i++)
		x[i] = 0.0;

	return true;
}

void EllipsoidNLP::finalize_solution(SolverReturn status,
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

bool EllipsoidNLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
	// TODO new_x -> caching?
	obj_value = obj->evaluate(x);
	return true;
}

bool EllipsoidNLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
	// TODO new_x -> caching?
	grad->evaluate(x, grad_f);
	return true;
}

bool EllipsoidNLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
	// This problem does not have constraints
	assert(m == 0);
	return true;
}

bool EllipsoidNLP::eval_jac_g(Index n, const Number* x, bool new_x,
		Index m, Index nele_jac, Index* iRow, Index *jCol,
		Number* values)
{
	if (values == NULL) {
		// return the structure of the jacobian,
		// assuming that the Jacobian is dense

		Index idx = 0;
		for(Index i=0; i<m; i++)
			for(Index j=0; j<n; j++)
			{
				iRow[idx] = i;
				jCol[idx++] = j;
			}
	}
	else {
		// return the values of the jacobian of the constraints

		// This problem does not have constraints
		assert(m == 0);
	}

	return true;
}

}

#endif
