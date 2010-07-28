/** Copyright (c) 2010, University of Szeged
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

#include "GyroNLP.hpp"
#include "GradType.hpp"
#include "ObjectiveEvaluator.hpp"
#include "CompileTimeConstants.hpp"

using namespace std;

namespace gyro {

const int GyroNLP::N_VARS(NUMBER_OF_VARIABLES);
const int GyroNLP::N_CONS(0);

class ObjDouble {

public:

	ObjDouble(const input& data, ostream& os, bool verbose)
		: obj(ObjEval<double> (data, os, verbose))
	{

	}

	double evaluate(const double* x) { return obj.f(x); }

private:

	ObjEval<double> obj;
};

class ObjGrad {

public:

	ObjGrad(const input& data, ostream& os, bool verbose) :
		obj(ObjEval<GradType<NUMBER_OF_VARIABLES> > (data, os, verbose))
	{

	}

	void evaluate(const double* x, double* grad_f) {

		GradType<NUMBER_OF_VARIABLES> vars[NUMBER_OF_VARIABLES];

		init_vars(vars, x);

		const GradType<NUMBER_OF_VARIABLES> result = obj.f(vars);

		const double* const g = result.gradient();

		for (int i=0; i<NUMBER_OF_VARIABLES; ++i)
			grad_f[i] = g[i];
	}

private:

	ObjEval<GradType<NUMBER_OF_VARIABLES> > obj;

};

GyroNLP::GyroNLP(const input& data, ostream& os, bool verbose) :
		minimizer(new double[N_VARS]),
		obj(new ObjDouble(data, os, verbose)),
		grad(new  ObjGrad(data, os, verbose))
{

}

GyroNLP::~GyroNLP(){

	delete[] minimizer;
	delete obj;
	delete grad;
}

bool GyroNLP::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
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

bool GyroNLP::get_bounds_info(Index n, Number* x_l, Number* x_u,
		Index m, Number* g_l, Number* g_u)
{

	for (Index i=0; i<n; i++) {
		x_l[i] = -1.0;
		x_u[i] =  1.0;
	}

	// Set the bounds for the constraints
	for (Index i=0; i<m; i++) {
		g_l[i] = 0;
		g_u[i] = 0;
	}

	return true;
}

bool GyroNLP::get_starting_point(Index n, bool init_x, Number* x,
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

void GyroNLP::finalize_solution(SolverReturn status,
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


bool GyroNLP::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
	// TODO new_x -> caching?
	obj_value = obj->evaluate(x);
	return true;
}

bool GyroNLP::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
	// TODO new_x -> caching?
	grad->evaluate(x, grad_f);
	return true;
}

bool GyroNLP::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
	// This problem does not have constraints
	assert(m == 0);
	return true;
}

bool GyroNLP::eval_jac_g(Index n, const Number* x, bool new_x,
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
