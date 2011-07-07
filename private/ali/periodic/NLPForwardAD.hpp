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

#ifndef NLPFORWARDAD_HPP_
#define NLPFORWARDAD_HPP_

#include <vector>
#include "IpTNLP.hpp"
#include "GradType.hpp"
#include "HessType.hpp"
#include "Model.hpp"
#include "Variables.hpp"

using namespace Ipopt;

namespace gyro {

template <typename > class Model;
class Sample;
class Variables;

class NLPForwardAD : public TNLP
{

public:

	NLPForwardAD(ModelType type, const std::vector<Sample>& samples);

	const double* solution() const { return minimizer; }

	virtual ~NLPForwardAD ();

	virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
			Index& nnz_h_lag, IndexStyleEnum& index_style);

	virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
			Index m, Number* g_l, Number* g_u);

	virtual bool get_starting_point(Index n, bool init_x, Number* x,
			bool init_z, Number* z_L, Number* z_U,
			Index m, bool init_lambda,
			Number* lambda);

	virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);

	virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);

	virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);

	virtual bool eval_jac_g(Index n, const Number* x, bool new_x,
			Index m, Index nele_jac, Index* iRow, Index *jCol,
			Number* values);

	virtual bool eval_h(Index n, const Number* x, bool new_x,
			Number obj_factor, Index m, const Number* lambda,
			bool new_lambda, Index nele_hess, Index* iRow,
			Index* jCol, Number* values);

	virtual void finalize_solution(SolverReturn status,
			Index n, const Number* x, const Number* z_L, const Number* z_U,
			Index m, const Number* g, const Number* lambda,
			Number obj_value,
			const IpoptData* ip_data,
			IpoptCalculatedQuantities* ip_cq);

private:

	NLPForwardAD(const NLPForwardAD& );
	NLPForwardAD& operator=(const NLPForwardAD& );

	void fill_Jacobian_sparsity_as_dense(Index* iRow, Index *jCol) const;

	void compute_Jacobian(const double* x, double* values);

	void fill_Hessian_sparsity_as_dense(Index* iRow, Index *jCol) const;

	void compute_Hessian(const double* x, const double obj_factor, const double* lambda, double* values);

	double* const minimizer;

	Model<double>* const            modelDouble;

	Model<GradType<N_VARS> >* const modelGradType;

	Model<HessType<N_VARS> >* const modelHessType;

	Variables* variables;

};

}

#endif // NLPFORWARDAD_HPP_
