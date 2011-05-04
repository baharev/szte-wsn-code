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

#include <stdexcept>
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"
#include "EllipsoidOptimizer.hpp"
#include "EllipsoidNLP_GradType.hpp"

namespace gyro {

EllipsoidOptimizer::EllipsoidOptimizer(const std::vector<StaticSample>& samples)
: minimizer(new double[N_VARS])
{
	try {

		init(samples);
	}
	catch (std::exception& ) {

		throw;
	}
	catch(...) {

		throw std::runtime_error("unknown error during optimization");
	}
}

void EllipsoidOptimizer::init(const std::vector<StaticSample>& samples) {

	SmartPtr<IpoptApplication> app = new IpoptApplication();

	SmartPtr<OptionsList> opt = app->Options();

	opt->SetNumericValue("tol", 1.0e-4);
	opt->SetIntegerValue("print_level", 0);
	opt->SetStringValue("output_file", "ipopt.log");
	opt->SetIntegerValue("file_print_level", 5);
	opt->SetStringValue("hessian_approximation", "limited-memory");
	opt->SetStringValue("limited_memory_update_type", "bfgs");
	//opt->SetStringValue("derivative_test","first-order");
	//opt->SetStringValue("derivative_test_print_all","yes");

	ApplicationReturnStatus status(app->Initialize("ipopt.opt"));

	if (status != Solve_Succeeded) {

		throw std::runtime_error("initialization of IPOPT failed");
	}

	EllipsoidNLP* const nlp = new EllipsoidNLP(samples);

	status = app->OptimizeTNLP(SmartPtr<TNLP>(nlp));

	check_return_code(status);

	copy_solution(nlp->solution());
}

void EllipsoidOptimizer::check_return_code(int status) const {

	if (status == Invalid_Option) {

		throw std::runtime_error("invalid IPOPT option");
	}
	else if (status == Invalid_Number_Detected) {

		throw std::runtime_error("function or derivative evaluation failed (NaN?)");
	}
	else if (status != Solve_Succeeded && status != Solved_To_Acceptable_Level) {

		throw std::runtime_error("optimization failed, check the log");
	}
}

void EllipsoidOptimizer::copy_solution(const double* const x) {

	double* const sol = minimizer.get();

	for (int i=0; i<N_VARS; ++i) {

		sol[i] = x[i];
	}
}

}
