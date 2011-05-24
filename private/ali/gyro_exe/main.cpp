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

#include <iostream>
#include "CompileTimeConstants.hpp"
#include "Optimizer.hpp"
#include "DataIO.hpp"
#include "Result.hpp"
#include "RotationMatrix.hpp"
#include "InputData.hpp"

using namespace std;

using namespace gyro;

void run_solver(const Input& data, const char* outfile) {

	Optimizer opt(data);

	const double* const x = opt.solution();

	const double X[12] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, x[9], x[10], x[11] };

	RotationMatrix rot(data, X);

	Result result = { opt.config_file_id(), opt.error_in_g(), opt.n_vars(),
                      X,
			          opt.var_lb(), opt.var_ub(), data.N(), rot.matrices() };

	write_result(outfile, result);

	for (int i=0; i<12; ++i) {

		cout << X[i] << endl;
	}
}

void real_main(const char* input_filename, const char* output_filename) {

	const Input* data = read_file(input_filename);

	run_solver(*data, output_filename);

	delete data;
}

int main(int argc, char* argv[]) {

	if (argc!=3) {

		cerr << "Usage: " << argv[0] << " input_file_name  output_file_name" << endl;
		cerr << "Built on " __DATE__ " " __TIME__ << endl;;

		return ERROR_ARG_COUNT;
	}

	try {

		real_main(argv[1], argv[2]);
	}
	catch (std::exception& e) {

		cerr << "Runtime error: " << e.what() << endl;

		return ERROR_UNKNOWN;
	}
	catch (...) {

		cerr << "Unknown exception type (IPOPT?)" << endl;

		return ERROR_UNKNOWN;
	}

	return SUCCESS;
}
