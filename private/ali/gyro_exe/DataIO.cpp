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
#include <fstream>
#include <cstdlib>
#include "DataIO.hpp"
#include "InputData.hpp"
#include "Optimizer.hpp"
#include "RotationMatrix.hpp"
#include "CompileTimeConstants.hpp"

using namespace std;

typedef double NT;

namespace gyro {

void print_vector(ostream& out, const double* x, const int length) {

	for (int i=0; i<length; ++i) {
		out << x[i] << '\n';
	}
}

void print_result(const char* filename,
				const Optimizer& opt,
				const Input& data,
				const RotationMatrix& rot)
{
	ofstream out;

	out.exceptions(ofstream::failbit | ofstream::badbit);

	out.open(filename);

	out << '\n' << FIRST_LINE << '\n';

	out << BUILD_ID << '\n';

	out << CONFIG_FILE_ID << '\n';
	out << opt.config_file_id() << '\n';

	out << ERROR_IN_G << '\n';
	out << opt.error_in_g() << '\n';

	out << NUMBER_OF_VARS << '\n';
	out << opt.n_vars() << '\n';

	out << SOLUTION_VECTOR << '\n';
	print_vector(out, opt.solution(), opt.n_vars());

	out << VARIABLE_LOWER_BOUNDS << '\n';
	print_vector(out, opt.var_lb(), opt.n_vars());

	out << VARIABLE_UPPER_BOUNDS << '\n';
	print_vector(out, opt.var_ub(), opt.n_vars());

	out << NUMBER_OF_SAMPLES << '\n';
	out << data.N() << '\n';

	out << ROTATION_MATRICES << '\n';
	rot.dump_matrices(out);

	out << END_OF_FILE << '\n';
	out << flush;
}

void write_result(const char* filename,
				const Optimizer& opt,
				const Input& data,
				const RotationMatrix& rot)
{

	try {

		print_result(filename, opt, data, rot);
	}
	catch(...) {
		cerr << "Unexpected error when writing the results into file ";
		cerr << filename << endl;
		exit(ERROR_WRITING_RESULTS);
	}
}

}
