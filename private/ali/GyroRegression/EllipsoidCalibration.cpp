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

#include <exception>
#include <iostream>
#include <fstream>
#include "EllipsoidOptimizer.hpp"
#include "Log.hpp"
#include "StaticSample.hpp"

using namespace std;
using namespace gyro;

namespace {

const int SUCCESS = 0;
const int ERROR   = 1;
const int N_VARS  = 9;

}

void print_results(const EllipsoidOptimizer& opt) {

	const double* const x = opt.solution();

	cout << "Solution" << endl;

	for (int i=0; i<N_VARS; ++i) {

		cout << x[i] << endl;
	}

	cout << endl << "1/scale (check if near variable bound)" << endl;

	for (int i=0; i<6; ++i) {

		cout << ((x[i]!=0)? 1.0/x[i] : 0.0) << endl;
	}

	cout << endl << "Max error: " << opt.max_error() << endl << endl;
}

void write_results(fstream& out, const double* x) {

	for (int i=0; i<N_VARS; ++i) {

		out << x[i] << '\n';
	}

	out << flush;
}

void runOptimizer(const StaticSampleReader& reader, CALIB_TYPE type, fstream& out) {

	EllipsoidOptimizer opt(reader.get_samples(), type);

	print_results(opt);

	write_results(out, opt.solution());
}

void realMain(const char* input_file, const char* output_file) {

	StaticSampleReader reader(input_file);

	string outfile(output_file+reader.mote_id_str()+".cal");

	fstream out(outfile.c_str(), ios_base::out);

	out << "# Mote ID\n";

	out << reader.mote_id_str() << '\n';

	out << "# Accelerometer A11, A12, A13, A22, A23, A33, B1, B2, B3; y = A(x-b)\n";

	runOptimizer(reader, ACCELEROMETER, out);

	out << "# Magnetometer A11, A12, A13, A22, A23, A33, B1, B2, B3; y = A(x-b)\n";

	runOptimizer(reader, MAGNETOMETER, out);

	out << '\n';
}

int main(int argc, char* argv[]) {

	if (argc!=3) {

		log_error("invalid number of arguments, specify input and output files");

		return ERROR;
	}

	try {

		realMain(argv[1], argv[2]);

	}
	catch (exception& excp) {

		log_error(excp.what());

		return ERROR;
	}
	catch (...) {

		log_error("unhandled exception");

		return ERROR;
	}

	log_success();

	return SUCCESS;
}
