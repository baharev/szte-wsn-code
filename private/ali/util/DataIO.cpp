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

#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <cstdlib>
#include <cmath>
#include "DataIO.hpp"
#include "InputData.hpp"
#include "Result.hpp"
#include "CompileTimeConstants.hpp"

using namespace std;

namespace gyro {

void skip_line(istream& in, const char* text) {

	string line;
	getline(in, line);
	if (line != text) {
		cerr << "Expected explanatory comment: " << text << endl;
		cerr << "Found: " << line << endl;
		exit(ERROR_READING_INPUT);
	}
}

void check_time_stamp(int sample, double current) {

	const double SAMPLING_RATE(160);
	const double GAP_TRESHOLD(40);

	static double previous = current - SAMPLING_RATE;

	if (current <= previous) {
		cerr << "Timestamp error at sample " << sample << " : ";
		cerr << previous << " >= " << current << endl;
		exit(ERROR_INCORRECT_TIMESTAMP);
	}
	else if (fabs(current-previous-SAMPLING_RATE)>GAP_TRESHOLD) {
		clog << "Missing samples at " << sample << ", gap = ";
		clog << fabs(current-previous-SAMPLING_RATE) << endl;
	}

	previous = current;
}

Input* grab_content(const char* filename) {

	double dt    = SAMPLING_RATE_TICK/TICKS_PER_SEC;
	double g_ref = GRAVITY;

	ifstream in;

	in.exceptions(ifstream::eofbit | ifstream::failbit | ifstream::badbit);

	in.open(filename);

	skip_line(in, NUMBER_OF_SAMPLES);

	int N = -1;

	in >> N;

	if (N<1) {
		cerr << endl << "Invalid number of samples!" << endl;
		exit(ERROR_READING_INPUT);
	}

	string endline;

	getline(in, endline);

	skip_line(in, INPUT_DATA);

	double* time_stamp = new double[N];

	double* wx = new double[N];
	double* wy = new double[N];
	double* wz = new double[N];

	double* acc_x = new double[N];
	double* acc_y = new double[N];
	double* acc_z = new double[N];

	for (int i=0; i<N; ++i) {

		in >> time_stamp[i];

		check_time_stamp(i, time_stamp[i]);

		in >> acc_x[i];
		in >> acc_y[i];
		in >> acc_z[i];

		in >> wx[i];
		in >> wy[i];
		in >> wz[i];

	}

	// TODO End of file line?
	in.close();

	return new Input(time_stamp, acc_x, acc_y, acc_z, wx, wy, wz, N, dt, g_ref);
}

Input* read_file(const char* filename) {

	Input* input = 0;

	try {

		input = grab_content(filename);
	}
	catch (...) {

		cerr << "Unexpected error occurred when reading the input file: ";
		cerr << filename << endl;
		exit(ERROR_READING_INPUT);
	}

	return input;
}

void print_vector(ostream& out, const double* x, const int length) {

	for (int i=0; i<length; ++i) {
		out << x[i] << '\n';
	}
}

void print_rotation_matrices(ostream& out, const double* const R, const int N) {

	out << scientific;

	out << setprecision(16);

	for (int k=0; k<N; ++k) {

		const double* const mat = R + 9*k;

		for (int i=0; i<9; ++i) {

			out << mat[i] << '\n';
		}
	}

	out << flush;
}

void print_result(const char* filename, const Result& res) {

	ofstream out;

	out.exceptions(ofstream::failbit | ofstream::badbit);

	out.open(filename);

	out << '\n' << FIRST_LINE << '\n';

	out << BUILD_ID << '\n';

	out << CONFIG_FILE_ID << '\n';
	out << res.config_file_id << '\n';

	out << ERROR_IN_G << '\n';
	out << res.error_in_g << '\n';

	out << NUMBER_OF_VARS << '\n';
	out << res.n_vars << '\n';

	out << SOLUTION_VECTOR << '\n';
	print_vector(out, res.solution, res.n_vars);

	out << VARIABLE_LOWER_BOUNDS << '\n';
	print_vector(out, res.var_lb, res.n_vars);

	out << VARIABLE_UPPER_BOUNDS << '\n';
	print_vector(out, res.var_ub, res.n_vars);

	out << NUMBER_OF_SAMPLES << '\n';
	out << res.N_SAMPLES << '\n';

	out << ROTATION_MATRICES << '\n';
	print_rotation_matrices(out, res.rotation_matrices, res.N_SAMPLES);

	out << END_OF_FILE << '\n';
	out << flush;
}

void write_result(const char* filename, const Result& res) {

	try {

		print_result(filename, res);
	}
	catch(...) {
		cerr << "Unexpected error when writing the results into file ";
		cerr << filename << endl;
		exit(ERROR_WRITING_RESULTS);
	}
}

}
