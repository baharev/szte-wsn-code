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
#include <vector>
#include "Model.hpp"
#include "Optimizer.hpp"
#include "SampleReader.hpp"
#include "Variables.hpp"

using namespace std;
using namespace gyro;

enum {
	SUCCESS,
	FAILURE
};

const string str(size_t i) {

	ostringstream os;

	os << i;

	return os.str();
}

const string path_file_name(size_t i) {

	return "path_"+str(i)+".csv";
}

const string avg_file_name(size_t i) {

	return "avg_"+str(i)+".csv";
}

void write_results(const vector<Sample>& slice, const double* x, size_t i) {

//	Variables estimates;

//	const double* const xL = estimates.lower_bounds();
//	const double* const xU = estimates.upper_bounds();

	Model<double>* obj = Model<double>::newInstance(PWL_GYRO_OFFSET, slice);

	obj->init();

	obj->rotate_sum_downwards(x);

	vector3 sum = obj->downward_rotated_sum();

	cout << "Sum as rotated back: " << sum << endl;

	obj->set_v0(x);

	vector3 delta_r = obj->delta_r();

	cout << "Delta r: " << delta_r << endl;

	cout << endl;

	ofstream outfile(path_file_name(i).c_str());

	obj->dump_recomputed_path(x, outfile);

	outfile.close();

	outfile.open(avg_file_name(i).c_str());

	obj->store_path();

	obj->dump_moving_averages(outfile);

	delete obj;
}

void run_optimizer(const vector<Sample>& slice, size_t i) {

	try {

		Optimizer opt(PWL_GYRO_OFFSET, slice);

		write_results(slice, opt.solution(), i);
	}
	catch (...) {

	}
}

void run_window(const vector<Sample>& samples, const vector<int>& periods, size_t i) {

	int per_beg = periods.at(i);

	int per_end = periods.at(i+N_PERIODS);

	Variables::set_current_periods(periods, i);

	const vector<Sample> slice(&samples.at(per_beg), &samples.at(per_end)+1);

	run_optimizer(slice, i);
}

void run_for_each_window(const vector<Sample>& samples, const vector<int>& periods) {

	const size_t runs = periods.size()-N_PERIODS;

	for (size_t i=0; i<runs; ++i) {

		cout << "Run #" << i << endl;

		run_window(samples, periods, i);
	}
}

void real_main(const char* input, const char* output) {

	vector<Sample> samples;

	vector<int> periods;

	SampleReader read(input, samples, "periods.txt", periods);

	run_for_each_window(samples, periods);
}

int main(int argc, char* argv[]) {

	if (argc!=3) {

		cerr << "Usage: " << argv[0] << " input_file  output_file" << endl;
		cerr << "Built on " __DATE__ " " __TIME__ << endl;

		return FAILURE;
	}

//	try {

		real_main(argv[1], argv[2]);
//	}
//	catch (exception& e) {
//
//		cerr << "Runtime error: " << e.what() << endl;
//
//		return FAILURE;
//	}
//	catch (...) {
//
//		cerr << "Unknown exception type (IPOPT?)" << endl;
//
//		return FAILURE;
//	}

	return SUCCESS;
}
