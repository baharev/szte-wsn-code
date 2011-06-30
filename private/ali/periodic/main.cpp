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
#include "VarEstimates.hpp"

using namespace std;
using namespace gyro;

enum {
	SUCCESS,
	FAILURE
};

void real_main(const char* input, const char* output) {

	vector<Sample> samples;

	SampleReader read(input, samples);

	cout << "Read " << samples.size() << " samples" << endl;

	Optimizer rot(MINIMIZE_ROTATION, samples);

	const double* const sol = rot.solution();

	cout << "gyro offset: " << endl;

	for (int i=D1; i<=D3; ++i) {

		cout << sol[i] << endl;
	}

	cout << endl;

	//set_new_gyro_offset_estimates(rot.solution());

	Optimizer opt(MINIMIZE_BUMPS, samples);

	VarEstimates estimates;

	const double* const xL = estimates.lower_bounds();
	const double* const xU = estimates.upper_bounds();
	const double* const x = opt.solution();

	for (int i=GRAVITY_X; i<=GRAVITY_Z; ++i) {

		cout << x[i] << '\t' << "( " << xL[i] << ", " << xU[i] << ")" << endl;
	}

	Model<double>* obj = Model<double>::newInstance(MINIMIZE_BUMPS, samples);

	obj->init();

	obj->rotate_sum_downwards(x);

	vector3 sum = obj->downward_rotated_sum();

	cout << "Sum as rotated back: " << sum << endl;

	obj->set_v0(x);

	vector3 delta_r = obj->delta_r();

	cout << "Delta r: " << delta_r << endl;

	cout << "gyro offset: " << endl;

	for (int i=D1; i<=D3; ++i) {

		cout << x[i] << endl;
	}

	cout << endl;

	ofstream outfile("path.csv");

	obj->dump_recomputed_path(x, outfile);

	outfile.close();

	outfile.open("avg.csv");

	obj->store_path();

	obj->dump_moving_averages(outfile);

	delete obj;
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
