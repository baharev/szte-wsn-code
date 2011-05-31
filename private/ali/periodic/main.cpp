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
#include <vector>
#include "Optimizer.hpp"
#include "Sample.hpp"
#include "VarEnum.hpp"

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

	Optimizer opt(samples);

	const double* const x = opt.solution();

	for (int i=D1; i<=D3; ++i) {

		cout << x[i] << endl;
	}
}

int main(int argc, char* argv[]) {

	if (argc!=3) {

		cerr << "Usage: " << argv[0] << " input_file  output_file" << endl;
		cerr << "Built on " __DATE__ " " __TIME__ << endl;

		return FAILURE;
	}

	try {

		real_main(argv[1], argv[2]);
	}
	catch (exception& e) {

		cerr << "Runtime error: " << e.what() << endl;

		return FAILURE;
	}
	catch (...) {

		cerr << "Unknown exception type (IPOPT?)" << endl;

		return FAILURE;
	}


	return SUCCESS;
}
