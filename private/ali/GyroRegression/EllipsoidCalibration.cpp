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
#include "EllipsoidOptimizer.hpp"
#include "Log.hpp"
#include "StaticSample.hpp"

using namespace gyro;

namespace {

const int SUCCESS = 0;
const int ERROR   = 1;

}

void realMain(int argc, char* argv[]) {

	StaticSampleReader reader(argv[1]);

	const std::vector<StaticSample> samples = reader.get_samples();

	EllipsoidOptimizer opt(samples);

	using namespace std;

	const double* x = opt.solution();

	for (int i=0; i<9; ++i) {

		cout << x[i] << endl;
	}
}

int main(int argc, char* argv[]) {

	if (argc!=3) {

		log_error("invalid number of arguments, specify input and output files");

		return ERROR;
	}

	try {

		realMain(argc, argv);

	}
	catch (std::exception& excp) {

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
