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

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include "ampl2gui.hpp"
#include "input2ampl.hpp"

using namespace std;

enum {
	SUCCESS,
	FAILURE
};

void real_main(const string& io_mode, const char* file_name) {

	if      (io_mode=="in") {

		input2ampl converter(file_name);
	}
	else if (io_mode=="out") {

		ampl2gui converter(file_name);
	}
	else {

		ostringstream os;

		os << "io_mode not recognized: " << io_mode << ", should be in or out";

		throw runtime_error(os.str());
	}
}

int main(int argc, char* argv[]) {

	if (argc!=3) {

		cerr << "Usage: " << argv[0] << " io_mode  file_name" << endl;
		cerr << "Built on " __DATE__ " " __TIME__ << endl;

		return FAILURE;
	}

	try {

		real_main(argv[1], argv[2]);
	}
	catch (std::exception& e) {

		cerr << "Runtime error: " << e.what() << endl;

		return FAILURE;
	}
	catch (...) {

		cerr << "Unknown exception type (IPOPT?)" << endl;

		return FAILURE;
	}


	return SUCCESS;
}
