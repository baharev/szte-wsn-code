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
#include "Sample.hpp"

using namespace std;

namespace gyro {

SampleReader::SampleReader(const char* input, vector<Sample>& samples) :
samples(samples),
ptr_in(new fstream(input)),
in(*ptr_in),
line(1)
{
	init();

	try {

		read_all();
	}
	catch (...) {

		cerr << "Fatal error occured when reading line " << line << endl;

		throw;
	}

	in.close();
}

void SampleReader::init() {

	if (!in.is_open()) {

		throw runtime_error("failed to open the input file");
	}

	in.exceptions(ios_base::failbit | ios_base::badbit);

	samples.clear();

	samples.reserve(10000);
}

void SampleReader::read_all() {

	while (!in.eof()) {

		getline(in, buffer);

		push_back_sample();

		++line;
	}
}

void SampleReader::push_back_sample() {

	if (buffer.empty()) {

		return;
	}

	const Sample s = parse_buffer();

	samples.push_back(s);
}

const Sample SampleReader::parse_buffer() const {

	istringstream is(buffer);

	is.exceptions(ios_base::failbit | ios_base::badbit | ios_base::eofbit);

	unsigned int timestamp, accel[3], gyro[3];

	is >> timestamp;

	for (int i=0; i<3; ++i) {

		is >> accel[i];
	}

	for (int i=0; i<3; ++i) {

		is >> gyro[i];
	}

	Sample s = { timestamp, vector3(accel[X], accel[Y], accel[Z]),
			                vector3( gyro[X],  gyro[Y],  gyro[Z]) };

	return s;
}

}
