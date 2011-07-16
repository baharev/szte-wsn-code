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

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include "SampleReader.hpp"

using namespace std;

namespace gyro {

SampleReader::SampleReader(const char* samples_file, std::vector<Sample>& samples,
	                       const char* periods_file, std::vector<int>& periods)
:
samples(samples),
periods(periods),
in(new ifstream(samples_file)),
line(1)
{
	init();

//	try {

		read_all_samples();

		cout << "Read " << samples.size() << " samples" << endl;

		read_periods(periods_file);

		cout << "Number of periods: " << periods.size()-1 << endl;

//	}
//	catch (...) {
//
//		cerr << "Fatal error occured when reading line " << line << endl;
//
//		throw;
//	}
}

void SampleReader::init() {

	if (!in->is_open()) {

		throw runtime_error("failed to open the samples file");
	}

	in->exceptions(ios_base::failbit | ios_base::badbit);

	samples.clear();

	samples.reserve(10000);
}

void SampleReader::read_all_samples() {

	while (!in->eof()) {

		getline(*in, buffer);

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

	unsigned int timestamp;

	is >> timestamp;

	double accel[3], gyro[3];

	for (int i=0; i<3; ++i) {

		is >> accel[i];
	}

	for (int i=0; i<3; ++i) {

		is >> gyro[i];
	}

	Sample s = { timestamp, vector3(accel), vector3(gyro) };

	return s;
}

void SampleReader::read_periods(const char* periods_file) {

	in.reset(new ifstream(periods_file));

	if (!in->is_open()) {

		throw runtime_error("failed to open the input file");
	}

	in->exceptions(ios_base::failbit | ios_base::badbit);

	push_back_periods();

	in->close();
}

void SampleReader::push_back_periods() {

	periods.clear();

	line = 1;

	while (!in->eof()) {

		int marker;

		*in >> marker;

		periods.push_back(marker);

		++line;
	}
}

SampleReader::~SampleReader() {
	// Do not remove! Necessary to generate dtor of auto_ptr
}

}
