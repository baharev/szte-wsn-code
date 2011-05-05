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
#include <sstream>
#include <stdexcept>
#include <string>
#include "StaticSample.hpp"

using namespace std;

namespace gyro {

StaticSampleReader::StaticSampleReader(const char* file) : in(new ifstream) {

	in->open(file);

	if (!in->is_open()) {

		throw runtime_error("failed to open file "+string(file));
	}

	in->exceptions(ios_base::failbit | ios_base::badbit | ios_base::eofbit);

	readSamples();

	in.reset();
}

void StaticSampleReader::readSamples() {

	string buffer;

	getline(*in, buffer);

	*in >> moteID;

	getline(*in, buffer);

	getline(*in, buffer);

	while (!buffer.empty()) {

		getline(*in, buffer);

		parseLine(buffer);
	}
}

void StaticSampleReader::parseLine(const string& line) {

	if (line.empty()) {

		return;
	}

	istringstream iss(line);

	double acc[3], mag[3], temp;
	char c;

	iss >> acc[0] >> c >> acc[1] >> c >> acc[2] >> c;
	iss >> mag[0] >> c >> mag[1] >> c >> mag[2] >> c;
	iss >> temp;

	samples.push_back(StaticSample(acc, mag));
}

const string StaticSampleReader::mote_id_str() const {

	ostringstream oss;

	oss << moteID << flush;

	return oss.str();
}

StaticSampleReader::~StaticSampleReader() {
	// Do NOT remove, needed to generate dtor of auto_ptr
}

}
