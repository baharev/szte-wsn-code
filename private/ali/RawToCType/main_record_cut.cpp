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
#include <fstream>
#include <string>
#include "Utility.hpp"

using namespace std;

namespace {

const char infile_name[]  = "record.csv";

const char outfile_name[] = "record_cut.csv";

const unsigned int TICKS_PER_SEC = 32768;

const unsigned int SAMPLING_RATE = 160;

const double sampling_rate_hz = 204.8;

}

int count_lines() {

	int lines = 0;

	ifstream in(infile_name);

	while (in.good()) {

		string dummy;

		getline(in, dummy);

		++lines;
	}

	return lines;
}

int lines_to_skip(int seconds_to_keep_from_the_back) {

	int lines = count_lines();

	int lines_to_keep = sampling_rate_hz * seconds_to_keep_from_the_back;

	int skip = lines - lines_to_keep;

	if (skip < 0) {

		skip = 0;
	}

	return skip;
}

void skip_lines(ifstream& in, int lines_to_skip) {

	for (int i=0; i<lines_to_skip; ++i) {

		string dummy;

		getline(in, dummy);
	}
}

void dump_lines_until_eof(ifstream& in) {

	int counter = 0;

	ofstream out(outfile_name);

	while (!in.eof()) {

		string buffer;

		getline(in, buffer);

		out << counter << ',' << buffer << '\n';

		++counter;
	}

	cout << "Lines written: " << counter << endl;
}

void dump_last_seconds(int sec) {

	int lines_skipped = lines_to_skip(sec);

	ifstream in(infile_name);

	skip_lines(in, lines_skipped);

	dump_lines_until_eof(in);
}

void show_approx_lenght() {

	int lines = count_lines();

	unsigned int ticks = SAMPLING_RATE*lines;

	string length = sdc::ticks2time(ticks);

	cout << "Approximate length is " << length << endl;

}

int main() {

	dump_last_seconds(120);
}
