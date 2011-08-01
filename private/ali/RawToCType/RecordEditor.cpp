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
#include <sstream>
#include <stdexcept>
#include "RecordEditor.hpp"

using namespace std;

namespace sdc {

// returns true if failed, false if everything is OK
template <typename T>
bool from_string(const string& s, T& value) {

	istringstream is(s);

	is >> value;

	return ( is.fail() || !is.eof() ) ? true : false;
}

// hh:mm:ss to uint seconds, true if failed, false if everything is OK
bool hh_mm_ss_2_uint(const string& s, unsigned int& value) {

	bool failed = true;

	if (s.size() != 8) {

		return failed;
	}

	unsigned int hour(0), min(0), sec(0);

	bool error_h(false), error_m(false), error_s(false);

	error_h = from_string(s.substr(0, 2), hour);

	error_m = from_string(s.substr(3, 2), min);

	error_s = from_string(s.substr(6, 2), sec);

	if (!error_h && !error_m && !error_s) {

		value = 3600*hour + 60*min + sec;

		failed = false;
	}

	return failed;
}

RecordEditor::RecordEditor(const char* input_record_name) {

	ifstream in(input_record_name);

	if (!in.good()) {

		throw runtime_error("failed to open the input file");
	}

	start_at_global_time = 0;

	samples.reserve(1500000);

	while (in.good()) {

		string buffer;

		getline(in, buffer);

		samples.push_back(buffer);
	}

	cout << "Read " << samples.size() << " lines" << endl;
}

void RecordEditor::run() {

	get_start();

}

void RecordEditor::get_start() {

	while (ask_for_start()) {

		;
	}
}

bool RecordEditor::ask_for_start() {

	cout << "Enter the time of start in hh:mm:ss format or hit enter for 00:00:00" << endl;

	bool ask_again = false;

	string buffer;

	getline(cin, buffer);

	if (buffer.empty()) {

		start_at_global_time = 0;
	}
	else {

		ask_again = hh_mm_ss_2_uint(buffer, start_at_global_time);
	}

	return ask_again;
}

}
