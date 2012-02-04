/** Copyright (c) 2010, 2011, 2012 University of Szeged
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
* Author: Ali Baharev
*/

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include "time_parser.hpp"

using namespace std;

namespace sdc {

int count(const string& s, const char c) {

	return count(s.begin(), s.end(), c);
}

template <typename T>
const T convert_to(const string& s) {

	istringstream is(s);

	T value;

	is >> value;

	if (is.fail() || !is.eof()) {

		throw runtime_error("failed to parse "+s);
	}

	return value;
}

template <typename T>
bool is_type(const string& s) {

	istringstream is(s);

	T value;

	is >> value;

	return !is.fail();
}

bool is_index(const string& str) {

	return is_type<int>(str);
}

int to_int(const string& str) {

	int index = 0;

	try {

		index = convert_to<int>(str);
	}
	catch (exception& ) {

		throw runtime_error("parsing \'"+str+"\' as integer failed");
	}

	return index;
}

double hh_mm_ss_to_seconds(const string& s) {
	// 01 2 34 5 678901
    // hh : mm : ss.sss
	if (s.at(2) != ':' || s.at(5)!= ':') {

		throw runtime_error("misplaced \':\' in time string "+s);
	}

	typedef unsigned int uint;

	uint hour  = convert_to<uint>(  s.substr(0, 2));

	uint min   = convert_to<uint>(  s.substr(3, 2));

	double sec = convert_to<double>(s.substr(6   ));

	if (sec < 0) {

		throw runtime_error("invalid value for seconds in "+s);
	}

	double value = 3600*hour + 60*min + sec;

	return value;
}

double to_seconds(const std::string& timestr) {

	const string timestamp = count(timestr, ':')==1 ? "00:"+timestr : timestr;

	double seconds;

	try {

		seconds = hh_mm_ss_to_seconds(timestamp);
	}
	catch (exception& ) {

		throw runtime_error("parsing timestamp \'"+timestr+"\' failed");
	}

	return seconds;
}

bool is_length(const std::string& timestr) {

	return timestr.at(0)=='l' || timestr.at(0)=='L';
}

double length_in_sec(const std::string& timestr) {

	if (!is_length(timestr)) {

		throw logic_error("trying to convert "+timestr+" to time length");
	}

	return to_seconds(timestr.substr(1));
}

}

