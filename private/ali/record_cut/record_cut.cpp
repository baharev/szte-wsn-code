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

#include <iostream>
#include <fstream>
#include <stdexcept>
#include "record_cut.hpp"
#include "time_parser.hpp"
#include "Constants.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

record_cut::record_cut(const std::string& file_name) : infile_name(file_name) {

	ifstream in(file_name.c_str());

	if (!in.good()) {

		throw runtime_error("failed to open file "+file_name);
	}

	string buffer;

	while (getline(in, buffer)) {

		if (!buffer.empty()) {

			samples.push_back(buffer);
		}
	}

	cout << "Lines:  " << number_of_lines() << endl;

	cout << "Length: " << length() << endl;
}

const string record_cut::length() const {

	uint32_t ticks = SAMPLING_RATE*samples.size();

	return ticks2time(ticks);
}

double record_cut::length_in_sec() const {

	double len = samples.size();

	len *= SAMPLING_RATE;

	len /= TICKS_PER_SEC;

	return len;
}

void record_cut::cut(const string& begin, const string& end, const string& offset) const {

	const indices i = to_indices(begin, end, offset);

	cout << "Selected: " << i.first << ", " << i.last << " of " << samples.size()-1 << endl;

}

const record_cut::indices record_cut::to_indices(const string& begin, const string& end, const string& offset) const {

	double first(0), last(0);

	if      (is_length(begin)) {

		last  = get_end(end, offset);

		first = last -  sdc::length_in_sec(begin);
	}
	else if (is_length(end))   {

		first = get_begin(begin, offset);

		last  = first + sdc::length_in_sec(end);
	}
	else {

		first = get_begin(begin, offset);

		last  = get_end(end, offset);
	}

	return to_indices(first, last);
}

double record_cut::get_begin(const string& begin, const string& offset) const {

	double first = 0.0;

	if (begin=="begin" || begin=="beg") {

		first = 0.0;
	}
	else {

		first = to_seconds(begin) - to_seconds(offset);
	}

	return first;
}

double record_cut::get_end(const string& end, const string& offset) const {

	double last = 0.0;

	if (end=="end") {

		last = length_in_sec();
	}
	else {

		last = to_seconds(end) - to_seconds(offset);
	}

	return last;
}


const record_cut::indices record_cut::to_indices(double beg, double end) const {

	const double len = length_in_sec();

	const int n = samples.size() - 1;

	int first = round((beg/len)*n);

	int last  = round((end/len)*n);

	if (first < 0) {

		cout << "Warning: truncating first index from " << first << " to zero!" << endl;

		first = 0;
	}

	if (last > n) {

		cout << "Warning: truncating last index from " << last << " to " << n << "!" << endl;

		last = n;
	}

	if (first >= last) {

		throw runtime_error("invalid index range ("+int2str(first)+", "+int2str(last)+"), revise your timestamps");
	}

	indices i = { first, last };

	return i;
}

}

