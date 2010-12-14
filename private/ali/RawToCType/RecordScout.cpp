/* Copyright (c) 2010, University of Szeged
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
// FIXME Remove when finished
#include <iostream>
#include "RecordScout.hpp"
#include "MoteRegistrar.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

void RecordScout::read_all_existing() {

	records.clear();

	vector<int> ids = MoteRegistrar::existing_ids();

	const int n = static_cast<int> (ids.size());

	for (int i=0; i<n; ++i) {

		mote_id = ids.at(i);

		read_mote_rdb();
	}
}

void RecordScout::read_mote_rdb() {

	ifstream in;

	in.open(rdb_file_name(mote_id).c_str());

	while (in.good()) {

		string buffer;

		getline(in, buffer);

		push_line(buffer);
	}
}

void RecordScout::push_line(const string& buffer) {

	if (buffer.size()!=0) {

		records.push_back(Line(buffer));
	}
}

void RecordScout::dump() const {

	const int n = static_cast<int> (records.size());

	for (int i=0; i<n; ++i) {

		Line line = records.at(i);

		cout << line << endl;
	}
}

}
