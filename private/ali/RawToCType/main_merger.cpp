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
#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <string>
#include <sstream>
#include <typeinfo>
#include "FlatFileDB.hpp"
#include "TimeSyncMerger.hpp"
using namespace std;
using namespace sdc;

typedef istringstream iss;

void Main(int mote_id, int record_id) {

	const int n = FlatFileDB(mote_id).number_of_records();

	do {

		cout << "=============================================================";
		cout << "===================" << endl;

		TimeSyncMerger tsm(mote_id, record_id);

		tsm.pairs();

		++record_id;

	} while (record_id<=n);
}

void dump_time_sync_points(ofstream& out, const vector<Pair>& v) {

	const int n = static_cast<int> (v.size());

	for (int i=0; i<n; ++i) {

		const Pair& p = v.at(i);

		out << p.first << '\t' << p.second << endl;
	}
}

void dump(const Map& results) {

	for (Map::const_iterator i=results.begin(); i!=results.end(); ++i) {

		string filename = i->first.toFilenameString();

		ofstream out(filename.c_str());

		out << i->first << endl;

		dump_time_sync_points(out, i->second);
	}
}

void Main2(int mote_id, int record_id) {

	const int n = FlatFileDB(mote_id).number_of_records();

	while (record_id<=n) {

		TimeSyncMerger tsm(mote_id, record_id);

		dump(tsm.pairs());

		++record_id;
	}
}

int str2int(char* str) {

	iss in((string(str)));

	in.exceptions(iss::failbit | iss::badbit);

	int ret_val = -1;

	try {

		in >> ret_val;
	}
	catch (ios_base::failure&) {

		string msg("failed to read ");
		msg.append(str);

		throw runtime_error(msg);
	}

	return ret_val;
}

enum { SUCCESS, FAILURE };

void check_arg_count(int argc, char* argv[]) {

	if (argc != 3) {

		clog << "Usage: " << argv[0] << " mote_id  start_from_record" << endl;

		exit(FAILURE);
	}
}

int main(int argc, char* argv[]) {

	check_arg_count(argc, argv);

	try {

		int mote_id       = str2int(argv[1]);

		int starting_from = str2int(argv[2]);

		Main2(mote_id, starting_from);
	}
	catch (exception& e) {

		clog << "Error: " << e.what() << " (" << typeid(e).name() << ")" << endl;

		return FAILURE;
	}

	return SUCCESS;
}
