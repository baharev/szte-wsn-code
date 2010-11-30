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

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include "FlatFileDB.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

typedef istringstream iss;

struct Line {

	explicit Line(const string& line);
	Line(int first, int last, int reboot_id)
		: first_block(first), last_block(last), reboot(reboot_id) { }

	void consistent_with(const Line& previous) const;

	int first_block;
	int last_block;
	int reboot;
};

Line::Line(const string& line) {

	iss in(line);

	in.exceptions(iss::failbit | iss::badbit | iss::eofbit);

	in >> first_block;
	in >> last_block;
	in >> reboot;

	if (first_block > last_block || reboot < 1) {
		throw runtime_error("corrupted line");
	}
}

void Line::consistent_with(const Line& previous) const {

	if ((first_block != previous.last_block+1) ||
		(reboot      != previous.reboot   +1) )
	{
		throw runtime_error("corrupted database");
	}
}

FlatFileDB::FlatFileDB(int mote_ID) : mote_id(mote_ID) {

	if (mote_id<=0) {
		logic_error("mote id must be positive");
	}

	string fname = rdb_file_name(mote_id);

	ifstream in;

	in.open(fname.c_str());

	if (in.is_open()) {

		Line previous(0, -1, 0);

		while (in.good()) {

			string buffer;
			getline(in, buffer);

			if (buffer.size()>0) {
				Line current(buffer);
				current.consistent_with(previous);
				previous = current;
				record.push_back(current.first_block);
			}
		}
	}
	else {
		string msg("perhaps records of mote ");
		msg.append(int2str(mote_id));
		msg.append(" have not been downloaded");
		throw runtime_error(msg);
	}

	cout << "DB of mote " << mote_ID << " is opened" << endl;
}

// TODO Replace this mock implementation
int FlatFileDB::reboot(int first_block) {

	if (mote_id==4) {

		throw logic_error("update mock implementation mote 4");
	}

	if (mote_id==5) {

		if (first_block==37435)
			return 42;

		if (first_block==64424)
			return 43;

		throw logic_error("update mock implementation mote 5");
	}

	throw logic_error("update mock implementation mote ID");
}

}
