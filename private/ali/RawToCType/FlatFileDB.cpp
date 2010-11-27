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

class DBLine {

public:

	explicit DBLine(const string& line);

private:

	int first_block;
	int last_block;
	int reboot;
	string length_computed;
	string length_recorded;
	string date_downloaded;
};

DBLine::DBLine(const string& line) {

	iss in(line);

	in.exceptions(iss::failbit | iss::badbit);

	in >> first_block;
	in >> last_block;
	in >> reboot;
	in >> length_computed;
	in >> length_recorded;
	in >> date_downloaded;
}

FlatFileDB::FlatFileDB(int mote_ID) : in(new ifstream), mote_id(mote_ID) {

	if (mote_id<=0) {

		logic_error("mote id must be positive");
	}
/*
	string fname = rdb_file_name(mote_id);

	in->open(fname.c_str());

	if (in->is_open()) {
		// TODO Finish
	}
	else {

	}
*/

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

		throw logic_error("update mock implementation mote 5");
	}

	throw logic_error("update mock implementation mote ID");
}

FlatFileDB::~FlatFileDB() {
	// Do NOT remove this empty dtor: required to generate the dtor of auto_ptr
}

}
