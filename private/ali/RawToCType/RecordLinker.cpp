/* Copyright (c) 2010, 2011 University of Szeged
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
#include "RecordLinker.hpp"

using namespace std;

namespace sdc {

RecordLinker::RecordLinker(const char* filename) : out(new ofstream(filename)) {

	if (!out->is_open()) {

		throw runtime_error("failed to create output file");
	}

	out->exceptions(ios_base::failbit | ios_base::badbit);

	*out << "# Mote and reboot IDs involved\n";
}

void RecordLinker::write_participant(int mote, int record) {

	*out << mote << '/' << record << '\t';
}

void RecordLinker::write_first_record(const RecordInfo& rec_info, unsigned int boot_utc) {

	*out << "\n# Time of start in time_t, UTC (zero of unknown)\n";
	*out << boot_utc << '\n';
	*out << "# Length of record\n";
	*out << rec_info.length() << 'n';

}


void RecordLinker::write_pair(const RecordID& rid, double skew_1, double offset) {

}

RecordLinker::~RecordLinker() {

	out << '\n' << flush;
	// Do NOT remove this empty dtor: required to generate the dtor of auto_ptr
}

}
