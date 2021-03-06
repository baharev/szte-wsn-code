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

#include <fstream>
#include <stdexcept>
#include "Tracker.hpp"
#include "BlockIterator.hpp"
#include "Header.hpp"
#include "Line.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

void Tracker::set_filename(int mote_ID) {

	filename = rdb_file_name(mote_ID);
}

void Tracker::process_last_line(const string& line) {

	Line last_record(line);

	first_block = last_record.finished_at_block()+1;

	reboot_id = last_record.reboot_id();
}

void Tracker::find_last_line(ifstream& in) {

	string line;

	while (in.good()) {

		string buffer;

		getline(in, buffer);

		if (buffer.length()) {
			line = buffer;
		}
	}

	if (line.length()!=0) {

		process_last_line(line);
	}
}

void Tracker::set_first_block_reboot_id() {

	first_block = 0;

	reboot_id = 0;

	ifstream in;

	in.open(filename.c_str());

	find_last_line(in);
}

Tracker::Tracker(BlockIterator& zeroth_block) : db(new ofstream()) {

	mote_ID = Header(zeroth_block).mote();

	if (mote_ID==0) {

		throw runtime_error("0 is an invalid mote ID");
	}

	set_filename(mote_ID);

	set_first_block_reboot_id();

	db->exceptions(ofstream::failbit | ofstream::badbit);

	db->open(filename.c_str(), ofstream::app | ofstream::binary);
}

int Tracker::start_from_here() const {

	return first_block;
}

int Tracker::reboot() const {

	return reboot_id;
}

int Tracker::mote_id() const {

	return mote_ID;
}

void Tracker::mark_beginning(int block_beg, int reboot) {

	first_block = block_beg;
	reboot_id = reboot;
}

void Tracker::append_to_db(int last_block, uint32_t time_len) {
	
	*db << Line(first_block, last_block, reboot_id, time_len) << flush;
}

Tracker::~Tracker() {
	// Do NOT remove this empty dtor: required to generate the dtor of auto_ptr
}

}
