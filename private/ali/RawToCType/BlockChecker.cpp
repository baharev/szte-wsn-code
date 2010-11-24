/** Copyright (c) 2010, University of Szeged
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
#include <cstdlib>
#include "BlockChecker.hpp"
#include "BlockIterator.hpp"
#include "Constants.hpp"
#include "BlockRelatedConsts.hpp"

using std::cout;
using std::endl;
using std::abs;

namespace sdc {

void fix_counter_overflow(int& i) {

	if (i < 0)
		i += 0x10000;
}

BlockChecker::BlockChecker(int mote_id) : mote_ID(mote_id) {

	samples_processed = 0;

	timesync.set_timesync_zero();

	new_time_sync_info = false;

	// on the first call reboot() needs nonzero for previous.counter()
	// current will become previous by then
	current.force_counter(1);

}


void BlockChecker::reset_time_sync() {

	new_time_sync_info = false;
	timesync.set_timesync_zero();
}

void BlockChecker::set_current_header(BlockIterator& i, int offset) {

	header = Header(i);
	block_offset = offset;

	if ( header.timesync_differs_from(timesync) ) {

		timesync = header;
		new_time_sync_info = true;
	}
	else {

		new_time_sync_info = false;
	}
}

bool BlockChecker::time_sync_info_is_new() const {

	return new_time_sync_info;
}

const Header& BlockChecker::get_timesync() const {

	return timesync;
}

void BlockChecker::mote_id() const {

	if (header.mote() != mote_ID) {

		cout << "Warning: mote id " << header.mote() << " in block ";
		cout << block_offset << " differs from " << mote_ID << endl;
	}
}

bool BlockChecker::finished() const {

	return header.data_length()==0;
}

bool BlockChecker::datalength() const {

	bool is_ok = true;

	if (MAX_SAMPLES != header.data_length()/SAMPLE_LENGTH) {

		cout << "Warning: invalid length " << header.data_length();
		cout << " in block " << block_offset << endl;
		is_ok = false;
	}

	return is_ok;
}

void BlockChecker::set_current(const Sample& s) {

	previous = current;
	current = s;

	++samples_processed;
}

bool BlockChecker::reboot() const {
	// FIXME Bug: reboot may be unnoticed
	return current.counter()==1 && previous.counter()!=0;
}

void BlockChecker::counter() const {

	int diff = current.counter()-previous.counter();

	fix_counter_overflow(diff);

	int missed = diff-1;

	if (missed) {

		cout << "Warning: at sample " << samples_processed;
		cout << " missing at least ";
		cout << missed << " samples" << endl;
	}
}

void BlockChecker::timestamp() const {

	uint32 expected = previous.timestamp() + SAMPLING_RATE;

	int error_in_ticks = current.timestamp() - expected;

	if (abs(error_in_ticks)>TOLERANCE) {

		cout << "Warning: at sample " << samples_processed << " ";
		cout << error_in_ticks << " ticks error" << endl;
	}
}

int BlockChecker::line() const {

	return samples_processed;
}

void BlockChecker::reset_line_counter() {

	samples_processed = 0;
}

unsigned int BlockChecker::get_current_timestamp() const {

	return current.timestamp();
}

unsigned int BlockChecker::get_previous_timestamp() const {

	return previous.timestamp();
}

}

