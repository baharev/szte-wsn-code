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

#include <ctime>
#include <iomanip>
#include <sstream>
#include "Constants.hpp"
#include "BlockRelatedConsts.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

const string ticks2time(unsigned int t) {

	ostringstream os;

	unsigned int hour, min, sec, milli;

	hour = t/(3600*TICKS_PER_SEC);
	t =    t%(3600*TICKS_PER_SEC);

	min = t/(60*TICKS_PER_SEC);
	t   = t%(60*TICKS_PER_SEC);

	sec = t/TICKS_PER_SEC;
	t   = t%TICKS_PER_SEC;

	milli = static_cast<unsigned int> ( t/(TICKS_PER_SEC/1000.0) );

	os << setfill('0') << setw(2) << hour << ":";
	os << setfill('0') << setw(2) << min  << ":";
	os << setfill('0') << setw(2) << sec  << ".";
	os << setfill('0') << setw(3) << milli<< flush;

	return os.str();
}

const string current_time() {

	time_t t;
	time(&t);
	return string(ctime(&t));
}

const string get_filename(int mote_id, int reboot_id, int first_block) {

	ostringstream os;

	os << 'm' << setfill('0') << setw(3) << mote_id << '_';
	os << 'r' << setfill('0') << setw(3) << reboot_id << '_';
	os << 'b' << first_block;

	os.flush(); // TODO Is it needed?

	return os.str();
}

const string time_to_filename() {

	string time_stamp(current_time().substr(4, 20));

	time_stamp.erase(3, 1);
	time_stamp.at(5) = '_';
	time_stamp.erase(8, 1);
	time_stamp.erase(10, 1);
	time_stamp.at(12) = '_';
	time_stamp.erase(13, 2);

	return time_stamp;
}

const string recorded_length(int first_block, int last_block) {

	int n_blocks = last_block-first_block+1;

	int n_samples = n_blocks*MAX_SAMPLES-1;

	unsigned int length_in_ticks = n_samples*SAMPLING_RATE;

	return ticks2time(length_in_ticks);
}

const string failed_to_read_block(int i) {

	ostringstream os;

	os << "Failed to read block " << i << flush;

	return os.str();
}

}
