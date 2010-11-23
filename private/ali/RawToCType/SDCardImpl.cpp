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

#include <stdexcept>
#include "SDCardImpl.hpp"
#include "BlockDevice.hpp"
#include "BlockIterator.hpp"
#include "BlockRelatedConsts.hpp"
#include "BlockChecker.hpp"
#include "DataWriter.hpp"
#include "Tracker.hpp"

using namespace std;

namespace sdc {

SDCardImpl::SDCardImpl(BlockDevice* source)
	: device(source), out(new DataWriter), tracker(0), check(0), console(Console())
{
	time_start = 0;

	block_offset = 0;

	reboot_seq_num = 0;

	init_tracker();

	check.reset(new BlockChecker(tracker->mote_id()));
}

void SDCardImpl::init_tracker() {

	const char* const block = device->read_block(0);

	BlockIterator zeroth_block(block);

	tracker.reset(new Tracker(zeroth_block));
}

double SDCardImpl::size_GB() const {

	return device->size_GB();
}

void SDCardImpl::process_new_measurements() {

	bool finished = false;

	block_offset = tracker->start_from_here();

	reboot_seq_num = tracker->reboot();

	console.start(device->size_GB(), tracker->mote_id(), block_offset, reboot_seq_num);

	while (!finished ) {

		const char* const block = device->read_block(block_offset);

		finished = process_block(block);

		++block_offset;
	}

	close_out_if_open();

	console.finished(device->size_GB(), block_offset);
}

void SDCardImpl::close_out_if_open() {

	if (out->is_open()) {

		uint32 length_in_ticks = check->get_previous_timestamp()-time_start;

		tracker->append_to_db(block_offset-1, length_in_ticks);

		console.record_end(block_offset-1, length_in_ticks);

		out->close();
	}
}

bool SDCardImpl::reboot(const int sample_in_block) {

	bool reboot = check->reboot();

	if ((reboot && sample_in_block==0) || !out->is_open()) {

		close_out_if_open();

		++reboot_seq_num;

		check->reset_line_counter();

		check->reset_time_sync();

		time_start = check->get_current_timestamp();

		console.record_start(reboot_seq_num, block_offset);

		out->start_new_record(tracker->mote_id(), reboot_seq_num, block_offset);

		tracker->mark_beginning(block_offset, reboot_seq_num);

		return true;
	}
	// FIXME Should not we log if sample_in_block != 0 ?

	return false;
}

void SDCardImpl::check_sample(const int sample_in_block) {

	// TODO Maybe this should be pushed into BlockChecker
	if (!reboot(sample_in_block)) {

		check->counter();

		check->timestamp();
	}
}

void SDCardImpl::write_samples(BlockIterator& itr) {

	for (int i=0; i<MAX_SAMPLES; ++i) {

		Sample s(itr);

		check->set_current(s);

		check_sample(i);

		s.shift_timestamp(time_start);

		out->write(s);
	}

	out->flush();
}

void SDCardImpl::write_time_sync_info() {

	if (check->time_sync_info_is_new()) {

		out->write_time_sync_info(check->get_timesync());
	}
}

bool SDCardImpl::process_block(const char* block) {

	bool finished = false;

	BlockIterator itr(block);

	check->set_current_header(itr, block_offset);

	if (check->finished()) {

		finished = true;
	}
	else if (check->datalength()) {

		check->mote_id();

		write_samples(itr);
		// Assumes that out is opened and time sync is reset by write_samples
		// FIXME Now reboot can be identified from the header.local_start
		// TODO Check if we are writing samples and time sync under the same file name
		write_time_sync_info();
	}

	return finished;
}

SDCardImpl::~SDCardImpl() {
	// Do NOT remove this empty dtor: required to generate the dtor of auto_ptr
}

}
