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

#include <stdexcept>
#include "TimeSyncMerger.hpp"
#include "TimeSyncInfo.hpp"
#include "DataReader.hpp"
#include "FlatFileDB.hpp"
#include "Merger.hpp"
#include "VirtualMoteID.hpp"

using namespace std;

namespace sdc {

TimeSyncMerger::TimeSyncMerger(int mote, int reboot, int first_block)
	: mote1(mote), block1(first_block)
{
	mote2  = -1;
	block2 = -1;

	if (mote1 <= 0) {
		throw logic_error("mote ID must be a positive integer");
	}

	if (reboot <= 0) {
		throw logic_error("reboot ID must be a positive integer");
	}

	if (block1 < 0) {
		throw logic_error("index of the first block cannot be negative");
	}

	DataReader reader1(mote1, reboot, block1);

	reader1.read_messages_from_file();

	VirtualMoteID vmote1(mote1, block1);

	merger.reset(new Merger(vmote1, reader1.messages_as_list()));
}

void TimeSyncMerger::reset_db_if_needed() {

	if (merger->mote2_id_changed()) {

		int mote2_id = merger->mote2_id();

		db.reset(new FlatFileDB(mote2_id));
	}
}

void TimeSyncMerger::process_pairs() {

	while(merger->set_next()) {

		reset_db_if_needed();

		int mote2_id = merger->mote2_id();

		int first_block2 = merger->block2();

		int reboot2 = db->reboot(first_block2);

		DataReader reader2(mote2_id, reboot2, first_block2);

		reader2.read_messages_from_file();

		merger->set_mote2_messages(reader2.messages_as_list());

		merger->merge();

		// TODO Dump results
	}
}

TimeSyncMerger::~TimeSyncMerger() {
	// Do NOT remove this empty dtor: required to generate the dtor of auto_ptr
}

}
