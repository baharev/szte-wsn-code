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
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include "Merger.hpp"
#include "TimeSyncInfo.hpp"
#include "TimeSyncConsts.hpp"

using namespace std;

namespace sdc {

typedef List::iterator li;
typedef List::const_iterator cli;
typedef List::size_type Size_t;
typedef Map::iterator mi;
typedef Map::const_iterator cmi;

Merger::Merger(const VirtualMoteID& vmote_1, const List& msg_mote1)
	: vmote1(vmote_1), mote1(msg_mote1)
{
	drop_inconsistent(mote1);

	mote2_id_new = false;

	cout << "Initialized mote1 list" << endl;
}

void Merger::drop_inconsistent(List& messages) {

	cout << "Dropping inconsistent messages" << endl;

	Size_t size_before = messages.size();

	for (li i = messages.begin(); i != messages.end(); ) {

		if (!i->consistent()) {

			cout << "Warning: inconsistent message " << endl << *i << endl;

			i = messages.erase(i);
		}
		else {

			++i;
		}
	}

	cout << "Dropped " << size_before - messages.size() << " messages" << endl;
}

void Merger::log_msg_loss(const List& messages, const VirtualMoteID& vmid) const {

	if (messages.empty()) {

		return;
	}

	cout << "Checking message frequency on: " << vmid << endl;

	cli i = messages.begin();

	TimeSyncInfo previous = *i;

	++i;

	while (i != messages.end()) {

		const TimeSyncInfo current = *i;

		int lost = current.lost_messages_since(previous);

		if (lost) {

			cout << "Warning: missing " << lost << " messages" << endl;
		}

		previous = current;

		++i;
	}

	cout << "Message loss checked" << endl;
}

void Merger::init_for_mote2() {

	mote2.clear();
	temp.clear();
	merged.clear();

	VirtualMoteID vmote = *mote1.begin();

	mote2_id_new = vmote.mote_id()==vmote2.mote_id() ? false : true;

	vmote2 = vmote;

	cout << "-----------------------------------------------------" << endl;
	cout << "Found: " << vmote2 << endl;
}

bool Merger::set_next() {

	if (mote1.empty()) {

		return false;
	}

	init_for_mote2();

	for (li i = mote1.begin(); i != mote1.end(); ) {

		if (vmote2 == *i) {

			temp.push_back(*i);

			i = mote1.erase(i);
		}
		else {

			++i;
		}
	}

	cout << "Relevant messages from " << vmote1 << " copied" << endl;

	log_msg_loss(temp, vmote1);

	return !temp.empty();
}

bool Merger::mote2_id_changed() const {

	return mote2_id_new;
}

int Merger::mote2_id() const {

	return vmote2.mote_id();
}


int Merger::block2() const {

	return vmote2.first_block();
}

void Merger::set_mote2_messages(const List& messages_mote2) {

	mote2 = messages_mote2;

	drop_inconsistent(mote2);

	drop_not_from_mote1();

	log_msg_loss(mote2, vmote2);
}

void Merger::drop_not_from_mote1() {

	cout << "Discarding messages not from " << vmote1 << endl;

	Size_t size_before = mote2.size();

	for (li i = mote2.begin(); i != mote2.end(); ) {

		if (vmote1 != *i) {

			i = mote2.erase(i);
		}
		else {

			++i;
		}
	}

	cout << "Dropped " << size_before - mote2.size() << " messages" << endl;
}

void Merger::insert(const Pair& sync_point) {

	pair<Map::iterator, bool>  pos = merged.insert(sync_point);

	if (!pos.second) {

		cout << "Warning: conflicting keys in time pairs, ";
		cout << "dropping the first one shown below" << endl;
		cout << sync_point.first << '\t' << sync_point.second << endl;
		cout << pos.first->first << '\t' << pos.first->second << endl;
	}
}

int Merger::offset(const CPair& p) const {

	return p.first - p.second;
}

void Merger::two_offsets() {

	cmi i = merged.begin();

	int offset1 = offset(*i++);

	int offset2 = offset(*i);

	if (fabs(offset1-offset2) > OFFSET_TOLERANCE) {

		cout << "Warning: got only 2 time sync points and they ";
		cout << "differ too much, dropping both" << endl;

		merged.clear();
	}
}

int Merger::initial_offset() const {

	const int n = 3, middle = 1;

	int ofs[n];

	int i=0;

	for (cmi j = merged.begin(); i<n; ++i, ++j) {

		ofs[i] = offset(*j);
	}

	cout << "1st, 2nd, 3rd offset: ";
	cout << ofs[0] << '\t' << ofs[1] << '\t' << ofs[2] << endl;

	sort(ofs, ofs+n);

	return ofs[middle];
}

bool Merger::wrong_offset(const CPair& time_pair, int& previous_offset) const {

	bool wrong = false;

	int current_offset = offset(time_pair);

	int diff = current_offset-previous_offset;

	if (fabs(diff) > OFFSET_TOLERANCE) {

		cout << "Warning: wrong offset found, previous " << previous_offset;
		cout << ", current " << current_offset << ", diff " << diff << endl;

		wrong = true;
	}
	else {

		previous_offset = current_offset;
	}

	return wrong;
}

void Merger::drop_wrong_offsets() {

	int offset = initial_offset();

	int k = 1;

	for (mi i = merged.begin(); i!=merged.end(); ++k) {

		if (wrong_offset(*i, offset)) {

			cout << "Erasing wrong time pair " << i->first << "  ";
			cout << i->second << " (" << k << ")" << endl;

			merged.erase(i++);
		}
		else {
			++i;
		}
	}
}

void Merger::merge() {

	cout << "Merging time sync info" << endl;

	if (!merged.empty()) {

		throw logic_error("merged should have been cleared");
	}

	for (cli i = temp.begin(); i != temp.end(); ++i) {

		insert(i->time_pair());
	}

	for (cli i = mote2.begin(); i != mote2.end(); ++i) {

		insert(i->reversed_time_pair());
	}

	if (merged.size()>2) {

		drop_wrong_offsets();
	}
	else if (merged.size()==2){

		two_offsets();
	}

	if (merged.size() < 2) {

		cout << "Warning: time sync requires at least 2 sync points" << endl;
		merged.clear();
	}

	cout << "Merged, size: " << merged.size() << endl;
}

}
