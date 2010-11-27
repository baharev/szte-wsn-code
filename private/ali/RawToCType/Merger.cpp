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
#include "Merger.hpp"
#include "TimeSyncInfo.hpp"

using namespace std;

namespace sdc {

typedef List::iterator li;

typedef List::size_type Size_t;

Merger::Merger(const VirtualMoteID& vmote_1, const List& msg_mote1)
	: vmote1(vmote_1), mote1(msg_mote1)
{

	drop_inconsistent(mote1);

	mote2_id_new = false;
}

void Merger::drop_inconsistent(List& messages) {

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

void Merger::init_for_mote2() {

	merged.clear();

	vmote2.reset();

	li beg = mote1.begin();

	if (beg!=mote1.end()) {

		vmote2 = *beg;

		mote2_id_new = true;

		cout << "Found: " << vmote2 << endl;
	}
	else {

		mote2_id_new = false;
	}
}

bool Merger::set_next() {

	init_for_mote2();

	for (li i = mote1.begin(); i != mote1.end(); ) {

		if (vmote2 == *i) {

			merged.push_back(*i);

			i = mote1.erase(i);
		}
		else {

			++i;
		}
	}

	return !merged.empty();
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
}

void Merger::drop_not_from_mote1() {

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

}
