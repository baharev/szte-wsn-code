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
*      Author: Ali Baharev
*/

#include <iostream>
#include <iomanip>
#include <algorithm>
#include "Compare.hpp"
#include "BlockDevice.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

Compare::Compare(const string& src) {

	in1.reset(BlockDevice::new_instance(src.c_str()));

	in2.reset(BlockDevice::zero_device());
}

Compare::Compare(const string& src1, const string& src2) {

	in1.reset(BlockDevice::new_instance(src1.c_str()));

	in2.reset(BlockDevice::new_instance(src2.c_str()));
}

void Compare::compare(const uint64_t start_at_block, const uint64_t block_limit) {

	const uint64_t bytes  = std::min(in1->size_in_bytes(), in2->size_in_bytes());

	const uint64_t blocks = std::min(bytes/BLOCK_SIZE-start_at_block,block_limit);

	const uint64_t bytes_to_copy = blocks*BLOCK_SIZE;

	cout << "Comparing " <<  card_size_GB(bytes_to_copy) << endl;

	cout << "Started, please be patient, it will take a while..." << endl;

	for (uint64_t i=start_at_block; i<start_at_block+blocks; ++i) {

		bool mismatch = compare(in1->read_block(i), in2->read_block(i-start_at_block), i);

		if (mismatch) {

			return;
		}

		show_progress(i-start_at_block, blocks);
	}

	cout << "Everything is OK! Successfully compared " << blocks << " blocks, ";

	cout << bytes_to_copy << " bytes" << endl;

	// TODO Warn about size mismatch <--> Everthing is OK ?
}

bool Compare::compare(const char* buffer1, const char* buffer2, uint64_t i) const {

	const char* res = mismatch(buffer1, buffer1+BLOCK_SIZE, buffer2).first;

	bool differs = res!=buffer1+BLOCK_SIZE;

	if (differs) {
		cout << "Mismatch found in block " << i << ", ";
		cout << "offset " << res-buffer1 << ", exiting..." << endl;
	}

	return differs;
}

void Compare::show_progress(uint64_t i, uint64_t blocks) const {

	++i;

	if (!(i%500)) {
		cout << "progress made so far: compared " << i << " blocks (approx. ";
		cout << setprecision(2) << fixed << (((double)i)/blocks*100.0);
		cout << "% ready)" << endl;
	}
}

Compare::~Compare() {

}

}
