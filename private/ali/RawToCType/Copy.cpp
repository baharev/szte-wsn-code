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

#include <iostream>
#include <iomanip>
#include "Copy.hpp"
#include "BlockDevice.hpp"
#include "DeviceFormatter.hpp"
#include "BlockRelatedConsts.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

Copy::Copy(const string& source) {

	in.reset(BlockDevice::zero_device());

	bool open_existing = true;

	out.reset(DeviceFormatter::new_instance(source.c_str(), open_existing));
}

Copy::Copy(const string& source, const string& destination, bool nocreate_destination) {

	in.reset(BlockDevice::new_instance(source.c_str()));

	out.reset(DeviceFormatter::new_instance(destination.c_str(), nocreate_destination));
}

void Copy::copy(const uint64_t start_at_block, const uint64_t block_limit) {

	const uint64_t bytes  = std::min(in->size_in_bytes(), out->size_in_bytes());

	const uint64_t blocks = std::min(bytes/BLOCK_SIZE-start_at_block,block_limit);

	const uint64_t bytes_to_copy = blocks*BLOCK_SIZE;

	cout << "Copying " <<  card_size_GB(bytes_to_copy) << endl;

	cout << "Started, please be patient, it will take a while..." << endl;

	for (uint64_t i=start_at_block; i<start_at_block+blocks; ++i) {

		const char* buffer = in->read_block(i);

		out->write_block(i-start_at_block, buffer);

		show_progress(i-start_at_block, blocks);
	}

	out->flush_to_device();

	cout << "\nSuccessfully written " << blocks << " blocks, ";

	cout << bytes_to_copy << " bytes" << endl;
}

void Copy::show_progress(uint64_t i, uint64_t blocks) const {

	++i;

	if (!(i%500)) {
		cout << "\rprogress made so far: written " << i << " blocks (approx. ";
		cout << setprecision(2) << fixed << (((double)i)/blocks*100.0);
		cout << "% ready)" << flush;
	}
}

Copy::~Copy() {

}

}
