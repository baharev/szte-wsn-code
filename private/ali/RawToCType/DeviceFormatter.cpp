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
#include <stdexcept>
#include "DeviceFormatter.hpp"
#include "BinaryFileFormatter.hpp"
#include "BlockRelatedConsts.hpp"
#include "Utility.hpp"
#include "Win32DriveFormatter.hpp"

using namespace std;

namespace sdc {

DeviceFormatter* DeviceFormatter::new_instance(const char* path) {

	DeviceFormatter* df = 0;

	if (is_drive(path)) {

		df = new Win32DriveFormatter(path);
	}
	else {

		df = new BinaryFileFormatter(path);
	}

	return df;
}

void DeviceFormatter::format() {

	cout << "Started, please be patient, it will take a while..." << endl;

	char buffer[BLOCK_SIZE] = { 0 };

	for (uint64_t i=0; i<BLOCK_OFFSET_MAX; ++i) {

		write_block(i, buffer);
	}

	flush_to_device();

	cout << "Successfully formatted " << BLOCK_OFFSET_MAX << " blocks, ";

	cout << BLOCK_OFFSET_MAX*BLOCK_SIZE << " bytes" << endl;
}

void DeviceFormatter::check_index(uint64_t i) const {

	if (i>=BLOCK_OFFSET_MAX) {
		throw out_of_range("block index "+uint2str(i));
	}
}

}
