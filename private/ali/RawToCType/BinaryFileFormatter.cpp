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
#include <limits>
#include <stdexcept>
#include "BinaryFileFormatter.hpp"
#include "BlockRelatedConsts.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

BinaryFileFormatter::BinaryFileFormatter(const char* source, bool open_existing) {

	if (open_existing) {

		open(source);
	}
	else {

		create(source);
	}
}

bool BinaryFileFormatter::open_existing_file(const char* source) {

	out.reset(new fstream(source, ios_base::in | ios_base::out | ios_base::binary));

	return out->good();
}

void BinaryFileFormatter::open(const char* source) {

	bool success = open_existing_file(source);

	if (!success) {
		string msg("failed to open file ");
		msg += source;
		msg += ", " + last_error();
		throw runtime_error(msg);
	}

	out->exceptions(ios_base::failbit | ios_base::badbit);

	out->seekp(0, ios_base::end);

	card_size = static_cast<uint64_t> (out->tellp());

	BLOCK_OFFSET_MAX = card_size/BLOCK_SIZE;
}

void BinaryFileFormatter::create(const char* source) {

	bool success = open_existing_file(source);

	if (success) {

		out.reset();

		throw runtime_error("destination file already exists");
	}

	out.reset(new fstream(source, ios_base::out));

	if (!out->good()) {
		string msg("failed to create file ");
		msg += source;
		msg += ", " + last_error();
		throw runtime_error(msg);
	}

	card_size = BLOCK_OFFSET_MAX = numeric_limits<uint64_t>::max();
}

void BinaryFileFormatter::write_block(uint64_t i, const char* buffer) {

	check_index(i);

	try {

		out->seekp(i*BLOCK_SIZE, ios_base::beg);

		out->write(buffer, BLOCK_SIZE);
	}
	catch (ios_base::failure& ) {

		throw runtime_error("failed to write block "+uint2str(i));
	}
}

void BinaryFileFormatter::flush_to_device() {

	out->flush();
}

BinaryFileFormatter::~BinaryFileFormatter() {
	// Do NOT remove: required to generate the dtor of auto_ptr
}

}
