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

#include <fstream>
#include <stdexcept>
#include "BinaryFileFormatter.hpp"
#include "BlockRelatedConsts.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

BinaryFileFormatter::BinaryFileFormatter(const char* source)
: out(new fstream(source, ios_base::in | ios_base::out | ios_base::binary))
{
	if (!out->good()) {
		string msg("failed to open file ");
		msg += source;
		throw runtime_error(msg);
	}

	out->exceptions(ios_base::failbit | ios_base::badbit);

	BLOCK_OFFSET_MAX = device_size()/BLOCK_SIZE;
}

int32_t BinaryFileFormatter::device_size() {

	out->seekp(0, ios_base::end);

	int64_t size_in_bytes = static_cast<int64_t> (out->tellp());

	out->seekp(0, ios_base::beg);

	return cast_to_int32(size_in_bytes);
}

void BinaryFileFormatter::write_block(int i, const char* buffer) {

	check_index(i);

	try {

		out->seekp(i*BLOCK_SIZE, ios_base::beg);

		out->write(buffer, BLOCK_SIZE);
	}
	catch (ios_base::failure& ) {

		throw runtime_error("failed to write block "+int2str(i));
	}
}

void BinaryFileFormatter::flush_to_device() {

	out->flush();
}

BinaryFileFormatter::~BinaryFileFormatter() {
	// Do NOT remove: required to generate the dtor of auto_ptr
}

}
