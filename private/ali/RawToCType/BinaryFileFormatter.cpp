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

#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <stdint.h>
#include "BinaryFileFormatter.hpp"
#include "BlockRelatedConsts.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

BinaryFileFormatter::BinaryFileFormatter(const char* source)
: out(new fstream(source, ios_base::in | ios_base::out | ios_base::binary)),
  buffer(new char[BLOCK_SIZE])
{
	if (!out->good()) {
		string msg("failed to open file ");
		msg += source;
		throw runtime_error(msg);
	}

	out->exceptions(ios_base::failbit | ios_base::badbit);

	out->seekg(0, ios_base::end);

	int64_t size_in_bytes = static_cast<int64_t> (out->tellg());

	if (size_in_bytes >= numeric_limits<int32_t>::max() || size_in_bytes < 0) {

		throw runtime_error("card size is larger than 2GB");
	}

	int32_t size = static_cast<int32_t> (size_in_bytes);

	BLOCK_OFFSET_MAX = size/BLOCK_SIZE; // TODO Is it safe?

	memset(buffer.get(), '\0', BLOCK_SIZE);
}

void BinaryFileFormatter::format() {

	for (int i=0; i<=BLOCK_OFFSET_MAX; ++i) {

		write_block(i);
	}

	cout << "Successfully formatted " << BLOCK_OFFSET_MAX << " blocks, ";

	cout << BLOCK_OFFSET_MAX*BLOCK_SIZE << " bytes" << endl;
}

void BinaryFileFormatter::write_block(int i) {

	if (i<0 || i>BLOCK_OFFSET_MAX) {
		throw out_of_range("block index");
	}

	try {

		out->seekg(i*BLOCK_SIZE);

		out->write(buffer.get(), BLOCK_SIZE);
	}
	catch (ios_base::failure& ) {

		throw runtime_error("failed to write block "+int2str(i));
	}
}

BinaryFileFormatter::~BinaryFileFormatter() {
	// Do NOT remove this empty dtor: required to generate the dtor of auto_ptr
}

}
