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
#include <stdexcept>
#include "BlockRelatedConsts.hpp"
#include "FileAsBlockDevice.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

FileAsBlockDevice::FileAsBlockDevice(const char* source)
	: in(new ifstream(source, ios::binary))
{
	if (!in->good()) {
		string msg("failed to open file ");
		msg += source;
		msg += ", " + last_error();
		throw runtime_error(msg);
	}

	in->exceptions(ifstream::failbit | ifstream::badbit | ifstream::eofbit);

	in->seekg(0, ios::end);

	card_size = in->tellg();

	BLOCK_OFFSET_MAX = card_size/BLOCK_SIZE;
}

const char* FileAsBlockDevice::read_block(uint64_t i) {

	check_index(i);

	try {

		in->seekg(i*BLOCK_SIZE);

		in->read(buffer, BLOCK_SIZE);
	}
	catch (ios_base::failure& ) {

		throw runtime_error(failed_to_read_block(i));
	}

	return buffer;
}

FileAsBlockDevice::~FileAsBlockDevice() {
	// Do NOT remove this empty dtor: required to generate the dtor of auto_ptr
}

}


