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
#include <iostream>
#include <stdexcept>
#include "Win32DriveFormatter.hpp"
#include "Win32DeviceHelper.hpp"
#include "BlockRelatedConsts.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

#ifdef _WIN32

Win32DriveFormatter::Win32DriveFormatter(const char* source)
: buffer(new char[BLOCK_SIZE])
{

	hDevice = open_device(source, GENERIC_WRITE);

	int64_t size64 = size_in_bytes(hDevice);

	int32_t size32 = cast_to_int32(size64);

	BLOCK_OFFSET_MAX = size32/BLOCK_SIZE;

	memset(buffer.get(), '\0', BLOCK_SIZE);
}

void Win32DriveFormatter::write_block(int i) {

	if (i<0 || i>BLOCK_OFFSET_MAX) {
		throw out_of_range("block index");
	}

	sdc::write_block(hDevice, i, buffer.get(), BLOCK_SIZE);
}

void Win32DriveFormatter::format() {

	for (int i=0; i<=BLOCK_OFFSET_MAX; ++i) {

		write_block(i);
	}

	cout << "Successfully formatted " << BLOCK_OFFSET_MAX << " blocks, ";

	cout << BLOCK_OFFSET_MAX*BLOCK_SIZE << " bytes" << endl;
}

Win32DriveFormatter::~Win32DriveFormatter() {

	sdc::close_device(hDevice);
}

#else

Win32DriveFormatter::Win32DriveFormatter(const char* ) {

	throw logic_error("Win32 block device is not implemented!");
}

void Win32DriveFormatter::write_block(int ) { }

void Win32DriveFormatter::format() { }

Win32DriveFormatter::~Win32DriveFormatter() { }

#endif

}
