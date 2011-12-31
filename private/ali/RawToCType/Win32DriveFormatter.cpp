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

#include <stdexcept>
#include "Win32DriveFormatter.hpp"
#include "Win32DeviceHelper.hpp"
#include "BlockRelatedConsts.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

#ifdef _WIN32

Win32DriveFormatter::Win32DriveFormatter(const char* source) {

	hDevice = open_device(source, GENERIC_WRITE);

	BLOCK_OFFSET_MAX = device_size()/BLOCK_SIZE;
}

int32_t Win32DriveFormatter::device_size() {

	int64_t size64 = size_in_bytes(hDevice);

	return cast_to_int32(size64);
}

void Win32DriveFormatter::write_block(int i, const char* buffer) {

	check_index(i)

	sdc::write_block(hDevice, i, buffer, BLOCK_SIZE);
}


void Win32DriveFormatter::flush_to_device() {
	//  FILE_FLAG_NO_BUFFERING is used in CreateFile,
	//  otherwise call FlushFileBuffers(hDevice);
}

Win32DriveFormatter::~Win32DriveFormatter() {

	sdc::close_device(hDevice);
}

#else

Win32DriveFormatter::Win32DriveFormatter(const char* ) {

	throw logic_error("Win32 block device is not implemented!");
}

int32_t Win32DriveFormatter::device_size() { return 0; }

void Win32DriveFormatter::write_block(int , const char* ) { }

void Win32DriveFormatter::flush_to_device() { }

Win32DriveFormatter::~Win32DriveFormatter() { }

#endif

}
