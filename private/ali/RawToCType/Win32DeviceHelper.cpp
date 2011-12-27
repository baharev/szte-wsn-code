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

#ifdef _WIN32

#include <stdexcept>
#include <limits>
#include "Win32DeviceHelper.hpp"
#include "WinBlockDevice.h"
#include "BlockRelatedConsts.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

const device_data open_device(const char* source) {

	const char drive_letter = string(source).at(0);
	wstring path(L"\\\\.\\");
	path += drive_letter;
	path += ':';

	device_data ret;

	int64_t size = card_size_in_bytes(path.c_str(), &ret.hDevice);

	if (size==0) {
		string msg("failed to open block device ");
		msg += source;
		throw runtime_error(msg);
	}

	int32_t intmax = (numeric_limits<int32_t>::max)(); // Otherwise error C2589

	if (size >= intmax || size < 0) {
		close_device(&ret.hDevice);
		throw runtime_error("card size is larger than 2GB");
	}

	ret.size = static_cast<int32_t>(size);

	return ret;
}

const char* read_device_block(PHANDLE pHandle, int i, char* buffer, const unsigned int BLOCK_SIZE) {

	return ::read_device_block(pHandle, i, buffer, BLOCK_SIZE);
}

void write_device_block(PHANDLE pHandle, int i, char* buffer, const unsigned int BLOCK_SIZE) {

	int error = ::write_device_block(pHandle, i, buffer, BLOCK_SIZE);

	if (error) {

		throw runtime_error("failed to write block "+int2str(i));
	}
}

void close_device(PHANDLE pHandle) {

	::close_device(pHandle);
}

unsigned long error_code() {

	return ::error_code();
}

}

#endif

