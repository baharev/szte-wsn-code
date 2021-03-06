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

#ifndef BLOCKDEVICE_HPP_
#define BLOCKDEVICE_HPP_

#include <stdint.h>
#include "BlockRelatedConsts.hpp"

namespace sdc {

class BlockDevice {

public:

	static BlockDevice* new_instance(const char* binary_file_or_win32_drive);

	static BlockDevice* zero_device();

	// Pointer to the device's internal buffer, do NOT delete it
	virtual const char* read_block(uint64_t i) = 0;

	int32_t end_int32() const; // First invalid block index, throws if > 2GB

	uint64_t size_in_bytes() const { return card_size; }

	virtual ~BlockDevice() { }

protected:

	BlockDevice();

	void check_index(uint64_t i) const;

	char buffer[BLOCK_SIZE];

	uint64_t BLOCK_OFFSET_MAX;

	uint64_t card_size;

private:

	BlockDevice(const BlockDevice& );

	BlockDevice& operator=(const BlockDevice& );

};

}

#endif
