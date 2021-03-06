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

#ifndef TRACKER_HPP_
#define TRACKER_HPP_

#include <iosfwd>
#include <memory>
#include <string>
#include <stdint.h>

namespace sdc {

class BlockIterator;

class Tracker {

public:

	explicit Tracker(BlockIterator& zeroth_block);

	int start_from_here() const;

	int reboot() const;

	int mote_id() const;

	void mark_beginning(int block_beg, int reboot);

	void append_to_db(int block_end, uint32_t time_len);

	~Tracker();

private:

	Tracker(const Tracker& );

	Tracker& operator=(const Tracker& );

	void set_filename(int mote_ID);

	void set_first_block_reboot_id();

	void find_last_line(std::ifstream& in);

	void process_last_line(const std::string& line);

	const std::auto_ptr<std::ofstream> db;

	std::string filename;

	int first_block;

	int reboot_id;

	int mote_ID;
};

}

#endif
