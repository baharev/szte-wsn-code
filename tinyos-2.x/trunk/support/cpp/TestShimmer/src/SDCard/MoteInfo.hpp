/* Copyright (c) 2010, University of Szeged
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
*      Author: Ali Baharev
*/

#ifndef MOTEINFO_HPP_
#define MOTEINFO_HPP_

#include <iosfwd>
#include <string>

namespace sdc {

class RecordID;

class MoteInfo {

public:

	MoteInfo();

	MoteInfo(int    mote,
			double card_size_in_blocks,
			int    last_block,
			const  std::string& last_download,
			int    number_of_records);

	explicit MoteInfo(const RecordID& rid);

	int mote_id() const;

	const std::string& last_download() const;

	const std::string& remaining_hours() const;

	int number_of_records() const;

private:

	int mote_ID;

	std::string hours_remaining;

	std::string last_seen;

	int num_of_records;
};

std::ostream& operator<<(std::ostream& , const MoteInfo& );

bool operator<(const MoteInfo& lhs, const MoteInfo& rhs);

bool id_equals(const MoteInfo& info1, const MoteInfo& info2);

}

#endif /* MOTEINFO_HPP_ */
