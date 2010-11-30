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

#ifndef MERGER_HPP_
#define MERGER_HPP_

#include <list>
#include <map>
#include "VirtualMoteID.hpp"

namespace sdc {

class TimeSyncInfo;

typedef std::list<TimeSyncInfo> List;
typedef std::map<unsigned int, unsigned int> Map;
typedef std::pair<unsigned int, unsigned int> Pair;
typedef std::pair<const unsigned int, unsigned int> CPair;

class Merger {

public:

	explicit Merger(const VirtualMoteID& vmote_1, const List& messages_mote1);

	bool set_next();

	bool mote2_id_changed() const;

	int mote2_id() const;

	int block2() const;

	void set_mote2_messages(const List& messages_mote2);

	void merge();

private:

	Merger(const Merger& );
	Merger& operator=(Merger& );

	void insert(const Pair& sync_point);
	void log_msg_loss(const List& messages, const VirtualMoteID& vmid) const;
	void drop_inconsistent(List& messages);
	void drop_not_from_mote1();
	int offset(const CPair& p) const;
	bool wrong_offset(const CPair& time_pair, int& previous_offset) const;
	int initial_offset() const;
	void drop_wrong_offsets();
	void two_offsets();
	void init_for_mote2();

	const VirtualMoteID vmote1;

	List mote1;
	List mote2;
	List temp;
	Map merged;

	VirtualMoteID vmote2;
	bool mote2_id_new;
};

}

#endif /* MERGER_HPP_ */
