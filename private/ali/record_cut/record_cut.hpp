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
#ifndef RECORD_CUT_HPP_
#define RECORD_CUT_HPP_

#include <vector>
#include <string>
#include <stdint.h>

namespace sdc {

class record_cut {

public:

	record_cut(const std::string& file_name);

	uint32_t number_of_lines() const { return samples.size(); }

	double length_in_sec() const;

	const std::string length() const;

	void cut(const std::string& begin, const std::string& end) const;

	void cut(const std::string& begin, const std::string& end, const std::string& offset) const;

private:

	struct indices {
		int first;
		int last;
	};

	const indices to_indices(const std::string& begin, const std::string& end, const std::string& offset) const;

	const indices to_indices(double beg, double end) const;

	double get_begin(const std::string& begin, const std::string& offset) const;

	double get_end(const std::string& end, const std::string& offset) const;

	const std::string infile_name;

	std::vector<std::string> samples;
};

}

#endif
