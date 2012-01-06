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

#include <memory>
#include <stdexcept>
#include "Action.hpp"
#include "DeviceFormatter.hpp"
#include "SDCard.hpp"

using namespace std;

namespace sdc {

class download : public Action {

	virtual const string help_message() const {

		return "path_to_file_or_device\n"
				"  to download the latest records";
	}

	virtual void parse_args(const vector<string>& args) {

		src  = args.at(2);
	}

	virtual void run() {

		auto_ptr<SDCard> bd(SDCard::new_instance(src.c_str()));

		bd->process_new_measurements();
	}

	string src;
};

class format : public Action {

	virtual const string help_message() const {

		return "path_to_file_or_device\n"
				"  to format the device before the first usage";
	}

	virtual void parse_args(const vector<string>& args) {

		dev = args.at(2);
	}

	virtual void run() {

		auto_ptr<DeviceFormatter> df(DeviceFormatter::new_instance(dev.c_str()));

		df->format();
	}

	string dev;
};

void Action::run(const std::vector<std::string>& args) {

	try {

		parse_args(args);
	}
	catch (out_of_range& ) {
		// TODO Print usage of the particular flag here? Would be good for -help
		throw runtime_error("parsing command line arguments");
	}

	run();
}

const Action::Map Action::available_actions() {

	Map m;

	m.insert( make_pair("download", new download) );
	m.insert( make_pair("format",   new format  ) );


	return m;
}

}
