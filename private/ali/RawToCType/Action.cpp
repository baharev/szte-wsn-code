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

#include <algorithm>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <sstream>
#include "Action.hpp"
#include "DeviceFormatter.hpp"
#include "SDCard.hpp"
#include "Copy.hpp"

using namespace std;

namespace sdc {

template <typename T>
T to(const string& s) {

	istringstream is(s);

	is.exceptions(ios_base::failbit|ios_base::badbit);

	T temp;

	is >> temp;

	return temp;
}

class download : public Action {

public:

	download(const string& name) : Action(name) { }

private:

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

public:

	format(const string& name) : Action(name) { }

private:

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

class copy : public Action {

public:

	copy(const string& name) : Action(name) { }

private:

	virtual const string help_message() const {

		return "path_to_source  path_to_destination\n"
				"  to copy all binary data without checking,\n"
				"  both source and destination must exist";
	}

	virtual void parse_args(const vector<string>& args) {

		src  = args.at(2);
		dest = args.at(3);
	}

	virtual void run() {

		Copy cp(src, dest);

		cp.copy();
	}

	string src;
	string dest;
};

class rescue : public Action {

public:

	rescue(const string& name) : Action(name) { }

private:

	virtual const string help_message() const {

		return "path_to_device  backup_file\n"
				"  to copy all binary data from the device without checking,\n"
				"  back_up_file should NOT already exist";
	}

	virtual void parse_args(const vector<string>& args) {

		src  = args.at(2);
		dest = args.at(3);
	}

	virtual void run() {

		bool open_existing_dest = false;

		Copy cp(src, dest, open_existing_dest);

		cp.copy();
	}

	string src;
	string dest;
};

class rescue_partial : public Action {

public:

	rescue_partial(const string& name) : Action(name) { }

private:

	virtual const string help_message() const {

		return "path_to_device  backup_file  start_at  count\n"
				"  to copy the binary data from the device without checking,\n"
				"  back_up_file should NOT already exist\n"
				"  start copying at start_at block and copy at most count blocks";
	}

	virtual void parse_args(const vector<string>& args) {

		src  = args.at(2);
		dest = args.at(3);
		start_at    = to<uint64_t>(args.at(4));
		block_limit = to<uint64_t>(args.at(5));
	}

	virtual void run() {

		bool open_existing_dest = false;

		Copy cp(src, dest, open_existing_dest);

		cp.copy(start_at, block_limit);
	}

	string src;
	string dest;
	uint64_t start_at;
	uint64_t block_limit;
};

void Action::run(const std::vector<std::string>& args) {

	try {

		parse_args(args);
	}
	catch (exception& ) {

		cout << "Error: parsing command line arguments!\n";
		cout << "Try  " << usage(args.at(0)) << endl;
		cout << endl;

		return;
	}

	run();
}

const string Action::usage(const string& prog_name) const {

	return prog_name + "  " + flag() + "  " + help_message();
}

struct MapValueDelete {
	void operator()(const OptionMap::value_type& e) const { delete e; }
};

MapGuard::~MapGuard() {

	for_each(map.begin(), map.end(), MapValueDelete());
}

struct Cmp {
	string op;
	Cmp(const string& op) : op(op) { }
	bool operator()(const OptionMap::value_type& e) { return e->flag() == op; }
};

Action* MapGuard::find(const string& option) {

	OptionMap::const_iterator pos( std::find_if(map.begin(), map.end(), Cmp(option) ));

	return pos!=map.end() ? *pos : 0;
}

struct MapEntryPrinter {

	string prog_name;

	MapEntryPrinter(const string& name) : prog_name(name) { }

	void operator()(const OptionMap::value_type& e) const {

		const Action& a = *e;

		cout << prog_name << " " << a.flag() << " " << a.help_message() << endl << endl;
	}
};

void MapGuard::show_all(const string& program_name) {

	cout << "Usage:\n" << endl;

	for_each(map.begin(), map.end(), MapEntryPrinter(program_name));

#ifdef _WIN32
	cout << "The device path is the letter of the drive followed by a colon, like F:" << endl;
#endif
}

const MapGuard MapGuard::all_options() {

	OptionMap m;

#define ADD(arg) { m.push_back(new arg(#arg)); }

	ADD( download);
	ADD( format  );
	ADD( rescue  );
	ADD( rescue_partial );
	ADD( copy    );

	return m;
}

}
