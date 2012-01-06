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
#include <map>
#include "Dispatcher.hpp"
#include "Action.hpp"

using namespace std;

namespace sdc {

Dispatcher::Dispatcher(int argc, char* argv[]) : args(argv, argv+argc) {

}

struct MapValueDelete {
	template <typename T>
	void operator()(const T& p) const {
		delete p.second;
	}
};

struct RAII {
	const Action::Map map;
	RAII(const Action::Map& m) : map(m) { }
	~RAII() { for_each(map.begin(), map.end(), MapValueDelete()); }
};

struct MapEntryPrinter {

	string prog_name;

	MapEntryPrinter(const string& name) : prog_name(name) { }

	template <typename T>
	void operator()(const T& p) const {
		cout << prog_name << " -" << p.first << " " << p.second->help_message() << endl << endl;
	}
};

void Dispatcher::dispatch() {

	RAII all_actions(Action::available_actions());

	const Action::Map& ops = all_actions.map;

	if (need_help()) {

		cout << "Usage:\n" << endl;

		for_each(ops.begin(), ops.end(), MapEntryPrinter(args.at(0)));

		return;
	}

	Action::Map::const_iterator pos(ops.find(args.at(1).substr(1)));

	if (pos==ops.end()) {

		cout << "Error: flag " << args.at(1) << " not recognized\n";

		cout << "Usage:\n" << endl;

		for_each(ops.begin(), ops.end(), MapEntryPrinter(args.at(0)));
	}
	else {

		pos->second->run(args);
	}

}

bool Dispatcher::need_help() const {

	return (args.size()==1 || args.size()==2);
}

}

