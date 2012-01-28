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

#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include "demangle.hpp"
#include "record_cut.hpp"

using namespace std;

namespace {

enum ARGS {
	PROGNAME,
	FILENAME,
	BEGIN,
	END,
	OFFSET,
};

}

void print_usage(const string& progname) {

	cout << "Usage: " << progname << "  record_to_cut  begin  end  ";
	cout << "wall_clock_time (optional, defaults to 00:00:00)" << endl;
    cout << "Timestamp format: hh:mm:ss.sss or mm:ss.sss" << endl;
	cout << "A timestamp with L or l prefix means time length, ";
	cout << "useful when specifiying relative time" << endl;
	cout << "Synonyms: beg, begin, end" << endl;
}

void dispatch(const vector<string>& args) {

	const int n_args = args.size();

	if (n_args<4 || n_args>5) {

		print_usage(args.at(PROGNAME));

		throw runtime_error("too few or too many command line arguments");
	}

	const string fname = args.at(FILENAME);

	const string begin = args.at(BEGIN);

	const string end   = args.at(END);

	const string offset= (n_args==5) ? args.at(OFFSET) : "00:00:00.000";

	sdc::record_cut rec(fname);

	rec.cut(begin, end, offset);
}

int main(int argc, char* argv[]) {

	cout << "This program comes with absolutely no warranty!" << endl;

	cout << "Compiled on " << __DATE__ << ", " << __TIME__ << endl;

	enum { SUCCESS, FAILURE };

	try {

		dispatch(vector<string> (argv, argv+argc));

	}
	catch (exception& e) {

		cout << "\nError: " << e.what() << " (" << sdc::name(e) << ")" << endl;

		return FAILURE;
	}

	return SUCCESS;
}

