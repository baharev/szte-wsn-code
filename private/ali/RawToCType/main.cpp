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

#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include "BinaryFileFormatter.hpp"
#include "SDCard.hpp"

using namespace std;
using namespace sdc;

void download_new_records(const char* source) {

	auto_ptr<SDCard> bd(SDCard::new_instance(source));

	bd->process_new_measurements();
}

void format(const string& flag, const char* device) {

	if (flag != "-format") {

		throw runtime_error("unrecognized flag " + flag);
	}

	DeviceFormatter* df = new BinaryFileFormatter(device);

	df->format();
}

void print_usage(const char* program_name) {

	cout << endl;

	cout << "USAGE" << endl;

	cout << "=====" << endl;

	cout << "Type the following to download the new records:" << endl;

	cout << program_name << " path_to_device" << endl << endl;

	cout << "To format device type this:" << endl;

	cout << program_name << " -format path_to_device" << endl << endl;

#ifdef _WIN32
	cout << "The device path is the letter of the drive followed by a colon, like F:" << endl;
#endif
}

void real_main(int argc, char* argv[]) {

	if (argc == 2) {

		const char* device = argv[1];

		download_new_records(device);
	}
	else if (argc == 3) {

		const char* flag   = argv[1];

		const char* device = argv[2];

		format(flag, device);
	}
	else {

		print_usage(argv[0]);
	}
}

int main(int argc, char* argv[]) {

	cout << "Compiled on " << __DATE__ << ", " << __TIME__ << endl;

	cout << "This program comes with absolutely no warranty!" << endl;

	enum { SUCCESS, FAILURE };

	try {

		real_main(argc, argv);
	}
	catch (exception& e) {

		cout << "Error: " << e.what() << " (" << typeid(e).name() << ")" << endl;

		return FAILURE;
	}

	return SUCCESS;
}
