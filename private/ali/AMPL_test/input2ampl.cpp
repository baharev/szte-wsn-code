/* Copyright (c) 2011, University of Szeged
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

#include <sstream>
#include "input2ampl.hpp"
#include "DataIO.hpp"

using namespace gyro;

input2ampl::input2ampl(const char* filename) : ptr_out(new ofstream(filename)), out(*ptr_out) {

	out.exceptions(ios_base::failbit | ios_base::badbit );

	copy_header();

	data_header();

	append_data();

	closing_lines();
}

void input2ampl::copy_header() {

	ifstream in;

	in.exceptions(ios_base::failbit | ios_base::badbit | ios_base::eofbit);

	in.open("gyro_data_header.txt");

	string buffer;

	while (buffer!="# SAMPLES") {

		getline(in, buffer);

		out << buffer << '\n';
	}
}

void input2ampl::data_header() {

	out << "\n\nparam samples :\n";
    out << "#\ttime\taccel_x\taccel_y\taccel_z\tgyro_x\tgyro_y\tgyro_z\n";
}

void input2ampl::append_data() {

	line = 1;

	ifstream in;

	in.exceptions(ios_base::failbit | ios_base::badbit | ios_base::eofbit);

	in.open("record.csv");

	string buffer;

	getline(in, buffer);

	getline(in, buffer);

	while (buffer.at(0)!='#') {

		convert_line(buffer);

		getline(in, buffer);
	}
}

void input2ampl::convert_line(const string& buffer) {

	istringstream in(buffer);

	in.exceptions(ios_base::failbit | ios_base::badbit | ios_base::eofbit);

	int timestamp, accel[3], gyro[3];

	char dummy;

	in >> timestamp >> dummy;

	for (int i=0; i<3; ++i) {

		in >> accel[i] >> dummy;
	}

	for (int i=0; i<3; ++i) {

		in >> gyro[i] >> dummy;
	}

	write_line(timestamp, accel, gyro);
}

void input2ampl::write_line(int timestamp, const int accel[3], const int gyro[3]) {

	out << line;

	for (int i=0; i<3; ++i) {

		out << '\t' << accel[i];
	}

	for (int i=0; i<3; ++i) {

		out << '\t' << gyro[i];
	}

	out << '\n';

	++line;
}

void input2ampl::closing_lines() {

	out << ";\n\n" << "param N := " << line-1 << ";\n\n";

	out.close();
}
