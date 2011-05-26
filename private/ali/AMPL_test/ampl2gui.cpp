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

#include <iomanip>
#include <sstream>
#include "ampl2gui.hpp"
#include "CompileTimeConstants.hpp"

using namespace gyro;

ampl2gui::ampl2gui(const char* filename) : ptr_out(new ofstream(filename)), out(*ptr_out) {

	out.exceptions(ios_base::failbit | ios_base::badbit );

	append_rotation_matrices();
}

void ampl2gui::append_rotation_matrices() {

	ifstream in;

	in.exceptions(ios_base::failbit | ios_base::badbit | ios_base::eofbit);

	in.open("gyro.log");

	write_n_samples(in);

	extract_matrices(in);
}

void ampl2gui::write_n_samples(ifstream& in) {

	string buffer;

	while (buffer!="Number of samples") {

		getline(in, buffer);
	}

	in >> N;

	out << NUMBER_OF_SAMPLES << '\n';

	out << N << '\n';

	getline(in, buffer);

	getline(in, buffer);
}

void ampl2gui::extract_matrices(ifstream& in) {

	out << ROTATION_MATRICES << '\n';

	out << setprecision(16) << scientific;

	for (int i=1; i<=9*N; ++i) {

		double temp;

		in >> temp;

		out << temp << '\n';
	}

	out << END_OF_FILE << '\n';

	out << flush;
}
