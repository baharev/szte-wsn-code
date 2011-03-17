/* Copyright (c) 2011 University of Szeged
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

#include <fstream>
#include <sstream>
#include <iomanip>
#include <cmath>

using namespace std;

const string suffix() {

	static int suffix = 1;

	ostringstream os;

	os << '_' << suffix << flush;

	++suffix;

	return os.str();
}

void write_array(fstream& out, double offset, int n) {

	const string s = suffix();

	out << "pair z" << s << "[] = {\n" ;

	int i=1;

	for ( ; i<n; ++i) {

		out << "(\t" << i << ",\t" << sin(i+offset) << "),\n";
	}

	out << "(\t" << i << ",\t" << sin(i+offset) << ") };\n";

	out << "guide g" << s << " = graph(z" << s << ");\n";

	out << "pen p"<< s << " = gray(0.0);\n";
	out << "p" << s << " = p" << s << " + 1.5;\n";
	out << "draw(g" << s << ", p" << s << ");\n\n";


}

int main() {

	fstream out;

	out.open("graph", ios_base::out);
	out << setprecision(3) << fixed;

	out << "import graph;\n";
	out << "size(0, 250);\n";

	write_array(out, 0, 20);
	write_array(out, 1, 20);
	write_array(out, 2, 20);

	return 0;
}
