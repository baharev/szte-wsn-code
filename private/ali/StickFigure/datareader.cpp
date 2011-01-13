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
* Author: Ali Baharev
*/

#include <fstream>
#include <stdexcept>
#include "datareader.hpp"

using namespace std;

datareader::datareader() {

    rotation_matrices = 0;
    size = counter = 0;
}

datareader::~datareader() {

    delete rotation_matrices;
}

void datareader::grab_content(const char *filename) {

    ifstream in;

    in.exceptions(ifstream::failbit | ifstream::badbit | ifstream::eofbit);

    in.open(filename);

    in >> size;

    const int n_elem = 9*size;

    rotation_matrices = new double[n_elem];

    for (int i=0; i<n_elem; ++i) {

        in >> rotation_matrices[i];
    }
}

const double* datareader::matrix_at(int i) const {

    if (i<0 || i>=size) {
        throw range_error("Index is out of range in datareader::matrix_at()");
    }

    return rotation_matrices + (9*i);
}

const double* datareader::next_matrix() {

    const double* const m = matrix_at(counter%size);

    ++counter;

    return m;
}
