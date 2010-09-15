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

#include <iostream>
#include "QByteArray"
#include "Data.hpp"
#include "DataReader.hpp"
#include "DataReadException.hpp"
#include "CompileTimeConstants.hpp"
#include "Results.hpp"

using namespace std;

namespace ipo {

DataReader::DataReader(cli& begin, cli& end) : i(begin), end(end)
{

}

void DataReader::readAll(Results& r) {

    skip_irrelevant_lines();

    echo_line(gyro::FIRST_LINE);

    echo_line(gyro::CONFIG_FILE_ID);

    // TODO Copy error value
    echo_line(gyro::ERROR_IN_G);

    // TODO Check if matches the hard-coded value
    echo_line(gyro::NUMBER_OF_VARS);

    const int n_vars = gyro::NUMBER_OF_VARIABLES;

    // TODO Check acceptance level
    read_vector(gyro::SOLUTION_VECTOR, r.x, n_vars);

    read_vector(gyro::VARIABLE_LOWER_BOUNDS, r.x_lb, n_vars);

    read_vector(gyro::VARIABLE_UPPER_BOUNDS,  r.x_ub, n_vars);

    // TODO Check value
    echo_line(gyro::NUMBER_OF_SAMPLES);

    //=======================================================

    r.n = n_samples();

    const int n_elem = 9*(r.n);

    r.m = new double[n_elem];

    read_vector(gyro::ROTATION_MATRICES, r.m, n_elem);

    skip_line(gyro::END_OF_FILE);

}


void DataReader::read_vector(const char* text, double* r, int length) {

    skip_line(text);

    bool ok = false;

    for (int k=0; k<length; ++i, ++k) {

        if (i==end) {

            throw DataReadException("unexpected end of output");
        }

        r[k] = i->toDouble(&ok);

        if (!ok) {

            throw DataReadException("conversion error");
        }
    }

}

void DataReader::skip_irrelevant_lines() {

    int skipped = 0;

    while (i!=end) {

        if (!strcmp(i->constData(), gyro::FIRST_LINE)) {
            return;
        }

        ++i;
        ++skipped;
    }

    throw DataReadException("failed to find the first line");
}

void DataReader::skip_line(const char* text) {

    if (i==end) {
        string msg("unexpected end of output when checking ");
        msg += text;
        throw DataReadException(msg);
    }

    const char* const line = i->constData();

    if (!strcmp(line, text)) {
        ++i;
        cout << text << endl;
    }
    else {
        string msg("expected explanatory comment: ");
        msg += text;
        msg += ", found: ";
        msg +=  line;
        throw DataReadException(msg);
    }
}

void DataReader::echo_line(const char* text) {

    skip_line(text);

    if (i==end) {

        throw DataReadException("unexpected end of output");
    }

    cout << i->constData() << endl;

    ++i;
}

}
