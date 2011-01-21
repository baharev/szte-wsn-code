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

#include <algorithm>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include "datareader.hpp"

using namespace std;

datareader::datareader() {

    rotation_matrices = flexion = supination = deviation = 0;
    size = counter = 0;
}

datareader::~datareader() {

    delete[] rotation_matrices;

    delete[] flexion;
    delete[] supination;
    delete[] deviation;
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

    find_min_max();
}

void datareader::init_angle_arrays() {


    if (flexion || supination || deviation) {
        throw logic_error("already initialized");
    }

    flexion    = new double[size];
    supination = new double[size];
    deviation  = new double[size];
}

void datareader::fill_angle_arrays() {

    for (int i=0; i<size; ++i) {

        flexion[i]    = flexion_deg(i);
        supination[i] = supination_deg(i);
        deviation[i]  = deviation_deg(i);
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

void datareader::find_min_max() {

    init_angle_arrays();
    fill_angle_arrays();

    extrema[FLEX_MIN] = *min_element(flexion, flexion+size);
    extrema[FLEX_MAX] = *max_element(flexion, flexion+size);

    extrema[SUP_MIN] = *min_element(supination, supination+size);
    extrema[SUP_MAX] = *max_element(supination, supination+size);

    extrema[LAT_DEV_MIN] = *min_element(deviation, deviation+size);
    extrema[LAT_DEV_MAX] = *max_element(deviation, deviation+size);

    set_pronation();
    set_med_dev();
}

void datareader::set_pronation() {

    if (extrema[SUP_MAX] < 0) {

        extrema[PRON_MIN] = -extrema[SUP_MAX];
        extrema[PRON_MAX] = -extrema[SUP_MIN];

        extrema[SUP_MIN]  = extrema[SUP_MAX] = 0.0;
    }
    else if (extrema[SUP_MIN] < 0) {

        extrema[PRON_MAX] = -extrema[SUP_MIN];

        extrema[SUP_MIN] = extrema[PRON_MIN] = 0.0;
    }
    else {

        extrema[PRON_MIN] = extrema[PRON_MAX] = 0.0;
    }
}

void datareader::set_med_dev() {

    if (extrema[LAT_DEV_MAX] < 0) {

        extrema[MED_DEV_MIN] = -extrema[LAT_DEV_MAX];
        extrema[MED_DEV_MAX] = -extrema[LAT_DEV_MIN];

        extrema[LAT_DEV_MIN] = extrema[LAT_DEV_MAX] = 0.0;
    }
    else if (extrema[LAT_DEV_MIN] < 0) {

        extrema[MED_DEV_MAX] = -extrema[LAT_DEV_MIN];

        extrema[MED_DEV_MIN] = extrema[LAT_DEV_MIN] = 0.0;

    }
    else {

        extrema[MED_DEV_MIN] = extrema[MED_DEV_MAX] = 0.0;
    }
}

const double* datareader::get_extrema() const {

    return extrema;
}

// FIXME Duplication, same as in glwidget.cpp
namespace {

    enum {
            R11, R12, R13,
            R21, R22, R23,
            R31, R32, R33
    };

    const double RAD2DEG = 57.2957795131;
    const double PI_HALF = 1.57079632679;
}

// FIXME Duplication: also computed in glwidget.cpp
double datareader::flexion_deg(int i) const {

    const double* const m = matrix_at(i);

    return (atan2(m[R31], m[R21])+PI_HALF)*RAD2DEG;
}

double datareader::supination_deg(int i) const {

    const double* const m = matrix_at(i);

    return atan2(m[R12], m[R13])*RAD2DEG;
}

double datareader::deviation_deg(int i) const {

    const double* const m = matrix_at(i);

    return -(acos(m[R11])-PI_HALF)*RAD2DEG;  // Sign: make lateral-medial correct for right arm
}
