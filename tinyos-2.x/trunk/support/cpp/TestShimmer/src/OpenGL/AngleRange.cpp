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

#include <iomanip>
#include <sstream>
#include "AngleRange.hpp"

void AngleRange::next(double x) {

    end(x);

    if (x < min()) {

        min(x);
    }
    else if (x > max()) {

        max(x);
    }
}

const AngleRange AngleRange::positive() const {

    AngleRange res;

    for (int i=0; i<4; ++i) {

        res.v[i] = (v[i]>=0) ? v[i] : 0.0;
    }

    return res;
}

const AngleRange AngleRange::negative() const {

    AngleRange res;

    for (int i=0; i<2; ++i) {

        res.v[i] = (v[i]<0) ? -v[i] : 0.0;
    }

    res.v[2] = (v[3]<0) ? -v[3] : 0.0;

    res.v[3] = (v[2]<0) ? -v[2] : 0.0;

    return res;
}

const std::string AngleRange::str(const char* name) const {

    std::ostringstream os;

    os << std::setprecision(1) << std::fixed;

    os << name << " " << beg() << " / " << end() << " / ";

    os << min() << " / " << max() << " / " << range();

    return os.str();
}

const std::string AngleRange::toCSV() const {

    std::ostringstream os;

    os << std::setprecision(1) << std::fixed;

    os << beg() << ";" << end() << ";" << min() << ";" << max();

    return os.str();
}
