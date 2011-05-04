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

#ifndef ELLIPSOIDNLP_HPP_
#define ELLIPSOIDNLP_HPP_

#include "CalibrationType.hpp"

#ifdef USE_GRADTYPE
#include "EllipsoidNLP_GradType.hpp"
#elif defined USE_ADOLC
#include "EllipsoidNLP_ADOLC.hpp"
#else
#error Must define either USE_GRADTYPE or USE_ADOLC! (GRADTYPE is self-contained)
#endif

namespace gyro {

class VarEstimates {

public:

	enum { A11, A12, A13, A22, A23, A33, B1, B2, B3 };

	const double* lower_bounds()  const { return x_L; }
	const double* upper_bounds()  const { return x_U; }
	const double* initial_point() const { return x_0; }

	virtual ~VarEstimates() = 0;

protected:

	VarEstimates();

	double x_L[N_VARS];
	double x_U[N_VARS];
	double x_0[N_VARS];
};

class AccelVarEstimates : public VarEstimates {

public:

	AccelVarEstimates();
};


class MagnetoVarEstimates : public VarEstimates {

public:

	MagnetoVarEstimates();
};

}

#endif // ELLIPSOIDNLP_HPP_
