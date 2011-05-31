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

#ifndef VARESTIMATES_HPP_
#define VARESTIMATES_HPP_

#include "VarEnum.hpp"

namespace gyro {

class VarEstimates {

public:

	VarEstimates();

	const double* lower_bounds()  const { return x_L; }
	const double* upper_bounds()  const { return x_U; }
	const double* initial_point() const { return x_0; }

	const double* lower_bounds(VarEnum i)  const { return x_L+i; }
	const double* upper_bounds(VarEnum i)  const { return x_U+i; }
	const double* initial_point(VarEnum i) const { return x_0+i; }

private:

	void set_everything_incorrect();

	void set_intial_points();

	void copy_to_initial_point(const double array[], const VarEnum first, const VarEnum last_inclusive);

	void set_bounds();

	void set_bounds_by_abs_inflation(const VarEnum first, const VarEnum last_inclusive, double amount);

	void check_feasibility();

	double x_L[N_VARS];
	double x_U[N_VARS];
	double x_0[N_VARS];
};

}

#endif // VARESTIMATES_HPP_
