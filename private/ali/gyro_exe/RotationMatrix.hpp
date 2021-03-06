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

#ifndef ROTATIONMATRIX_HPP_
#define ROTATIONMATRIX_HPP_

#include <iosfwd>

namespace gyro {

class Input;

class RotationMatrix {

public:

	RotationMatrix(	const Input& data,
					const double* const x,
					std::ostream& log = std::cout,
					bool verbose = false);

	double at(const int measurement, const int i, const int j) const;

	void dump_matrices(std::ostream& log = std::cout) const;

	void dump_g_err(std::ostream& log = std::cout) const;

	const double* matrices() const { return R; }

	~RotationMatrix();

private:

	RotationMatrix(const RotationMatrix& );

	RotationMatrix& operator=(const RotationMatrix& );

	void compute_M(	const double ax,
					const double ay,
					const double az,
					const Input& data);

	void orthogonality_accept() const;

	void objective_accept(double sx, double sy, double sz) const;

	void dump_angles(const Input& data, std::ostream& log = std::cout) const;

	void dump_path(const Input& data, std::ostream& log);

	const double* matrix_at(int i) const;

	double* const R;
	double* const g_err;
	const int N;

};

}

#endif

