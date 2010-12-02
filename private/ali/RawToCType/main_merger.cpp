/* Copyright (c) 2010, University of Szeged
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

#include <iostream>
#include <typeinfo>
#include "FlatFileDB.hpp"
#include "TimeSyncMerger.hpp"
using namespace std;
using namespace sdc;

int main(int argc, char* argv[]) {

	enum { SUCCESS, FAILURE };

	FlatFileDB db5(5);

	for (int i=1; i<=db5.number_of_records(); ++i) {

		cout << "#############################################################";
		cout << endl;

		TimeSyncMerger tsm(5, i);

		tsm.pairs();
	}

	cout<<"============================================================="<<endl;

	FlatFileDB db4(4);

	for (int i=19; i<=db4.number_of_records(); ++i) {

		cout << "#############################################################";
		cout << endl;

		TimeSyncMerger tsm(4, i);

		tsm.pairs();
	}

/*
	if (argc != 3) {

		clog << "Error: !" << endl;

		return FAILURE;
	}


	try {


	}
	catch (exception& e) {

		clog << e.what() << " (" << typeid(e).name() << ")" << endl;

		return FAILURE;
	}
*/
	return SUCCESS;
}
