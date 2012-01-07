/** Copyright (c) 2010, 2011, 2012 University of Szeged
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

#ifndef ACTION_HPP_
#define ACTION_HPP_

#include <string>
#include <vector>

namespace sdc {

class Action;

typedef std::vector<Action*> OptionMap;

class MapGuard {

public:

	static const MapGuard all_options();

	void show_all(const std::string& progname);

	Action* find(const std::string& option);

	~MapGuard();

private:

	MapGuard(const OptionMap& m) : map(m) { }

	const OptionMap map;
};

class Action {

public:

	void run(const std::vector<std::string>& args);

	const std::string usage(const std::string& prog_name) const;

	virtual const std::string help_message() const = 0;

	const std::string& flag() const { return name; }

	virtual ~Action() { };

protected:

	virtual void parse_args(const std::vector<std::string>& args) = 0;

	virtual void run() = 0;

	Action(const std::string& cmd_name) : name(cmd_name) { }

private:

	Action(const Action& );

	Action& operator=(const Action& );

	const std::string name;
};

}

#endif
