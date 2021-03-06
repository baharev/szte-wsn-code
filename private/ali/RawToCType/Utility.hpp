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

#ifndef UTILITY_HPP_
#define UTILITY_HPP_

#include <string>
#include <stdint.h>

namespace sdc {

const std::string ticks2time(uint32_t t);

const std::string current_time();

const std::string get_filename(int mote_id, int reboot_id, int first_block);

const std::string time_to_filename(); // TODO Do we need this function?

const std::string recorded_length(int first_block, int last_block);

const std::string remaining_GB(double card_size_GB, int last_block);

const std::string remaining_hours(double card_size_GB, int last_block);

int round(double x);

int recorded_length_in_ms(int first_block, int last_block);

const std::string failed_to_read_block(uint64_t i);

const std::string rdb_file_name(int mote_id);

const std::string int2str(int i);

const std::string uint2str(uint64_t i);

uint32_t GB();

bool is_drive(const char* source);

double byte_to_GB(uint64_t size);

const std::string card_size_GB(uint64_t size);

int32_t cast_to_int32(uint64_t size); // throws if size is larger than 2GB

void replace(std::string& s, const char old, const char new_char);

const std::string last_error();

}

#endif
