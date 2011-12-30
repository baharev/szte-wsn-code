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

#ifdef _WIN32

#include <sstream>
#include <stdexcept>
#include <limits>
#include <windows.h>
#include <winioctl.h>
#include "Win32DeviceHelper.hpp"
#include "BlockRelatedConsts.hpp"
#include "Utility.hpp"

using namespace std;

namespace sdc {

const string error_message() {

    LPVOID lpMsgBuf;
    DWORD dw = GetLastError();

    FormatMessage(
        FORMAT_MESSAGE_ALLOCATE_BUFFER |
        FORMAT_MESSAGE_FROM_SYSTEM |
        FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        dw,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPTSTR) &lpMsgBuf,
        0,
        NULL );

    ostringstream os;

    os << static_cast<char*>(lpMsgBuf) << " (code: " << dw << ")";

    LocalFree(lpMsgBuf);

    string msg(os.str());

    replace(msg, '\n', ' ');

    replace(msg, '\r', ' ');

    return msg;
}


HANDLE open_device(const char* path, DWORD access) {

	HANDLE hDevice;

	hDevice = CreateFileA(path,  // drive to open
		access,                // access to the drive
		FILE_SHARE_READ | // share mode
		FILE_SHARE_WRITE,
		NULL,             // default security attributes
		OPEN_EXISTING,    // disposition
		FILE_FLAG_NO_BUFFERING,                // file attributes
		NULL);            // do not copy file attributes

	if (hDevice == INVALID_HANDLE_VALUE) // cannot open the drive
	{
		const string msg = error_message();

		throw runtime_error(msg);
	}

	return hDevice;
}

int64_t size_in_bytes(HANDLE hDevice) {

	DISK_GEOMETRY pdg;
	BOOL bResult;
	DWORD junk;
	int64_t ret_val = 0;

	bResult = DeviceIoControl(hDevice,  // device to be queried
		IOCTL_DISK_GET_DRIVE_GEOMETRY,  // operation to perform
		NULL, 0, // no input buffer
		&pdg, sizeof(pdg),     // output buffer
		&junk,                 // # bytes returned
		(LPOVERLAPPED) NULL);  // synchronous I/O

	if (FALSE == bResult) {

		const string msg = error_message();

		throw runtime_error(msg);
	}

	ret_val = ((int64_t) pdg.Cylinders.QuadPart) * (int64_t)pdg.TracksPerCylinder *
		(int64_t)pdg.SectorsPerTrack * (int64_t)pdg.BytesPerSector;

	return ret_val;
}

void write_block(HANDLE hDevice, int i, char* buffer, const unsigned int BLOCK_SIZE) {

	DWORD  dwBytesWritten = 0;
	BOOL success = FALSE;

	OVERLAPPED ol;

	ol.hEvent = 0;
	ol.Internal = 0;
	ol.InternalHigh = 0;
	ol.Offset = (DWORD) BLOCK_SIZE*i;
	ol.OffsetHigh = 0;

	success = WriteFile(hDevice, buffer, BLOCK_SIZE, &dwBytesWritten, (LPOVERLAPPED) &ol);

	if ((success==FALSE)||(dwBytesWritten!=BLOCK_SIZE)) {

		string msg("writing block "+int2str(i)+" failed, "+error_message());

		throw runtime_error(msg);
	}
}

const char* read_block(HANDLE hDevice, int i, char* buffer, const unsigned int BLOCK_SIZE) {

	DWORD  dwBytesRead = 0;
	BOOL success = FALSE;
	OVERLAPPED ol;

	ol.hEvent = 0;
	ol.Internal = 0;
	ol.InternalHigh = 0;
	ol.Offset = (DWORD) BLOCK_SIZE*i;
	ol.OffsetHigh = 0;

	success = ReadFile(hDevice, buffer, BLOCK_SIZE, &dwBytesRead, (LPOVERLAPPED) &ol);

	if((FALSE==success) || (dwBytesRead!=BLOCK_SIZE)) {

		string msg("reading block "+int2str(i)+" failed, "+error_message());

		throw runtime_error(msg);
	}

	return buffer;
}

void close_device(HANDLE hDevice) {

	CloseHandle(hDevice);
}

}

#endif
