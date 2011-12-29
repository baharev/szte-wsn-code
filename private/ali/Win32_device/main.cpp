
#include <iomanip>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <stdint.h>
#include <windows.h>
#include <winioctl.h>

using namespace std;

const std::string int2str(int i) {

	ostringstream os;

	os << i << flush;

	return os.str();
}

void replace(string& s, const char old, const char new_char) {

    string::size_type pos = 0;

    while ((pos = s.find(old, pos)) != string::npos) {

    	s.at(pos) = new_char;
    }
}

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

HANDLE open_device(const char* path, DWORD access = GENERIC_READ) {

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

unsigned int GB() {

	unsigned int one = 1;

	return (one << 30);
}

double byte_to_GB(const int64_t size) {

	return static_cast<double>(size)/GB();
}

const std::string card_size_GB(const int64_t size) {

	double size_in_GB = byte_to_GB(size);

	ostringstream os;

	os << setprecision(2) << fixed << size_in_GB << " GB" << flush;

	return os.str();
}

void throw_if_larger_than_2GB(const int64_t size) {

	int64_t int32_max = (numeric_limits<int32_t>::max)(); // Otherwise error C2589

	if (size > int32_max || size <= 0) {

		throw runtime_error("card size is larger than 2 GB");
	}
}

int32_t cast_to_int32(int64_t size) {

	throw_if_larger_than_2GB(size);

	return size;
}

int main() {

	try {

		HANDLE hDevice = open_device("\\\\.\\C:", GENERIC_WRITE);

		int64_t size = size_in_bytes(hDevice);

		cout << card_size_GB(size) << endl;

		throw_if_larger_than_2GB(size);
	}
	catch (runtime_error& e) {

		cout << e.what() << endl;
	}

	return 0;
}
