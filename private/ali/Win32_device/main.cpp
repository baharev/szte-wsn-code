
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <stdint.h>
#include <windows.h>
#include <winioctl.h>

using namespace std;

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

HANDLE open_for_reading(const char* path) {

	HANDLE hDevice;

	hDevice = CreateFileA(path,  // drive to open
		GENERIC_READ,                // access to the drive
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

unsigned int GB() {

	unsigned int one = 1;

	return (one << 30);
}

double byte_to_GB(int64_t size) {

	return static_cast<double>(size)/GB();
}

const std::string card_size_GB(int64_t size) {

	double size_in_GB = byte_to_GB(size);

	ostringstream os;

	os << setprecision(2) << fixed << size_in_GB << " GB" << flush;

	return os.str();
}

int main() {

	try {

		HANDLE hDevice = open_for_reading("\\\\.\\C:");

		int64_t size = size_in_bytes(hDevice);

		cout << card_size_GB(size) << endl;
	}
	catch (runtime_error& e) {

		cout << e.what() << endl;
	}

	return 0;
}
