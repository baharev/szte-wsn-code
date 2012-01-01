#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <windows.h>
#include <winioctl.h>
#include <stdio.h>

using namespace std;

void replace(string& s, const char old, const char new_char) {

    string::size_type pos = 0;

    while ((pos = s.find(old, pos)) != string::npos) {

    	s.at(pos) = new_char;
    }
}

unsigned int GB() {

	unsigned int one = 1;

	return (one << 30);
}

double byte_to_GB(int64_t size) {

	return static_cast<double>(size)/GB();
}

const std::string size_in_GB(int64_t size) {

	double size_in_GB = byte_to_GB(size);

	ostringstream os;

	os << setprecision(2) << fixed << size_in_GB << flush;

	return os.str();
}

const string error_message() {

    LPVOID lpMsgBuf;
    DWORD dw = GetLastError();

    FormatMessageA(
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

    const char* fmsg = (lpMsgBuf!=NULL)?static_cast<char*>(lpMsgBuf):"(no formatted message)";

    os << fmsg << " (code: " << dw << ")";

    LocalFree(lpMsgBuf);

    string msg(os.str());

    replace(msg, '\n', ' ');

    replace(msg, '\r', ' ');

    return msg;
}

HANDLE open_device(const char* path, DWORD access) {

	HANDLE hDevice;

	hDevice = CreateFileA(path, // drive to open
		access,           // access to the drive
		FILE_SHARE_READ | // share mode
		FILE_SHARE_WRITE,
		NULL,             // default security attributes
		OPEN_EXISTING,    // disposition
		FILE_FLAG_NO_BUFFERING, // file attributes
		NULL);            // do not copy file attributes

	if (hDevice == INVALID_HANDLE_VALUE) // cannot open the drive
	{
		const string msg = error_message();

		throw runtime_error(msg);
	}

	return hDevice;
}

template <typename T>
void device_io_ctrl(HANDLE hDevice, DWORD operation, T* dat) {

	BOOL bResult   = FALSE;
	DWORD bytesReturned;

	bResult = DeviceIoControl(hDevice,   // device to be queried
			  	  	  	  	  operation, // operation to perform
	                          NULL, 0,   // no input buffer
	                          (LPVOID*)dat, sizeof(*dat), // output buffer
	                          &bytesReturned,       // # bytes returned
	                          (LPOVERLAPPED) NULL); // synchronous I/O

	if (bResult == FALSE) {

		const string msg = error_message();

		throw runtime_error(msg);
	}
}

void real_main(char* path) {

	HANDLE h = open_device(path, GENERIC_READ|GENERIC_WRITE);

	cout << "Handle for " << path << " is obtained" << endl;

	GET_LENGTH_INFORMATION len;

	device_io_ctrl(h, IOCTL_DISK_GET_LENGTH_INFO, &len);

	int64_t length = len.Length.QuadPart;

	cout << "Length: " << length << " bytes, ";

	cout << size_in_GB(length) << " GB" << endl;
}

int main(int argc, char* argv[]) {

	if (argc!=2) {
		cout << "Usage: " << argv[0] << " \\\\.\\DriveLabel:" << endl;
		return 1;
	}
	try {
		real_main(argv[1]);
	}
	catch (runtime_error& e) {
		cout << "Error: " << e.what() << endl;
		return 1;
	}
	return 0;
}
