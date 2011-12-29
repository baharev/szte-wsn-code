
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <windows.h>

using namespace std;

const string error_message() {

    DWORD dw = GetLastError();

    ostringstream os;

    os << "error code: " << dw;

    return os.str();
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
		//const string msg = error_message();

		//throw runtime_error(msg);

		throw runtime_error(error_message());
	}

	return hDevice;
}

int main() {

	try {

		open_for_reading("\\\\.\\X:"); // Just make sure it does not exist
	}
	catch (runtime_error& e) {

		cout << e.what() << endl;
	}

	return 0;
}
