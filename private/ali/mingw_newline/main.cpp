#include <iostream>
#include <fstream>
#include <string>

using namespace std;

int main() {

	const char dummy[] = "dummy.txt";

	const string text("Some text ending with a LF");

	ofstream out(dummy, ios_base::binary);

	out << text << '\n' << flush;

	out.close();

	cout << "File created, hit enter to read it " << flush;

	string buffer;

	cin.get();

	ifstream in(dummy);

	getline(in, buffer);

	cout << "Is the same as written? " << (text==buffer?"yes":"no") << endl;

	return 0;
}
