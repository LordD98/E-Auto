// C++Test.cpp: Definiert den Einstiegspunkt für die Konsolenanwendung.
//

#include <iostream>
#include "classA.h"

using namespace std;

int main()
{
	cout << "Hallo Wolr!" << endl;
	classA a1(2);
	a1.f();
	cin.get();
    return 0;
}

