
#include "classA.h"
#include <iostream>

using namespace std;

int a = 1;

classA::classA()
{
}

classA::classA(int ain) : a(ain)
{
}

void classA::f()
{
	cout << "Test1: " << a			<< endl;
	cout << "Test2: " << classA::a	<< endl;
	cout << "Test3: " << this->a	<< endl;
	cout << "Test4: " << ::a		<< endl;
}