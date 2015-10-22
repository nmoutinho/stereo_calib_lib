#ifndef _TO_STRING_H_
#define _TO_STRING_H_

#include <iostream>
#include <string>
#include <sstream>

using namespace std;

/*string toString(int value)
{
    stringstream ss;
    ss << value;
    return ss.str();
};

string toString(double value)
{
    stringstream ss;
    ss << value;
    return ss.str();
};//*/

string intToString(int value);
string floatToString(float value);
string doubleToString(double value);
string longUnsignedIntToString(long unsigned int value);

#endif
