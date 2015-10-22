#include"tools/ToString.h"


string intToString(int value)
{
    stringstream ss;
    ss << value;
    return ss.str();
};

string floatToString(float value)
{
    stringstream ss;
    ss << value;
    return ss.str();
};

string doubleToString(double value)
{
    stringstream ss;
    ss << value;
    return ss.str();
};

string longUnsignedIntToString(long unsigned int value)
{
    stringstream ss;
    ss << value;
    return ss.str();
};
