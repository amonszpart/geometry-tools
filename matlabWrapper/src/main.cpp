#include <iostream>

#include "matlab_wrapper/Matlab.h"

int main()
{
    using namespace smartgeometry;
    using namespace std;

    std::vector<Matlab::MatlabVariable> in_vars;
    in_vars.emplace_back( Matlab::MatlabVariable(1,2,"A", {1,2}) );
    in_vars.emplace_back( Matlab::MatlabVariable(1,2,"B", {3,4}) );

    std::vector<Matlab::MatlabVariable> out_vars;
    out_vars.push_back(Matlab::MatlabVariable(0,0,"C"));

    std::string code = "C = A + B;";
    bool success = Matlab::runCode( in_vars, out_vars, code );

    cout << "Hello World!" << endl;

    return 0;
}

