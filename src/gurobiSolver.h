#ifndef GUROBI_SOLVER_H
#define GUROBI_SOLVER_H


#include "gurobi_c++.h"
#include <fstream>
using namespace std;

class GurobiSolver{
    public:
    GurobiSolver(){}
    ~GurobiSolver(){}
    bool solver(string lpFileName, string solFileName);

};

#endif