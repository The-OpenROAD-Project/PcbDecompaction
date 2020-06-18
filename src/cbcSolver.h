#ifndef CBC_SOLVER_H
#define CBC_SOLVER_H
#include <fstream>
//#include "CbcModel.hpp"
#include "Cbc_C_Interface.h"

class CbcSolver
{
public:
    CbcSolver() {}
    ~CbcSolver() {}
    bool solver(std::string lpFileName, std::string solFileName);
};
#endif