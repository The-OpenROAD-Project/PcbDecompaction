#ifndef CLP_SOLVER_H
#define CLP_SOLVER_H
#include <fstream>
#include "ClpSimplex.hpp"

class ClpSolver
{
public:
    ClpSolver() {}
    ~ClpSolver() {}
    bool solver(std::string lpFileName, std::string solFileName);
};
#endif