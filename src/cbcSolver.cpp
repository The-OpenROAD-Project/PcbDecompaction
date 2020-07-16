#include "cbcSolver.h"

bool CbcSolver::solver(std::string lpFileName, std::string solFileName)
{
    Cbc_Model *model = Cbc_newModel();
    int status;
    status = Cbc_readLp(model, lpFileName.c_str());

    Cbc_solve(model);

    const double *sol = Cbc_getColSolution(model);
    int n = Cbc_getNumCols(model);
    size_t length = Cbc_maxNameLength(model);
    std::ofstream file;

    file.open(solFileName);
    for (int i = 0; i < n; ++i)
    {
        char *name = (char *)malloc(length);
        Cbc_getColName(model, i, name, length);
        printf("Column %d, val %g, name %s\n", i, sol[i], name);

        file << name << " " << sol[i] << std::endl;
        //std::cout << name;
        //std::cout << " " << sol[i] << std::endl;
    }
    Cbc_deleteModel(model);
}