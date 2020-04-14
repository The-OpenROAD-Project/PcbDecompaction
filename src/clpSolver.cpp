#include "clpSolver.h"

bool ClpSolver::solver(std::string lpFileName, std::string solFileName)
{
    ClpSimplex model;
    int status;
    status = model.readLp(lpFileName);
    if (!status)
    {
        model.primal();
    }

    int numberRows = model.numberRows();
    double *rowPrimal = model.primalRowSolution();
    double *rowDual = model.dualRowSolution();

    int iRow;

    for (iRow = 0; iRow < numberRows; iRow++)
        printf("Row %d, primal %g, dual %g\n", iRow,
               rowPrimal[iRow], rowDual[iRow]);

    int numberColumns = model.numberColumns();
    double *columnPrimal = model.primalColumnSolution();
    //auto columnName = model.columnNames();
    double *columnDual = model.dualColumnSolution();

    int iColumn;

    for (iColumn = 0; iColumn < numberColumns; iColumn++)
    {
        printf("Column %d, primal %g, dual %g, name %s\n", iColumn,
               columnPrimal[iColumn], columnDual[iColumn], (model.getColumnName(iColumn)).c_str());

        // std::cout << columnName[iColumn] << std::endl;
    }

    model.dual();
    return true;
}