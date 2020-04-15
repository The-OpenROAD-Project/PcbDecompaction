#include "clpSolver.h"

bool ClpSolver::solver(std::string lpFileName, std::string solFileName)
{
    ClpSimplex model;
    int status;
    status = model.readLp(lpFileName.c_str());
    if (!status)
    {
        model.primal();
    }

    int numberRows = model.numberRows();
    double *rowPrimal = model.primalRowSolution();
    double *rowDual = model.dualRowSolution();

    int iRow;

   /* for (iRow = 0; iRow < numberRows; iRow++)
        printf("Row %d, primal %g, dual %g\n", iRow,
               rowPrimal[iRow], rowDual[iRow]); */

    int numberColumns = model.numberColumns();
    double *columnPrimal = model.primalColumnSolution();
    //auto columnName = model.columnNames();
    double *columnDual = model.dualColumnSolution();

    int iColumn;

    std::ofstream file;

    file.open(solFileName);

    for (iColumn = 0; iColumn < numberColumns; iColumn++)
    {
       // printf("Column %d, primal %g, dual %g, name %s\n", iColumn,
       //        columnPrimal[iColumn], columnDual[iColumn], (model.getColumnName(iColumn)).c_str());

	file << model.getColumnName(iColumn) << " " << columnPrimal[iColumn] << std::endl;
        // std::cout << columnName[iColumn] << std::endl;
    }

    model.dual();
    return true;
}
