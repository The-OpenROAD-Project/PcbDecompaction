#include "decompaction.h"
#include "kicadPcbDataBase.h"
#include "clpSolver.h"

int main(int argc, char *argv[])
{

    auto db = kicadPcbDataBase{designName};

    GridBasedRouter router(db);

    ////////////////BUS//////////////////
    router.routeBus();
    Decompaction decompactor(db);
    decompactor.traverseRTree(std::string lpFile);
    decompactor.readBusLPSolution(solLpFile); //save to db
    decompactor.addSnakingPatterns();         //save new segments to db
    ///////////////////////////////////////////
    /// Problems
    /// "Maintain a double linked lists in db to let router know the connectivity"
    /// 1. Dangling issues
    /// 2. Cannot solve acute angle
    /// 3. Connectivity only exists in memory, cannot do experiments in file.
    //////////////////////////////////////////

    router.route(); //save segments to db

    Decompaction decompactor(db);
    decompactor.traverseRTree(std::string lpFile);
    double curDrcCount = decompactor.printDrc();

    ClpSolver clpModel;
    clpModel.solver(lpFile, solLpFile);

    double curSlacks = decompactor.readLPSolution(solLpFile); // update new positions of objects in db

    if (curDrcCount > alpha || curSlacks > beta)
    {
        while (true)
        {
            double preDrcCount = curDrcCount, preSlacks = curSlacks;
            router.route();

            decompactor.traverseRTree(std::string lpFile);
            curDrcCount = decompactor.printDrc();
            clpModel.solver(lpFile, solLpFile);
            curSlacks = decompactor.readLPSolution(solLpFile);
            if (abs(preDrcCount - curDrcCount) < threshold && abs(preSlacks - curSlacks) < epsilon)
                break;
        }
    }

    return 0;
}