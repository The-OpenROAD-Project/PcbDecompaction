#include <iostream>
#include "decompaction.h"
#include "shape.h"
#include "frTime.h"
#include "gurobiSolver.h"
#include "clpSolver.h"
#include "cbcSolver.h"

int main(int argc, char *argv[])
{
    std::string designName = argv[1];
    int maxIter = atoi(argv[2]);
    std::string lpFileName = argv[3];

    std::string outputKicad = argv[4];
    auto db = kicadPcbDataBase{designName};

    int cnt = 0;
    fr::frTime timeObj;
    double objVal = 0, preObjVal;
    while(cnt < maxIter) {
        preObjVal = objVal;
        std::string lpFile = lpFileName + "_" + std::to_string(cnt) + ".lp";

        Decompaction drc(db);
        drc.clearEquations();
        drc.createRTree();
        drc.traverseRTree();
        drc.writeLPfile(lpFile);
        drc.printObject();

        std::string solLpFile = lpFile + ".sol";
        fr::frTime timeObjG;
        std::cout << "################GUROBI SOLVER################" << std::endl;
        GurobiSolver model;
        
        model.solver(lpFile, solLpFile, objVal);

        timeObjG.print();


        std::cout << "==================LOAD RESULT===================" << std::endl;

        {
            drc.readLPSolution(solLpFile);
            drc.updateDatabase();
            drc.updatePinsShapeAndPosition();
        }
        ++cnt;
        if(abs(objVal-preObjVal) < 1000) break;
    }
    std::cout << "Total iterations: " << cnt << std::endl;
    timeObj.print();

    db.printKiCad("", outputKicad);

    return 0;
}
