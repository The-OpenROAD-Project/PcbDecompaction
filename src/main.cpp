#include <iostream>
#include "decompaction.h"
#include "shape.h"
#include "gurobiSolver.h"
#include "clpSolver.h"

int main(int argc, char *argv[])
{
    std::string designName = argv[1];
    std::string lpFile = argv[2];
    std::string solFile = argv[3];
    auto db = kicadPcbDataBase{designName};
    //db.printLockedInst();
    //db.printKiCad();
    //db.printNodes();
    //db.printComp();
    //db.printInst();
    //db.printNodes();
    //db.printNet();
    /*db.printInst();
    db.printNetclass();*/
    //db.printUnconnectedPins();

    Decompaction drc(db);
    drc.clearEquations();
    drc.createRTree();
    drc.traverseRTree();
    //int id = 891;
    //drc.printObject(id);
    //drc.printDrc();
    drc.printObject();
    if (solFile != "N")
    {
        drc.readLPSolution(solFile);
        drc.updateDatabase();
        drc.updatePinsShapeAndPosition();
    }
    //drc.printObject(id);
    //drc.printDrc();
    /*int objId = 755;
    drc.printObject(objId);*/
    drc.addWidthToBusSegmentEquation();
    //drc.writeLPfileForBus(lpFile);
    drc.writeLPfileForBus(lpFile);
    //

    //drc.printDrc();

    //drc.printObject(objId);
    db.printKiCad();

    //db.printClearanceDrc();
    //drc.testProjection();

    /*point_2d p1, p2;
    p1.m_x = 2.2;
    p1.m_y = 1.1;
    p2.m_x = 2.2;
    p2.m_y = 2.3;
    std::cout << "Larger " << (p1<=p2) << std::endl;*/

    db.printInst();

    GurobiSolver model;
    std::string solLpFile = lpFile + ".sol";
    model.solver(lpFile, solLpFile);

    ClpSolver clpModel;
    clpModel.solver(lpFile, solLpFile);
    //db.printInst();

    /*GurobiSolver model;
    model.solver(lpFile, solFile);*/

    return 0;
}
