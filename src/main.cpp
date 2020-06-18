#include <iostream>
#include "decompaction.h"
#include "shape.h"
//#include "gurobiSolver.h"
#include "clpSolver.h"
#include "cbcSolver.h"

int main(int argc, char *argv[])
{
    std::string designName = argv[1];
    std::string lpFile = argv[2];
    //std::string solFile = argv[3];
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
    drc.printAllNetLength();
    drc.clearEquations();
    drc.createRTree();
    drc.traverseRTree();
    //int id = 891;
    //drc.printObject(id);
    //drc.printDrc();
    drc.printObject();

    //drc.printObject(id);
    //drc.printDrc();
    /*int objId = 755;
    drc.printObject(objId);*/

    //

    /////////////FOR BUS////////////////
    drc.addWidthToBusSegmentEquation();
    drc.writeLPfileForBus(lpFile);
    //drc.printDrc();
    /////////////////////////////////////

    //drc.printObject(objId);
    //db.printKiCad();

    //db.printClearanceDrc();
    //drc.testProjection();

    /*point_2d p1, p2;
    p1.m_x = 2.2;
    p1.m_y = 1.1;
    p2.m_x = 2.2;
    p2.m_y = 2.3;
    std::cout << "Larger " << (p1<=p2) << std::endl;*/

    db.printInst();

    std::string solLpFile = lpFile + ".sol";
    /*GurobiSolver model;
    model.solver(lpFile, solLpFile);*/

    ClpSolver clpModel;
    //clpModel.solver(lpFile, solLpFile);
    //db.printInst();
    CbcSolver cbcModel;
    cbcModel.solver(lpFile, solLpFile);

    /*GurobiSolver model;
    model.solver(lpFile, solFile);*/

    {
        drc.readLPSolution(solLpFile);
        drc.updateDatabase();
        drc.updatePinsShapeAndPosition();
    }

    std::cout << "#############ADD SNACKING############" << std::endl;
    drc.addSnakingPatterns();

    //drc.testBBoxSnaking();

    /*std::cout << "===========SNACKING===========" << std::endl;
    drc.getSnaking();*/

    /*
    std::cout << "===========BBOX TEST==========" << std::endl;
    drc.getBoundingBox();*/

    db.printKiCad();
    return 0;
}
