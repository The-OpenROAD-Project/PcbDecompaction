#include <iostream>
#include "drc.h"
#include "shape.h"

int main(int argc, char *argv[])
{
    std::string designName = argv[1];
    std::string lpFile = "bm3_3.lp";
    std::string solFile = "bm3_2.sol";
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

    
    Drc drc(db);
    drc.clearEquations();
    drc.createRTree();
    drc.traverseRTree();
    //int id = 891;
    //drc.printObject(id);
    //drc.printDrc();
    drc.readLPSolution(solFile);
    drc.updateDatabase();
    drc.updatePinsShapeAndPosition();
    //drc.printObject(id);
    //drc.printDrc();
    int objId = 755;
    drc.printObject(objId);
    drc.writeLPfile(lpFile);
    //

    //drc.printObject();
    //drc.printDrc();
    
    
    drc.printObject(objId);
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


    return 0;
}
