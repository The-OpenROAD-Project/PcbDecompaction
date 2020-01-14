#include <iostream>
#include "drc.h"

int main(int argc, char *argv[])
{
    std::string designName = argv[1];
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

    /*
    Drc drc(db);
    drc.clearEquations();
    drc.createRTree();
    drc.traverseRTree();
    //int id = 891;
    //drc.printObject(id);
    //drc.printDrc();
    //drc.readLPSolution();
    //drc.updateDatabase();
    //drc.printObject(id);
    //drc.printDrc();
    drc.writeLPfile();
    //

    drc.printObject();
    //drc.printDrc();
    */
    db.testInstAngle();

    db.printKiCad();

    //db.printClearanceDrc();
    //drc.testProjection();

    return 0;
}
