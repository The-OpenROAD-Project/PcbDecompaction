#include <iostream>
#include "drc.h"

int main(int argc, char *argv[])
{
    std::string designName = argv[1];
    auto db = kicadPcbDataBase{designName};
    //db.printLockedInst();
    db.printKiCad();
    //db.printNodes();
    //db.printComp();
    db.printInst();
    //db.printNet();
    /*db.printInst();
    db.printNetclass();*/
    //db.printUnconnectedPins();
    /*Drc drc(db);
    drc.createRTree();
    drc.traverseRTree();
    //drc.writeLPfile();
    drc.readLPSolution();
    drc.updateDatabase();

    //drc.printObject();
    //drc.printDrc();
    db.printKiCad();*/
    //db.printClearanceDrc();
    //drc.testProjection();

    return 0;
}
