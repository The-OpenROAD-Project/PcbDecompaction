#include <iostream>
#include "drc.h"

int main(int argc, char *argv[])
{
    std::string designName = argv[1];
    auto db = kicadPcbDataBase{designName};
    //db.printKiCad();
    //db.printComp();
    //db.printNet();
    /*db.printInst();
    db.printNetclass();*/
    //db.printUnconnectedPins();
    Drc drc(db);
    drc.createRTree();
    //drc.traverseRTree();
    //drc.printObject();
    drc.printDrc();
    //drc.testProjection();

    
    
    return 0;
}
