#include <iostream>
#include "drc.h"

int main(int argc, char *argv[])
{
    std::string designName = argv[1];
    auto db = kicadPcbDataBase{designName};
    db.printUnconnectedPins();
    Drc drc(db);
    drc.createRTree();
    //drc.printObject();
    //drc.printDrc();

    
    
    return 0;
}
