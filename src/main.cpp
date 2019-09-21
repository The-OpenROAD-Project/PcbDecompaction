#include <iostream>
#include "drc.h"

int main(int argc, char *argv[])
{
    Drc drc;
    drc.printDrc();
    std::cout << "HELLO" << std::endl;

    std::string designName = argv[1];
    auto db = kicadPcbDataBase{designName};
    db.printNet();
    return 0;
}
