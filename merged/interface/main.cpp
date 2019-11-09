#include "interfacepsc.h"
#include <QApplication>
#include <iostream>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    std::vector<std::string> v;
    v.push_back("latitude");
    v.push_back("longitude");
    v.push_back("compass");
    v.push_back("temperature");
    v.push_back("acceleration");

    //std::cout << "tamanho original " << v.size() << std::endl;


    InterfacePSC* w = new InterfacePSC(0, v);

    //w.setValues(v);
    (*w).show();

    return a.exec();
}
