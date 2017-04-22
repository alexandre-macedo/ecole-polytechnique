#include "interfacepsc.h"
#include "ui_interfacepsc.h"

#include <QDialog>
#include <QTableWidgetItem>
#include <QTableWidget>
#include <iostream>

InterfacePSC::InterfacePSC(QWidget *parent, std::vector<std::string> v) :
    QMainWindow(parent),
    ui(new Ui::InterfacePSC)
{
    //int* valores[2];

    //QStringList titulos;

    //std::vector<std::string> vetor;

    ui->setupUi(this);
    this->vetor = v;

    //ui->myTable->setColumnCount(1);
    //titulos << "Value";
    //ui->myTable->setHorizontalHeaderLabels(titulos);

    // Latitude
    ui->myTable->setItem(-1, 1, new QTableWidgetItem( vetor.at(0).c_str() )); // "latitude"
    // Longitude
    ui->myTable->setItem(0, 1, new QTableWidgetItem( vetor.at(1).c_str() )); // "longitude"
    // Compass
    ui->myTable->setItem(1, 1, new QTableWidgetItem( vetor.at(2).c_str() )); // // "compass"
    // Temperature
    ui->myTable->setItem(2, 1, new QTableWidgetItem( vetor.at(3).c_str() )); // "temperature"
    // Acceleration
    ui->myTable->setItem(3, 1, new QTableWidgetItem( vetor.at(4).c_str() )); // "acceleration"

    //std::cout << "primeira posicao: " << vetor.at(0)<< std::endl;
}

InterfacePSC::~InterfacePSC()
{
    delete ui;
}

//void InterfacePSC::setValues(std::vector<std::string> v){
//   this->vetor = v;
//}

void InterfacePSC::on_quitButton_clicked()
{
    delete this;
}

/*

void InterfacePSC::saveDataIntoTable(std::string GPS)
{
   if (!myTable)
      return;

   const int currentRow = myTable->rowCount();
   myTable->setRowCount(currentRow + 1);
   myTable->setItem(currentRow, 1, new QTableWidgetItem(GPS));
}

*/


