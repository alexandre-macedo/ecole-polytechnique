#ifndef INTERFACEPSC_H
#define INTERFACEPSC_H

#include <QMainWindow>
#include <vector>


namespace Ui {
class InterfacePSC;
}

class InterfacePSC : public QMainWindow
{
    Q_OBJECT

public:
    std::vector<std::string> vetor;


    explicit InterfacePSC(QWidget *parent , std::vector<std::string> );
    ~InterfacePSC();
   // void saveDataIntoTable(std::string);
   // void setValues(std::vector<std::string>);

private slots:
    void on_quitButton_clicked();

private:
    Ui::InterfacePSC *ui;
};

#endif // INTERFACEPSC_H
