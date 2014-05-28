#include "smorescontrollerwindow.h"
#include "ui_smorescontrollerwindow.h"

SmoresControllerWindow::SmoresControllerWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SmoresControllerWindow)
{
    ui->setupUi(this);
}

SmoresControllerWindow::~SmoresControllerWindow()
{
    delete ui;
}

void SmoresControllerWindow::SetSmoreNames(std::vector<std::string> names)
{
    this->names = std::vector<std::string>(names);

}
