#include "smorescontrollerwindow.h"
#include "ui_smorescontrollerwindow.h"
#include <qthread.h>
#include <qtimer.h>

SmoresControllerWindow::SmoresControllerWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::SmoresControllerWindow)
{
    ui->setupUi(this);
	qRegisterMetaType<Components>("Components");
	qRegisterMetaType<boost::any>("boost::any");
	
	stringListModel = new QStringListModel(this);
    ui->listView_smoresName->setModel(stringListModel);
	
	connect(this, SIGNAL(SetGUIComponents_SIGNAL(Components,boost::any)),
			this, SLOT(SetGUIComponents_SLOT(Components,boost::any)));
	connect(ui->listView_smoresName->selectionModel(), SIGNAL(selectionChanged(QItemSelection,QItemSelection)),
            this, SLOT(listView_smoresName_selectionChanged(QItemSelection,QItemSelection)));
}

SmoresControllerWindow::~SmoresControllerWindow()
{
    delete ui;
}

void SmoresControllerWindow::SetGUIComponents(Components comp, boost::any arg)
{	
	emit SetGUIComponents_SIGNAL(comp, arg);
}

void SmoresControllerWindow::SetGUIComponents_SLOT(SmoresControllerWindow::Components comp, boost::any arg)
{
	if(comp == SMORES_NAMES_LIST_VIEW)
	{
		std::vector<std::string> list = boost::any_cast<std::vector<std::string> >(arg);
		this->SetSmoreNames(list);
	}
}


void SmoresControllerWindow::SetSmoreNames(std::vector<std::string> names)
{
    this->names = std::vector<std::string>(names);
    QStringList list;
    for(int i = 0; i < this->names.size(); i++)
    {
        list.push_back(QString::fromStdString(this->names[i]));
    }
    this->stringListModel->setStringList(list);
    ui->listView_smoresName->setModel(stringListModel);
}
std::string SmoresControllerWindow::GetCurrentSelectedTopic()
{
	return selectedTopic.toStdString();
}

void SmoresControllerWindow::on_btn_loadModules_clicked()
{
    this->btnCalled(GETMODULES);
}

void SmoresControllerWindow::on_btn_Up_clicked()
{
    this->btnCalled(VEL_UP);
}

void SmoresControllerWindow::on_btn_Left_clicked()
{
    this->btnCalled(VEL_LEFT);
}

void SmoresControllerWindow::on_btn_Down_clicked()
{
    this->btnCalled(VEL_DOWN);
}

void SmoresControllerWindow::on_btn_Right_clicked()
{
    this->btnCalled(VEL_RIGHT);
}

void SmoresControllerWindow::on_btn_STOP_clicked()
{
    this->btnCalled(STOP);
}

void SmoresControllerWindow::on_btn_A1Plus_clicked()
{
    this->btnCalled(JOINT_A1_P);
}

void SmoresControllerWindow::on_btn_A1Minus_clicked()
{
    this->btnCalled(JOINT_A1_M);
}

void SmoresControllerWindow::on_btn_A2Plus_clicked()
{
    this->btnCalled(JOINT_A2_P);
}

void SmoresControllerWindow::on_btn_A2Minus_clicked()
{
    this->btnCalled(JOINT_A2_M);
}

void SmoresControllerWindow::on_btn_A1_Swipping_clicked()
{
    this->btnCalled(JOINT_A1_TOGGLE);
}

void SmoresControllerWindow::on_btn_A2_Swipping_2_clicked()
{
    this->btnCalled(JOINT_A2_TOGGLE);
}

void SmoresControllerWindow::listView_smoresName_selectionChanged(const QItemSelection &selected, const QItemSelection &unselected)
{
    QModelIndexList selectedInices = selected.indexes();
    QVariant data = selectedInices[0].data();
	data.convert(QVariant::String);
// 	std::string topic = data.toString().toStdString();
	selectedTopic = data.toString();
	ui->lbl_smoreName->setText(selectedTopic);	
	
}
