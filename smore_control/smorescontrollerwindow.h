#ifndef SMORESCONTROLLERWINDOW_H
#define SMORESCONTROLLERWINDOW_H

#include <QMainWindow>
#include <QItemSelectionModel>
#include <string>
#include <vector>
#include <Qt/qstringlistmodel.h>
#include <boost/signals2.hpp>

namespace Ui {
class SmoresControllerWindow;
}

class SmoresControllerWindow : public QMainWindow
{
    Q_OBJECT
    
public: enum GUIBtnType { GETMODULES, VEL_UP, VEL_DOWN, VEL_RIGHT, VEL_LEFT, STOP, JOINT_A1_P, JOINT_A1_M, JOINT_A2_P, JOINT_A2_M, JOINT_A1_TOGGLE, JOINT_A2_TOGGLE };
public: enum Components { SMORES_NAMES_LIST_VIEW };

public:
    explicit SmoresControllerWindow(QWidget *parent = 0);
    ~SmoresControllerWindow();
    boost::signals2::signal<void (GUIBtnType)> btnCalled;
	void SetGUIComponents(Components comp, boost::any arg);
	std::string GetCurrentSelectedTopic();
	

signals:
	// this SIGNAL is defined for the thread-safe method calling
	void SetGUIComponents_SIGNAL(Components comp, boost::any arg);
    void formLoaded();

private slots:
	// this SLOT is defined for the thread-safe method calling
	void SetGUIComponents_SLOT(Components comp, boost::any arg);
	

private:
    Ui::SmoresControllerWindow *ui;
    std::vector<std::string> names;
    QStringListModel *stringListModel;
	void SetSmoreNames(std::vector<std::string> names);
	QString selectedTopic;

private slots:
    void on_btn_loadModules_clicked();
    void on_btn_Up_clicked();
    void on_btn_Left_clicked();
    void on_btn_Down_clicked();
    void on_btn_Right_clicked();
    void on_btn_STOP_clicked();
    void on_btn_A1Plus_clicked();
    void on_btn_A1Minus_clicked();
    void on_btn_A2Plus_clicked();
    void on_btn_A2Minus_clicked();
    void on_btn_A1_Swipping_clicked();
    void on_btn_A2_Swipping_2_clicked();
    void listView_smoresName_selectionChanged(const QItemSelection &item1, const QItemSelection &item2);
};

#endif // SMORESCONTROLLERWINDOW_H
