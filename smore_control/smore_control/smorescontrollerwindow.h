#ifndef SMORESCONTROLLERWINDOW_H
#define SMORESCONTROLLERWINDOW_H

#include <QMainWindow>
#include <string>
#include <vector>

namespace Ui {
class SmoresControllerWindow;
}

class SmoresControllerWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit SmoresControllerWindow(QWidget *parent = 0);
    ~SmoresControllerWindow();
    void SetSmoreNames(std::vector<std::string> names);
    
private:
    Ui::SmoresControllerWindow *ui;
    std::vector<std::string> names;
};

#endif // SMORESCONTROLLERWINDOW_H
