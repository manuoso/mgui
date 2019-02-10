#ifndef UAV_GUI_H
#define UAV_GUI_H

#include <QMainWindow>

#include <uav_abstraction_layer/ual.h>
#include <marble/MarbleWidget.h>

namespace Ui {
class UAV_gui;
}

class UAV_gui : public QMainWindow
{
    Q_OBJECT

public:
    explicit UAV_gui(QWidget *parent = 0);
    ~UAV_gui();

    bool configureGUI(std::vector<std::pair<std::string, std::string>> _config, int _argcCopy, char ** _argvCopy);

private slots:

    void landClicked();

    void takeOffClicked();

    void Run_poseClicked();

    void Run_customPoseClicked();

private:
    Ui::UAV_gui *ui;

    Marble::MarbleWidget *mMapWidget;
    grvc::ual::UAL *mUal;
    geometry_msgs::PoseStamped mPose;
    std::string mIdUAV;
    
};


#endif // UAV_GUI_H
