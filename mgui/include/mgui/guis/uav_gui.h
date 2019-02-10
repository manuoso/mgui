/////////////////////////////////////////////////////////////////
//															   //
//                       Header UAV GUI                        //
//															   //
//				Author: Manuel P. J. (aka. manuoso)			   //
//															   //
/////////////////////////////////////////////////////////////////

#ifndef GUIS_UAV_GUI_H
#define GUIS_UAV_GUI_H

#include <QMainWindow>
#include <QTextStream>

#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>

#include <marble/MarbleWidget.h>

namespace Ui {
    class UAV_gui;
}

class UAV_gui : public QMainWindow {
    Q_OBJECT

    public:
        explicit UAV_gui(QWidget *parent = 0);
        ~UAV_gui();

        bool configureGUI(int _argc, char **_argv);

    private slots:

        void takeOff();
        void land();

        void run_gpsPose();

        void run_localPose();
        void add_localPose();

        void run_customPose();
        void add_customPose();

        void run_velocity();
        void stop_velocity();

        void run_waypoints();
        void delete_waypoints();

    private:
        Ui::UAV_gui *ui;

        Marble::MarbleWidget *mMapWidget;

        std::string mIdUAV;
};

#endif // GUIS_UAV_GUI_H
