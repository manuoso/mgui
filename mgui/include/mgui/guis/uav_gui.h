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

#include <rapidjson/document.h>

#include <fastcom/Subscriber.h>
#include <fastcom/Publisher.h>

namespace Ui{
    class UAV_gui;
}

class UAV_gui : public QMainWindow {
    Q_OBJECT

    public:
        explicit UAV_gui(QWidget *parent = 0);
        ~UAV_gui();

        bool configureGUI(int _argc, char **_argv);

	struct command{
		std::string type;
		float height;
		float x;
		float y;
		float z;
	    };

	struct pose{
		float x;
		float y;
		float z;
	    };

signals:
    /// Signal that warns that there is a change in the pose of the uav
    void localPositionchanged();
    
    void stateChanged();

    private slots:

        void takeOff();
        void land();

        void run_gpsPose();

        void run_localPose();
        void stop_localPose();

        void run_customPose();
        void add_customPose();

        void run_velocity();
        void stop_velocity();

        void run_waypoints();
        void delete_waypoints();

    private:
        void updateLocalPose();

        void updateState();

    private:
        Ui::UAV_gui *ui;

        Marble::MarbleWidget *mMapWidget;

	fastcom::Publisher<command> *mPubCommand;
   	fastcom::Subscriber<std::string> *mSubsState;
	fastcom::Subscriber<pose> *mSubsPose;

	std::thread mVelocityThread, mLocalPoseThread;
	
	std::chrono::time_point<std::chrono::high_resolution_clock> mLastTimePose;	

        std::string mIdUAV;
	std::string mStateUAV;
        pose mPoseUAV;
        bool mPrintLocalPose = false;
        bool mSendVelocity = false;
};

#endif // GUIS_UAV_GUI_H
