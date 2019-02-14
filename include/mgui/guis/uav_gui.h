//---------------------------------------------------------------------------------------------------------------------
//  MGUI
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2019 Manuel Pérez Jiménez (a.k.a. manuoso) manuperezj@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------

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
    
    struct gps{
        float lat;
        float lon;
        float alt;
    };

signals:
    /// Signal that warns that there is a change in the pose of the uav
    void localPositionChanged();

    void gpsChanged();
    
    void stateChanged();

    private slots:

        void takeOff();
        void land();

        void run_gpsPose();
        void stop_gpsPose();

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

        void updateGPS();

        void updateState();

    private:
        Ui::UAV_gui *ui;

        Marble::MarbleWidget *mMapWidget;

        rapidjson::Document mConfigFile;

        fastcom::Publisher<command> *mPubCommand;
        fastcom::Subscriber<std::string> *mSubsState;
        fastcom::Subscriber<pose> *mSubsPose;
        fastcom::Subscriber<gps> *mSubsGPS;

        std::thread mVelocityThread, mLocalPoseThread, mGPSThread;
        
        std::chrono::time_point<std::chrono::high_resolution_clock> mLastTimePose, mLastTimeGPS;	
        std::vector<std::pair<int, std::vector<double>>> mWayPoints;
        
        int mIDWP = 0;
        std::string mIdUAV;
        std::string mStateUAV;
        pose mPoseUAV;
        gps mGPSUAV;
        bool mPrintLocalPose = false;
        bool mPrintGPS = false;
        bool mSendVelocity = false;
};

#endif // GUIS_UAV_GUI_H
