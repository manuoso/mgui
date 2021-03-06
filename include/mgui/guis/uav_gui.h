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
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsItem>

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <mutex>
#include <thread>

#include <marble/MarbleWidget.h>

#include <rapidjson/document.h>

#ifdef MGUI_USE_FASTCOM
    #include <fastcom/Subscriber.h>
    #include <fastcom/Publisher.h>
#endif

#ifdef MGUI_USE_ROS
    #include <ros/ros.h>
    #include <geometry_msgs/PoseStamped.h>
    #include <geometry_msgs/TwistStamped.h>
    #include <std_msgs/String.h>
    #include <std_msgs/UInt8.h>
    #include <mgui/WaypointData.h>
    #include <mgui/CommandData.h>
    #include <mgui/VelocityData.h>
#endif

namespace Ui{
    class UAV_gui;
}

class UAV_gui : public QMainWindow {
    Q_OBJECT

    public:
        /// Constructor
        explicit UAV_gui(QWidget *parent = 0);

        /// Destructor
        ~UAV_gui();

        /// Method that configure PCL GUI
        /// \param _argc: from main
        /// \param _argv: from main
        /// \return true if params are good or without errors, false if something failed
        bool configureGUI(int _argc, char **_argv);

        /// Struct for send commands to the UAV
        struct command{
            int type;
            float height;
            float x;
            float y;
            float z;
        };

        /// Struct for received pose and send the velocity of the UAV
        struct pose{
            float x;
            float y;
            float z;
        };

    signals:
        /// Signal that warns that there is a change in the pose of the uav
        void localPositionChanged();

        /// Signal that warns that there is a change in the vel of the uav
        void velChanged();
        
        /// Signal that warns that there is a change in the state of the uav
        void stateChanged();

        /// Signal that warns that there is a change in the list of waypoints
        void listWPChanged();

    private slots:
        /// Slot that send takeoff to the UAV
        void takeOff();

        /// Slot that send land to the UAV
        void land();   

        /// Slot that run get velocity of the UAV
        void run_getVelo();

        /// Slot that stop the thread of take velocity of the UAV
        void stop_getVelo();
        
        /// Slot that run get local position of the UAV
        void run_localPose();

        /// Slot that stop the thread of take local position of the UAV
        void stop_localPose();

        /// Slot that send a custom pose to go the UAV
        void run_customPose();

        /// Slot that add a waypoint of a value of a custom pose
        void add_customPose();

        /// Slot that send a desired velocity to go the UAV
        void run_velocity();

        /// Slot that stop sending a desired velocity to go the UAV
        void stop_velocity();

        /// Slot that send a list of waypoints to go the UAV
        void run_wayPoints();

        /// Slot that delete a desired waypoint of a list of waypoints
        void delete_waypoints();

        /// Slot that send a message of emergency to finish all in the UAV
        void emergencyStop();

    private:
        /// Method that update local position of the UAV
        void updateLocalPose();
    
        /// Method that update the velocity of the UAV
        void updateVel();

        /// Method that update the state of the UAV
        void updateState();

        /// Method that update the list of waypoints to send
        void updateListWP();

        #ifdef MGUI_USE_ROS
            /// Method for visualize a change of pose from a topic of ROS
            /// \param _msg: data receive to update pose
            void CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& _msg);

            /// Method for visualize a change of velocity from a topic of ROS
            /// \param _msg: data receive to update velocity
            void CallbackVel(const geometry_msgs::TwistStamped::ConstPtr& _msg);

            /// Method for visualize a change of state from a topic of ROS
            /// \param _msg: data receive to update state
            void CallbackState(const std_msgs::UInt8::ConstPtr& _msg);

            /// Method for receive services with waypoints in ROS
            bool CallbackWP(mgui::WaypointData::Request &_req, mgui::WaypointData::Response &_res);

        #endif

    private:
        Ui::UAV_gui *ui;

        QGraphicsScene *scene_;

        Marble::MarbleWidget *mapWidget_;

        rapidjson::Document configFile_;
        
        #ifdef MGUI_USE_FASTCOM
            fastcom::Publisher<command> *pubCommand_ = nullptr;
            fastcom::Subscriber<int> *subsState_ = nullptr;
            fastcom::Subscriber<pose> *subsPose_ = nullptr;
            fastcom::Subscriber<pose> *subsVel_ = nullptr;
            fastcom::Subscriber<int> *subsCheck_ = nullptr;
            fastcom::Subscriber<pose> *subsWP_ = nullptr;
        #endif

        #ifdef MGUI_USE_ROS
            ros::Subscriber poseSub_, velSub_, stateSub_;
            ros::ServiceClient commandSrv_, positionSrv_, velocitySrv_;
            ros::ServiceServer wpSrvRec_;
        #endif

        std::thread *velocityThread_, *localPoseThread_, *getVelThread_;
        std::thread sendWPThread_;
        std::mutex objectLockPose_, objectLockVel_, objectLockState_, objectLock_;
        
        std::chrono::time_point<std::chrono::high_resolution_clock> lastTimePose_, lastTimeVel_, lastTimeSendVel_;	
        std::vector<std::pair<int, std::vector<double>>> waypoints_;
        
        int idWP_ = 0;
        std::string idUAV_;
        std::string stateUAV_;
        pose poseUAV_, velUAV_, sendVelUAV_;
        int oldData_ = 0;
        bool printLocalPose_ = false;
        bool printVel_ = false;
        bool sendVelocity_ = false;
        bool sendNextWP_ = false;
        bool stopAll_ = false;
};

#endif // GUIS_UAV_GUI_H
