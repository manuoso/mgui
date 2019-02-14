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

#include <mgui/receivers/UAV_receiver.h>

//---------------------------------------------------------------------------------------------------------------------
bool UAV_receiver::init(int _argc, char**_argv) {

    if (_argc != 2) {
        std::cout << "Bad input arguments, please provide only the path of a json config file with the structure detailed in the documentation" << std::endl;
        return false;
    }

    std::ifstream rawFile(_argv[1]);
    if (!rawFile.is_open()) {
        std::cout << "Error opening config file" << std::endl;
        return false;
    }

    std::stringstream strStream;
    strStream << rawFile.rdbuf(); //read the file
    std::string json = strStream.str(); //str holds the content of the file

    if(mConfigFile.Parse(json.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    std::string ip = mConfigFile["ip_receiver"].GetString();
    int portCommand = mConfigFile["portCommand"].GetInt();
    int portState = mConfigFile["portState"].GetInt();
    int portPose = mConfigFile["portPose"].GetInt();
    int portVel = mConfigFile["portVel"].GetInt();

    // Init UAV controller
    mUAL = new grvc::ual::UAL(_argc, _argv);

    // Initialize Fastcom publishers and subscribers
    mSubsCommand = new fastcom::Subscriber<command>(ip, portCommand);
    mPubState = new fastcom::Publisher<std::string>(portState);
    mPubPose = new fastcom::Publisher<pose>(portPose);
    mPubVel = new fastcom::Publisher<pose>(portVel);

    // Callback of received commands
    mSubsCommand->attachCallback([&](command &_data){
        if(_data.type == "takeoff"){
            mState = eState::TAKEOFF;
            mHeight = _data.height;
        }else if(_data.type == "land"){
            mState = eState::LAND;
        }else if(_data.type == "position"){
            mState = eState::MOVE;
            mX = _data.x;
            mY = _data.y;
            mZ = _data.z;
        }else if(_data.type == "velocity"){
            mState = eState::MOVE_VEL;
            mX = _data.x;
            mY = _data.y;
            mZ = _data.z;
        }else if(_data.type == "wait"){
            mState = eState::WAIT;
        }else if(_data.type == "emergency"){
            mState = eState::EXIT;
        }else{
            std::cout << "Unrecognized state" << std::endl;
            mState = eState::WAIT;
        }
    });

    // Publisher State thread
    mStateThread = std::thread([&](){
        while(!mFin && ros::ok()){
            std::string msg;
            switch(mState){
                case eState::WAIT:
                    msg = "WAIT";
                    break;
                case eState::TAKEOFF:
                    msg = "TAKEOFF";
                    break;
                case eState::LAND:
                    msg = "LANDING";
                    break;
                case eState::MOVE:
                    msg = "MOVE_POSITION";
                    break;
                case eState::MOVE_VEL:
                    msg = "MOVE_VELOCITY";
                    break;
                case eState::EXIT:
                    msg = "EXIT";
                    break;
            }
            mPubState->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    // Publisher Pose thread
    mPoseThread = std::thread([&](){
        while(!mFin && ros::ok()){
            // Get pose
            grvc::ual::Pose p = mUAL->pose();

            pose sendPose;
            sendPose.x = p.pose.position.x;
            sendPose.y = p.pose.position.y;
            sendPose.z = p.pose.position.z;
            mPubPose->publish(sendPose);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    });

    // Publisher Velocity thread
    mVelThread = std::thread([&](){
        while(!mFin && ros::ok()){
            // Get velocity
            grvc::ual::Velocity v = mUAL->velocity();

            pose sendVel;
            sendVel.x = v.twist.linear.x;
            sendVel.y = v.twist.linear.y;
            sendVel.z = v.twist.linear.z;
            mPubVel->publish(sendVel);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    });

    mState = eState::WAIT;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool UAV_receiver::run() {

    while(!mFin && ros::ok()){
        switch (mState) {
            case eState::WAIT:
            {   
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
                break;
            }
            case eState::TAKEOFF:
            {   
                mUAL->takeOff(mHeight);
                mState = eState::WAIT;
                break;
            }
            case eState::LAND:
            {   
                mUAL->land();
                mState = eState::WAIT;
                break;
            }
            case eState::MOVE:
            {   
                grvc::ual::Waypoint waypoint;
                waypoint.header.frame_id = "map";
                waypoint.pose.position.x = mX;
                waypoint.pose.position.y = mY;
                waypoint.pose.position.z = mZ;
                waypoint.pose.orientation.x = 0;
                waypoint.pose.orientation.y = 0;
                waypoint.pose.orientation.z = 0;
                waypoint.pose.orientation.w = 1;
                mUAL->goToWaypoint(waypoint);
                mState = eState::WAIT;
                break;
            }
            case eState::MOVE_VEL:
            {   
                grvc::ual::Velocity velocity;
                velocity.header.frame_id = "map";
                velocity.twist.linear.x = mX;
                velocity.twist.linear.y = mY;
                velocity.twist.linear.z = mZ;
                mUAL->setVelocity(velocity);
                //mState = eState::WAIT; // Al estar comentado seguimos manteniendo la ultima velocidad en caso de que dejemos de enviar
                break;
            }
            case eState::EXIT:
            {
                std::cout << "State EXIT" << std::endl;
                mFin = true;
                mStateThread.join();
                mPoseThread.join();
                mVelThread.join();
                std::cout << "\nEXIT..." << std::endl;
                break;
            }
        }   
    }

    return true;
}
