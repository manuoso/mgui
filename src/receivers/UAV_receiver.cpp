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
    int portGPS = mConfigFile["portGPS"].GetInt();

    // Init UAV controller



    // Initialize Fastcom publishers and subscribers
    mSubsCommand = new fastcom::Subscriber<command>(ip, portCommand);
    mPubState = new fastcom::Publisher<std::string>(portState);
    mPubPose = new fastcom::Publisher<pose>(portPose);
    mPubGPS = new fastcom::Publisher<gps>(portGPS);

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
        }else{
            std::cout << "Unrecognized state" << std::endl;
            mState = eState::WAIT;
        }
    });

    // Publisher State thread
    mStateThread = std::thread([&](){
        while(!mFin){
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
            }
            mPubState->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    // Publisher Pose thread
    mPoseThread = std::thread([&](){
        while(!mFin){
            // Get pose
            
            pose sendPose;
            sendPose.x = 0;
            sendPose.y = 0;
            sendPose.z = 0;
            mPubPose->publish(sendPose);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    });

    // Publisher GPS thread
    mGPSThread = std::thread([&](){
        while(!mFin){
            // Get gps

            gps sendGPS;
            sendGPS.lat = 0;
            sendGPS.lon = 0;
            sendGPS.alt = 0;
            mPubGPS->publish(sendGPS);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    });

    mState = eState::WAIT;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool UAV_receiver::run() {

    while(!mFin){
        switch (mState) {
            case eState::WAIT:
            {   

                std::this_thread::sleep_for(std::chrono::milliseconds(100)); 

                break;
            }
            case eState::TAKEOFF:
            {   

                mState = eState::WAIT;
                break;
            }
            case eState::LAND:
            {   
                
                mState = eState::WAIT;
                break;
            }
            case eState::MOVE:
            {   

                break;
            }
            case eState::MOVE_VEL:
            {   

                break;
            }
            case eState::EXIT:
            {
                std::cout << "State EXIT" << std::endl;
                std::cout << "\nEXIT..." << std::endl;
                mFin = true;
                break;
            }
        }   
    }

    return true;
}
