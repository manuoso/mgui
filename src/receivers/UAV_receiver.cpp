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

    if(configFile_.Parse(json.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    std::string ip = configFile_["ip_receiver"].GetString();
    int portCommand = configFile_["portCommand"].GetInt();
    int portState = configFile_["portState"].GetInt();
    int portPose = configFile_["portPose"].GetInt();
    int portVel = configFile_["portVel"].GetInt();

    // Init UAV controller
    ual_ = new grvc::ual::UAL(_argc, _argv);

    // Initialize Fastcom publishers and subscribers
    subsCommand_ = new fastcom::Subscriber<command>(ip, portCommand);
    pubState_ = new fastcom::Publisher<std::string>(portState);
    pubPose_ = new fastcom::Publisher<pose>(portPose);
    pubVel_ = new fastcom::Publisher<pose>(portVel);

    // Callback of received commands
    subsCommand_->attachCallback([&](command &_data){
        if(_data.type == "takeoff"){
            state_ = eState::TAKEOFF;
            height_ = _data.height;
        }else if(_data.type == "land"){
            state_ = eState::LAND;
        }else if(_data.type == "position"){
            state_ = eState::MOVE;
            x_ = _data.x;
            y_ = _data.y;
            z_ = _data.z;
        }else if(_data.type == "velocity"){
            state_ = eState::MOVE_VEL;
            x_ = _data.x;
            y_ = _data.y;
            z_ = _data.z;
        }else if(_data.type == "wait"){
            state_ = eState::WAIT;
        }else if(_data.type == "emergency"){
            state_ = eState::EXIT;
        }else{
            std::cout << "Unrecognized state" << std::endl;
            state_ = eState::WAIT;
        }
    });

    // Publisher State thread
    stateThread_ = std::thread([&](){
        while(!fin_ && ros::ok()){
            std::string msg;
            switch(state_){
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
            pubState_->publish(msg);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    });

    // Publisher Pose thread
    poseThread_ = std::thread([&](){
        while(!fin_ && ros::ok()){
            // Get pose
            grvc::ual::Pose p = ual_->pose();

            pose sendPose;
            sendPose.x = p.pose.position.x;
            sendPose.y = p.pose.position.y;
            sendPose.z = p.pose.position.z;
            pubPose_->publish(sendPose);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    });

    // Publisher Velocity thread
    velThread_ = std::thread([&](){
        while(!fin_ && ros::ok()){
            // Get velocity
            grvc::ual::Velocity v = ual_->velocity();

            pose sendVel;
            sendVel.x = v.twist.linear.x;
            sendVel.y = v.twist.linear.y;
            sendVel.z = v.twist.linear.z;
            pubVel_->publish(sendVel);
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    });

    state_ = eState::WAIT;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool UAV_receiver::run() {

    while(!fin_ && ros::ok()){
        switch (state_) {
            case eState::WAIT:
            {   
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
                break;
            }
            case eState::TAKEOFF:
            {   
                ual_->takeOff(height_);
                state_ = eState::WAIT;
                break;
            }
            case eState::LAND:
            {   
                ual_->land();
                state_ = eState::WAIT;
                break;
            }
            case eState::MOVE:
            {   
                grvc::ual::Waypoint waypoint;
                waypoint.header.frame_id = "map";
                waypoint.pose.position.x = x_;
                waypoint.pose.position.y = y_;
                waypoint.pose.position.z = z_;
                waypoint.pose.orientation.x = 0;
                waypoint.pose.orientation.y = 0;
                waypoint.pose.orientation.z = 0;
                waypoint.pose.orientation.w = 1;
                ual_->goToWaypoint(waypoint);
                state_ = eState::WAIT;
                break;
            }
            case eState::MOVE_VEL:
            {   
                grvc::ual::Velocity velocity;
                velocity.header.frame_id = "map";
                velocity.twist.linear.x = x_;
                velocity.twist.linear.y = y_;
                velocity.twist.linear.z = z_;
                ual_->setVelocity(velocity);
                //state_ = eState::WAIT; // Al estar comentado seguimos manteniendo la ultima velocidad en caso de que dejemos de enviar
                break;
            }
            case eState::EXIT:
            {
                std::cout << "State EXIT" << std::endl;
                fin_ = true;
                stateThread_.join();
                poseThread_.join();
                velThread_.join();
                std::cout << "\nEXIT..." << std::endl;
                break;
            }
        }   
    }

    return true;
}
