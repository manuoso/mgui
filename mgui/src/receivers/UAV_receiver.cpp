/////////////////////////////////////////////////////////////////
//															   //
//                      Header UAV receiver                    //
//															   //
//				 Author: Manuel P. J. (aka. manuoso)		   //
//															   //
/////////////////////////////////////////////////////////////////

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

    std::string ip = mConfigFile["ip"].GetString();
    int portData = mConfigFile["portData"].GetInt();
    int portState = mConfigFile["portState"].GetInt();
    int portPose = mConfigFile["portPose"].GetInt();

    // Initialize Fastcom publishers and subscribers
    mSubsData = new fastcom::Subscriber<command>(ip, portData);
    mPubState = new fastcom::Publisher<std::string>(portState);
    mPubPose = new fastcom::Publisher<pose>(portPose);

    // Callback of received commands
    mSubsData->attachCallback([&](command &_data){
        if(_data.type == "takeoff"){
            mState = eState::TAKEOFF;
            mHeight = _data.height;
        }else if(_data.type == "land"){
            mState = eState::LAND;
        }else if(_data.type == "waypoint"){
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
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
    });

    // Publisher State thread
    mPoseThread = std::thread([&](){
        while(!mFin){
            mSecureLock.lock();
            // Get pose from UAL
            
            mSecureLock.unlock();
            pose sendPose;
            sendPose.x = 0;
            sendPose.y = 0;
            sendPose.z = 0;
            mPubPose->publish(sendPose);
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
