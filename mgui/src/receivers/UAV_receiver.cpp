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

    int id = mConfigFile["id"].GetInt();
    std::string key = mConfigFile["key"].GetString();
    std::string port = mConfigFile["port"].GetString();
    int baudrate = mConfigFile["baudrate"].GetInt();
	    
    mTelemThread = std::thread(&UAV_receiver::telemetryThread, this);

    mState = eState::WAIT;

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool UAV_receiver::run() {

    bool fin = false;
    while(fin == false && ros::ok()){
        switch (mState) {
            case eState::WAIT:
            {
                
                break;
            }
            case eState::TAKEOFF:
            {   

                
                break;
            }
            case eState::LAND:
            {   
                
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
                std::cout << "State EXIT" << std::endl;
                std::cout << "\nEXIT..." << std::endl;
                mFinishThreadTelem = true;
                mTelemThread.join();
                fin = true;
                break;
        }
    }

    return true;

}

//---------------------------------------------------------------------------------------------------------------------
bool UAV_receiver::telemetryThread() {

    while (mFinishThreadTelem == false && ros::ok()) { 

        
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
    }
    
    return true;
}
