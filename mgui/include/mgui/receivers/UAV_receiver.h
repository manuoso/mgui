/////////////////////////////////////////////////////////////////
//															                               //
//                      Source UAV receiver                    //
//															                               //
//				       Author: Manuel P. J. (aka. manuoso)			     //
//															                               //
/////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <chrono>
#include <stdlib.h>
#include <mutex>
#include <thread>

#include <rapidjson/document.h>

#include <fastcom/Subscriber.h>
#include <fastcom/Publisher.h>

class UAV_receiver
{
  public:
    /// States of the state machine
    enum class eState
    {   
        WAIT,
        TAKEOFF,
        LAND,
        MOVE,
        MOVE_VEL,
        EXIT
    };

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

    /// Init 
    /// \param _argc: argc from main
    /// \param _argv: argv from main
    bool init(int _argc, char** _argv);

    /// Run the state machine
    bool run();

  private:
    eState mState;

    rapidjson::Document mConfigFile;

    fastcom::Subscriber<command> *mSubsData;
    fastcom::Publisher<std::string> *mPubState;
    fastcom::Publisher<pose> *mPubPose;

    float mHeight, mX, mY, mZ; 

    std::thread mStateThread, mPoseThread;
    std::mutex mSecureLock;

    bool mFin = false;
};
