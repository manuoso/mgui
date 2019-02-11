/////////////////////////////////////////////////////////////////
//															                               //
//                      Source UAV receiver                    //
//															                               //
//				       Author: Manuel P. J. (aka. manuoso)			     //
//															                               //
/////////////////////////////////////////////////////////////////

#include <iostream>
#include <stdio.h>
#include <chrono>
#include <stdlib.h>
#include <mutex>
#include <thread>

#include <rapidjson/document.h>

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

    /// Init 
    /// \param _argc: argc from main
    /// \param _argv: argv from main
    bool init(int _argc, char** _argv);

    /// Run the state machine
    bool run();

  private:
    /// Get telemetry
    bool telemetryThread();

  private:
    eState mState;

    
    dal::Backend::dataTelemetry mDataTelem;
    rapidjson::Document mConfigFile;

    std::mutex mSecureLock;
    std::thread mTelemThread, mSendThread;

    bool mFinishThreadTelem = false;
};
