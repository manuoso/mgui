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

    struct gps{
        float lat;
        float lon;
        float alt;
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

    fastcom::Subscriber<command> *mSubsCommand;
    fastcom::Publisher<std::string> *mPubState;
    fastcom::Publisher<pose> *mPubPose;
    fastcom::Publisher<gps> *mPubGPS;

    float mHeight, mX, mY, mZ; 

    std::thread mStateThread, mPoseThread, mGPSThread;
    std::mutex mSecureLock;

    bool mFin = false;
};
