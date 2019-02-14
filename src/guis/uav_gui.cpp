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

#include <mgui/guis/uav_gui.h>
#include <guis/ui_uav_gui.h>

//---------------------------------------------------------------------------------------------------------------------
// PUBLIC 
//---------------------------------------------------------------------------------------------------------------------

UAV_gui::UAV_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::UAV_gui)
    {

    ui->setupUi(this);

    connect(ui->takeOff, SIGNAL(clicked()), this, SLOT(takeOff()));
    connect(ui->land, SIGNAL(clicked()), this, SLOT(land()));

    connect(ui->Run_getVel, SIGNAL(clicked()), this, SLOT(run_getVel()));
    connect(ui->Stop_getVel, SIGNAL(clicked()), this, SLOT(stop_getVel()));

    connect(ui->Run_pose, SIGNAL(clicked()), this, SLOT(run_localPose()));
    connect(ui->Stop_pose, SIGNAL(clicked()), this, SLOT(stop_localPose()));

    connect(ui->Run_customPose, SIGNAL(clicked()), this, SLOT(run_customPose()));
    connect(ui->Add_customPose, SIGNAL(clicked()), this, SLOT(add_customPose()));

    connect(ui->Run_vel, SIGNAL(clicked()), this, SLOT(run_velocity()));
    connect(ui->Stop_vel, SIGNAL(clicked()), this, SLOT(stop_velocity()));

    connect(ui->Run_waypoints, SIGNAL(clicked()), this, SLOT(run_waypoints()));
    connect(ui->DeleteWP, SIGNAL(clicked()), this, SLOT(delete_waypoints()));

    connect(ui->Emergency, SIGNAL(clicked()), this, SLOT(emergency()));

    connect(this, &UAV_gui::localPositionChanged , this, &UAV_gui::updateLocalPose);
    connect(this, &UAV_gui::stateChanged , this, &UAV_gui::updateState);
    connect(this, &UAV_gui::velChanged , this, &UAV_gui::updateVel);

    mMapWidget= new Marble::MarbleWidget();
    mMapWidget->setProjection(Marble::Mercator);
    mMapWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    mMapWidget->show();

    ui->horizontalLayout->addWidget(mMapWidget);

    ui->Run_pose->setVisible(1);
    ui->Stop_pose->setVisible(0);

    ui->Run_getVel->setVisible(1);
    ui->Stop_getVel->setVisible(0);

    }

//---------------------------------------------------------------------------------------------------------------------
UAV_gui::~UAV_gui(){
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool UAV_gui::configureGUI(int _argc, char **_argv){
    
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

    std::string ip = mConfigFile["ip_sender"].GetString();
    int portCommand = mConfigFile["portCommand"].GetInt();
    int portState = mConfigFile["portState"].GetInt();
    int portPose = mConfigFile["portPose"].GetInt();
    int portVel = mConfigFile["portVel"].GetInt();

    // Initialize Fastcom publishers and subscribers
    mPubCommand = new fastcom::Publisher<command>(portCommand);
    mSubsState = new fastcom::Subscriber<std::string>(ip, portState);
    mSubsPose = new fastcom::Subscriber<pose>(ip, portPose);
    mSubsVel = new fastcom::Subscriber<pose>(ip, portVel);

    // Callback of received commands
    mSubsState->attachCallback([&](std::string &_data){
        mStateUAV = _data;
        if(mStateUAV != _data){
            emit stateChanged(); 
        }     

    });

    // Callback of received pose
    mSubsPose->attachCallback([&](pose &_data){
        mPoseUAV.x = _data.x;
        mPoseUAV.y = _data.y;
        mPoseUAV.z = _data.z;

    });
    
    // Callback of received velocity
    mSubsVel->attachCallback([&](pose &_data){
        mVelUAV.x = _data.x;
        mVelUAV.y = _data.y;
        mVelUAV.z = _data.z;

    });

    mLastTimePose = std::chrono::high_resolution_clock::now();
    mLastTimeVel = std::chrono::high_resolution_clock::now();
    mLastTimeSendVel = std::chrono::high_resolution_clock::now();

    std::this_thread::sleep_for(std::chrono::seconds(1));

    return true;

}

//---------------------------------------------------------------------------------------------------------------------
// SLOTS
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::takeOff(){

    QString qTakeOff;
    qTakeOff = ui->lineEdit_takeoff->text();
    float takeOff;
    takeOff = qTakeOff.toFloat();

    command msg;
    msg.type = "takeoff";
    msg.height = takeOff;
    mPubCommand->publish(msg);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::land(){

    command msg;
    msg.type = "land";
    mPubCommand->publish(msg);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::run_getVel(){

    mPrintVel = true;
    mGetVelThread = std::thread([&]{
	while(mPrintVel){
        	auto t1 = std::chrono::high_resolution_clock::now();
     		if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - mLastTimeVel).count() > 50){
        		emit velChanged(); 
    		}
    
    		mLastTimeVel = t1;
	}
	
    });

    ui->Run_getVel->setVisible(0);
    ui->Stop_getVel->setVisible(1); 

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::stop_getVel(){

    mPrintVel = false;
    ui->Run_getVel->setVisible(1);
    ui->Stop_getVel->setVisible(0);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::run_localPose(){

    mPrintLocalPose = true;
    mLocalPoseThread = std::thread([&]{
	while(mPrintLocalPose){
        	auto t1 = std::chrono::high_resolution_clock::now();
     		if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - mLastTimePose).count() > 50){
        		emit localPositionChanged(); 
    		}
    
    		mLastTimePose = t1;
	}
	
    });

    ui->Run_pose->setVisible(0);
    ui->Stop_pose->setVisible(1);   

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::stop_localPose(){

    mPrintLocalPose = false;
    ui->Run_pose->setVisible(1);
    ui->Stop_pose->setVisible(0);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::run_customPose()
{   
    QString qX, qY, qZ;
    qX = ui->lineEdit_a1->text();
    qY = ui->lineEdit_a2->text();
    qZ = ui->lineEdit_a3->text();

    float x, y, z;
    x = qX.toFloat();
    y = qY.toFloat();
    z = qZ.toFloat();

    command msg;
    msg.type = "position";
    msg.x = x;
    msg.y = y;
    msg.z = z;
    if(!mSendVelocity){
        mPubCommand->publish(msg);
        }else{
        std::cout << "Cant send position while you are sending velocity" << std::endl;
    }

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::add_customPose()
{   
    QString qX, qY, qZ;
    qX = ui->lineEdit_a1->text();
    qY = ui->lineEdit_a2->text();
    qZ = ui->lineEdit_a3->text();

    float x, y, z;
    x = qX.toFloat();
    y = qY.toFloat();
    z = qZ.toFloat();

    int id = mIDWP;
    mIDWP++;
    std::vector<double> point = {x, y, z};
    mWayPoints.push_back(std::make_pair(id, point));

    std::string swaypoint = "ID: " + std::to_string(id) + " , " + "X: " + std::to_string(x) + " , " +  "Y: " + std::to_string(y) + " , " + "Z: " + std::to_string(z);
    
    ui->listWidget_WayPoints->addItem(QString::fromStdString(swaypoint));

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::run_velocity(){
	
    QString qX, qY, qZ;
    qX = ui->lineEdit_vel1->text();
    qY = ui->lineEdit_vel2->text();
    qZ = ui->lineEdit_vel3->text();

    float x, y, z;
    x = qX.toFloat();
    y = qY.toFloat();
    z = qZ.toFloat();

    mSendVelUAV.x = x; 
    mSendVelUAV.y = y;
    mSendVelUAV.z = z;

    mSendVelocity = true;
    mVelocityThread = std::thread([&]{
        while(mSendVelocity){
            command msg;
            msg.type = "velocity";
            msg.x = mSendVelUAV.x;
            msg.y = mSendVelUAV.y;
            msg.z = mSendVelUAV.z;
            auto t1 = std::chrono::high_resolution_clock::now();
     		if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - mLastTimeSendVel).count() > 50){
        		mPubCommand->publish(msg);
    		}
    
    		mLastTimeSendVel = t1;
        }
    });
    
    ui->Run_vel->setVisible(1);
    ui->Stop_vel->setVisible(0);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::stop_velocity(){

    mSendVelocity = false;
    ui->Run_vel->setVisible(0);
    ui->Stop_vel->setVisible(1);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::run_waypoints(){

    for(unsigned i = 0; i < mWayPoints.size(); i++){
        command msg;
        msg.type = "position";
        msg.x = mWayPoints[i].second[0];
        msg.y = mWayPoints[i].second[1];
        msg.z = mWayPoints[i].second[2];
        if(!mSendVelocity){
            mPubCommand->publish(msg);
            }else{
            std::cout << "Cant send position while you are sending velocity" << std::endl;
        }

        // 666 TODO: ESPERAR CONFIRMACIÓN ANTES DE ENVIAR EL SIGUIENTE WP?
    }

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::delete_waypoints(){

    QList<QListWidgetItem*> items = ui->listWidget_WayPoints->selectedItems();
    foreach(QListWidgetItem * item, items){

        int index = ui->listWidget_WayPoints->row(item);
        mWayPoints.erase(mWayPoints.begin() + index);
        delete ui->listWidget_WayPoints->takeItem(ui->listWidget_WayPoints->row(item));
    }

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::emergency(){

    command msg;
    msg.type = "emergency";
    mSendVelocity = false;
    mPubCommand->publish(msg);

}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::updateLocalPose(){

    ui->lineEdit_j1->setText(QString::number(mPoseUAV.x));
    ui->lineEdit_j2->setText(QString::number(mPoseUAV.y));
    ui->lineEdit_j3->setText(QString::number(mPoseUAV.z));

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::updateVel(){

    ui->lineEdit_v1->setText(QString::number(mVelUAV.x));
    ui->lineEdit_v2->setText(QString::number(mVelUAV.y));
    ui->lineEdit_v3->setText(QString::number(mVelUAV.z));

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::updateState(){

    ui->lineEdit_M->setText(QString::fromStdString(mStateUAV));

}


