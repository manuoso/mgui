/////////////////////////////////////////////////////////////////
//															   //
//                       Source UAV GUI                        //
//															   //
//				Author: Manuel P. J. (aka. manuoso)			   //
//															   //
/////////////////////////////////////////////////////////////////

#include <mgui/guis/uav_gui.h>
#include <mgui/ui_uav_gui.h>

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

    connect(ui->Run_gps, SIGNAL(clicked()), this, SLOT(run_gpsPose()));

    connect(ui->Run_pose, SIGNAL(clicked()), this, SLOT(run_localPose()));
    connect(ui->Stop_pose, SIGNAL(clicked()), this, SLOT(stop_localPose()));

    connect(ui->Run_customPose, SIGNAL(clicked()), this, SLOT(run_customPose()));
    connect(ui->Add_customPose, SIGNAL(clicked()), this, SLOT(add_customPose()));

    connect(ui->Run_vel, SIGNAL(clicked()), this, SLOT(run_velocity()));
    connect(ui->Stop_vel, SIGNAL(clicked()), this, SLOT(stop_velocity()));

    connect(ui->Run_waypoints, SIGNAL(clicked()), this, SLOT(run_waypoints()));
    connect(ui->DeleteWP, SIGNAL(clicked()), this, SLOT(delete_waypoints()));

    connect(this, &UAV_gui::localPositionchanged , this, &UAV_gui::updateLocalPose);
    connect(this, &UAV_gui::stateChanged , this, &UAV_gui::updateState);

    mMapWidget= new Marble::MarbleWidget();
    mMapWidget->setProjection(Marble::Mercator);
    mMapWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    mMapWidget->show();

    ui->horizontalLayout->addWidget(mMapWidget);

    ui->Run_pose->setVisible(1);
    ui->Stop_pose->setVisible(0);

    ui->Run_vel->setVisible(1);
    ui->Stop_vel->setVisible(0);

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

    std::string ip = mConfigFile["ip"].GetString();
    int portCommand = mConfigFile["portCommand"].GetInt();
    int portState = mConfigFile["portState"].GetInt();
    int portPose = mConfigFile["portPose"].GetInt();

    // Initialize Fastcom publishers and subscribers
    mPubCommand = new fastcom::Publisher<command>(portCommand);
    mSubsState = new fastcom::Subscriber<std::string>(ip, portState);
    mSubsPose = new fastcom::Subscriber<pose>(ip, portPose);

    // Callback of received commands
    mSubsState->attachCallback([&](std::string &_data){
	mStateUAV = _data;
	if(mState != _data){
		emit stateChanged(); 
	}     

    });

    // Callback of received commands
    mSubsPose->attachCallback([&](pose &_data){
        mPoseUAV.x = _data.x;
	mPoseUAV.y = _data.y;
	mPoseUAV.z = _data.z;

    });

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
void UAV_gui::run_gpsPose(){



}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::run_localPose(){

    mPrintLocalPose = true;
    mLocalPoseThread = std::thread([&]{
	while(mPrintLocalPose){
        	auto t1 = std::chrono::high_resolution_clock::now();
     		if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - mLastTimePose).count() > 50){
        		emit localPositionchanged(); 
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
    msg.type = "waypoint";
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

    mSendVelocity = true;
    mVelocityThread = std::thread([&]{

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



}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::delete_waypoints(){



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
void UAV_gui::updateLocalPose(){

    ui->lineEdit_M->setText(QString::string(mStateUAV));

}


