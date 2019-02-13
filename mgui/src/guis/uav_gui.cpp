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
    connect(ui->Stop_gps, SIGNAL(clicked()), this, SLOT(stop_gpsPose()));

    connect(ui->Run_pose, SIGNAL(clicked()), this, SLOT(run_localPose()));
    connect(ui->Stop_pose, SIGNAL(clicked()), this, SLOT(stop_localPose()));

    connect(ui->Run_customPose, SIGNAL(clicked()), this, SLOT(run_customPose()));
    connect(ui->Add_customPose, SIGNAL(clicked()), this, SLOT(add_customPose()));

    connect(ui->Run_vel, SIGNAL(clicked()), this, SLOT(run_velocity()));
    connect(ui->Stop_vel, SIGNAL(clicked()), this, SLOT(stop_velocity()));

    connect(ui->Run_waypoints, SIGNAL(clicked()), this, SLOT(run_waypoints()));
    connect(ui->DeleteWP, SIGNAL(clicked()), this, SLOT(delete_waypoints()));

    connect(this, &UAV_gui::localPositionChanged , this, &UAV_gui::updateLocalPose);
    connect(this, &UAV_gui::stateChanged , this, &UAV_gui::updateState);
    connect(this, &UAV_gui::gpsChanged , this, &UAV_gui::updateGPS);

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

    std::string ip = mConfigFile["ip_sender"].GetString();
    int portCommand = mConfigFile["portCommand"].GetInt();
    int portState = mConfigFile["portState"].GetInt();
    int portPose = mConfigFile["portPose"].GetInt();
    int portGPS = mConfigFile["portGPS"].GetInt();

    // Initialize Fastcom publishers and subscribers
    mPubCommand = new fastcom::Publisher<command>(portCommand);
    mSubsState = new fastcom::Subscriber<std::string>(ip, portState);
    mSubsPose = new fastcom::Subscriber<pose>(ip, portPose);
    mSubsGPS = new fastcom::Subscriber<gps>(ip, portGPS);

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
    
    // Callback of received gps
    mSubsGPS->attachCallback([&](gps &_data){
        mGPSUAV.lat = _data.lat;
        mGPSUAV.lon = _data.lon;
        mGPSUAV.alt = _data.alt;

    });

    mLastTimePose = std::chrono::high_resolution_clock::now();
    mLastTimeGPS = std::chrono::high_resolution_clock::now();

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

    mPrintGPS = true;
    mGPSThread = std::thread([&]{
	while(mPrintGPS){
        	auto t1 = std::chrono::high_resolution_clock::now();
     		if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - mLastTimeGPS).count() > 50){
        		emit gpsChanged(); 
    		}
    
    		mLastTimeGPS = t1;
	}
	
    });

    ui->Run_gps->setVisible(0);
    ui->Stop_gps->setVisible(1); 

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::stop_gpsPose(){

    mPrintGPS = false;
    ui->Run_gps->setVisible(1);
    ui->Stop_gps->setVisible(0);

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
// PRIVATE
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::updateLocalPose(){

    ui->lineEdit_j1->setText(QString::number(mPoseUAV.x));
    ui->lineEdit_j2->setText(QString::number(mPoseUAV.y));
    ui->lineEdit_j3->setText(QString::number(mPoseUAV.z));

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::updateGPS(){

    ui->lineEdit_gps1->setText(QString::number(mGPSUAV.lat));
    ui->lineEdit_gps2->setText(QString::number(mGPSUAV.lon));
    ui->lineEdit_gps3->setText(QString::number(mGPSUAV.alt));

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::updateState(){

    ui->lineEdit_M->setText(QString::fromStdString(mStateUAV));

}


