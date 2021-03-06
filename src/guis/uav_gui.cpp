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

    connect(ui->Run_getVel, SIGNAL(clicked()), this, SLOT(run_getVelo()));
    connect(ui->Stop_getVel, SIGNAL(clicked()), this, SLOT(stop_getVelo()));

    connect(ui->Run_pose, SIGNAL(clicked()), this, SLOT(run_localPose()));
    connect(ui->Stop_pose, SIGNAL(clicked()), this, SLOT(stop_localPose()));

    connect(ui->Run_customPose, SIGNAL(clicked()), this, SLOT(run_customPose()));
    connect(ui->Add_customPose, SIGNAL(clicked()), this, SLOT(add_customPose()));

    connect(ui->Run_vel, SIGNAL(clicked()), this, SLOT(run_velocity()));
    connect(ui->Stop_vel, SIGNAL(clicked()), this, SLOT(stop_velocity()));

    connect(ui->Run_waypoints, SIGNAL(clicked()), this, SLOT(run_wayPoints()));
    connect(ui->DeleteWP, SIGNAL(clicked()), this, SLOT(delete_waypoints()));

    connect(ui->Emergency, SIGNAL(clicked()), this, SLOT(emergencyStop()));

    connect(this, &UAV_gui::localPositionChanged , this, &UAV_gui::updateLocalPose);
    connect(this, &UAV_gui::stateChanged , this, &UAV_gui::updateState);
    connect(this, &UAV_gui::velChanged , this, &UAV_gui::updateVel);
    connect(this, &UAV_gui::listWPChanged , this, &UAV_gui::updateListWP);

    scene_ = new QGraphicsScene(this);
    QImage image;
    image.load("src/mgui/logos/GRVC_logo.png");
    scene_->addPixmap(QPixmap::fromImage(image));
    ui->graphicsView_grvc->setScene(scene_);

    mapWidget_= new Marble::MarbleWidget();
    mapWidget_->setProjection(Marble::Mercator);
    mapWidget_->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    mapWidget_->show();

    ui->horizontalLayout->addWidget(mapWidget_);

    ui->Run_pose->setVisible(1);
    ui->Stop_pose->setVisible(0);

    ui->Run_vel->setVisible(1);
    ui->Stop_vel->setVisible(0);

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

    if(configFile_.Parse(json.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    std::string ip = configFile_["ip_sender"].GetString();
    std::string ipWP = configFile_["ip_sender_wp"].GetString();
    int portCommand = configFile_["portCommand"].GetInt();
    int portState = configFile_["portState"].GetInt();
    int portPose = configFile_["portPose"].GetInt();
    int portVel = configFile_["portVel"].GetInt();
    int portCheck = configFile_["portCheck"].GetInt();
    int portWP = configFile_["portWP"].GetInt();

    std::string nameCallbackPose = configFile_["callback_pose"].GetString();
    std::string nameCallbackVel = configFile_["callback_vel"].GetString();
    std::string nameCallbackState = configFile_["callback_state"].GetString();
    std::string nameWPSrv = configFile_["wp_srv"].GetString();
    std::string nameCommandSrv = configFile_["command_srv"].GetString();
    std::string namePositionSrv = configFile_["position_srv"].GetString();
    std::string nameVelocitySrv = configFile_["velocity_srv"].GetString();

    #ifdef MGUI_USE_FASTCOM
        // Initialize Fastcom publishers and subscribers
        pubCommand_ = new fastcom::Publisher<command>(portCommand);
        subsState_ = new fastcom::Subscriber<int>(ip, portState);
        subsPose_ = new fastcom::Subscriber<pose>(ip, portPose);
        subsVel_ = new fastcom::Subscriber<pose>(ip, portVel);
        subsCheck_ = new fastcom::Subscriber<int>(ip, portCheck);
        subsWP_ = new fastcom::Subscriber<pose>(ipWP, portWP);

        // Callback of received commands
        subsState_->attachCallback([&](const int &_data){
            bool dataChanged = false;
            int cpy = _data;
            if(cpy != oldData_){
                if(cpy == 1){
                    objectLockState_.lock();
                    stateUAV_ = "WAIT";
                    objectLockState_.unlock();
                    dataChanged = true;
                }else if(cpy == 2){
                    objectLockState_.lock();
                    stateUAV_ = "TAKEOFF";
                    objectLockState_.unlock();
                    dataChanged = true;
                }else if(cpy == 3){                    
                    objectLockState_.lock();
                    stateUAV_ = "LAND";
                    objectLockState_.unlock();
                    dataChanged = true;
                }else if(cpy == 4){                    
                    objectLockState_.lock();
                    stateUAV_ = "MOVE_POSITION";
                    objectLockState_.unlock();
                    dataChanged = true;
                }else if(cpy == 5){                    
                    objectLockState_.lock();
                    stateUAV_ = "MOVE_VELOCITY";
                    objectLockState_.unlock();
                    dataChanged = true;
                }else if(cpy == 6){                   
                    objectLockState_.lock();
                    stateUAV_ = "EXIT";
                    objectLockState_.unlock();
                    dataChanged = true;
                }else{                   
                    objectLockState_.lock();
                    stateUAV_ = "UNRECOGNIZED";
                    objectLockState_.unlock();
                    dataChanged = true;
                    std::cout << "Received unrecognized state" << std::endl;
                }
                oldData_ = cpy;
            }
            
            if(dataChanged){
                emit stateChanged(); 
            }     
        });

        // Callback of received pose
        subsPose_->attachCallback([&](const pose &_data){
            objectLockPose_.lock();
            poseUAV_.x = _data.x;
            poseUAV_.y = _data.y;
            poseUAV_.z = _data.z;
            objectLockPose_.unlock();
        });
        
        // Callback of received velocity
        subsVel_->attachCallback([&](const pose &_data){
            objectLockVel_.lock();
            velUAV_.x = _data.x;
            velUAV_.y = _data.y;
            velUAV_.z = _data.z;
            objectLockVel_.unlock();
        });

        // Callback of received check
        subsCheck_->attachCallback([&](const int &_data){
            if(_data == 1){
                sendNextWP_ = true;
            }else{
                std::cout << "Received unrecognized check" << std::endl;
                sendNextWP_ = false;
            }
        });

        // Callback of received waypoints
        subsWP_->attachCallback([&](const pose &_data){
            std::cout << "Received WP" << std::endl;
            float x = _data.x;
            float y = _data.y;
            float z = _data.z;

            if((x != 0) && (y != 0) && (z != 0)){
                int id = idWP_;
                idWP_++;
                std::vector<double> point = {x, y, z};
                waypoints_.push_back(std::make_pair(id, point));

                emit listWPChanged();
            }
        });
    #endif

    #ifdef MGUI_USE_ROS
        ros::NodeHandle nh;
        poseSub_ = nh.subscribe(nameCallbackPose, 1, &UAV_gui::CallbackPose, this);
        velSub_ = nh.subscribe(nameCallbackVel, 1, &UAV_gui::CallbackVel, this);
        stateSub_ = nh.subscribe(nameCallbackState, 1, &UAV_gui::CallbackState, this);

        wpSrvRec_ = nh.advertiseService(nameWPSrv, &UAV_gui::CallbackWP, this);  
        commandSrv_ = nh.serviceClient<mgui::CommandData>(nameCommandSrv);
        positionSrv_ = nh.serviceClient<mgui::WaypointData>(namePositionSrv);
        velocitySrv_ = nh.serviceClient<mgui::VelocityData>(nameVelocitySrv);

    #endif

    lastTimePose_ = std::chrono::high_resolution_clock::now();
    lastTimeVel_ = std::chrono::high_resolution_clock::now();
    lastTimeSendVel_ = std::chrono::high_resolution_clock::now();

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
    
    #ifdef MGUI_USE_FASTCOM
        command msg;
        msg.type = 2;
        msg.height = takeOff;
        pubCommand_->publish(msg);
    #endif

    #ifdef MGUI_USE_ROS
        mgui::CommandData srv;
        srv.request.req = true;
        srv.request.type = 2;
        srv.request.height = takeOff;

        if(commandSrv_.call(srv)){
            if(srv.response.success){
                std::cout << "Service of Send Command success" << std::endl;
            }else{
                std::cout << "Service of Send Command failed" << std::endl;
            }
        }else{
            std::cout << "Failed to call service of Send Command" << std::endl;
        }

    #endif

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::land(){

    #ifdef MGUI_USE_FASTCOM
        command msg;
        msg.type = 3;
        pubCommand_->publish(msg);
    #endif

    #ifdef MGUI_USE_ROS
        mgui::CommandData srv;
        srv.request.req = true;
        srv.request.type = 3;

        if(commandSrv_.call(srv)){
            if(srv.response.success){
                std::cout << "Service of Send Command success" << std::endl;
            }else{
                std::cout << "Service of Send Command failed" << std::endl;
            }
        }else{
            std::cout << "Failed to call service of Send Command" << std::endl;
        }

    #endif

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::run_getVelo(){

    printVel_ = true;
    getVelThread_ = new std::thread([&]{
        while(printVel_ && !stopAll_){
            auto t1 = std::chrono::high_resolution_clock::now();
            if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - lastTimeVel_).count() > 50){
                lastTimeVel_ = t1;
                emit velChanged(); 
            }		
        }
    });

    ui->Run_getVel->setVisible(0);
    ui->Stop_getVel->setVisible(1); 

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::stop_getVelo(){

    printVel_ = false;
    getVelThread_->join();
    delete getVelThread_;
    ui->Run_getVel->setVisible(1);
    ui->Stop_getVel->setVisible(0);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::run_localPose(){

    printLocalPose_ = true;
    localPoseThread_ = new std::thread([&]{
        while(printLocalPose_ && !stopAll_){
            auto t1 = std::chrono::high_resolution_clock::now();
            if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - lastTimePose_).count() > 50){
                lastTimePose_ = t1;
                emit localPositionChanged(); 
            }
        }
    });

    ui->Run_pose->setVisible(0);
    ui->Stop_pose->setVisible(1);   

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::stop_localPose(){

    printLocalPose_ = false;
    localPoseThread_->join();
    delete localPoseThread_;
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

    #ifdef MGUI_USE_FASTCOM
        command msg;
        msg.type = 4;
        msg.x = x;
        msg.y = y;
        msg.z = z;
        if(!sendVelocity_){
            pubCommand_->publish(msg);
        }else{
            std::cout << "Cant send position while you are sending velocity" << std::endl;
        }
    #endif

    #ifdef MGUI_USE_ROS
        mgui::WaypointData srv;
        srv.request.req = true;

        srv.request.poseWP.header.stamp = ros::Time::now();
        srv.request.poseWP.header.frame_id = "map";
        srv.request.poseWP.pose.position.x = x;
        srv.request.poseWP.pose.position.y = y;
        srv.request.poseWP.pose.position.z = z;  
        srv.request.poseWP.pose.orientation.x = 0; 
        srv.request.poseWP.pose.orientation.y = 0; 
        srv.request.poseWP.pose.orientation.z = 0;  
        srv.request.poseWP.pose.orientation.w = 1; 

        if(positionSrv_.call(srv)){
            if(srv.response.success){
                std::cout << "Service of Send Command success" << std::endl;
            }else{
                std::cout << "Service of Send Command failed" << std::endl;
            }
        }else{
            std::cout << "Failed to call service of Send Command" << std::endl;
        }

    #endif

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

    int id = idWP_;
    idWP_++;
    std::vector<double> point = {x, y, z};
    waypoints_.push_back(std::make_pair(id, point));

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

    sendVelUAV_.x = x; 
    sendVelUAV_.y = y;
    sendVelUAV_.z = z;

    sendVelocity_ = true;

    #ifdef MGUI_USE_FASTCOM
    velocityThread_ = new std::thread([&]{
        while(sendVelocity_ && !stopAll_){
            command msg;
            msg.type = 5;
            msg.x = sendVelUAV_.x;
            msg.y = sendVelUAV_.y;
            msg.z = sendVelUAV_.z;
            auto t1 = std::chrono::high_resolution_clock::now();
            if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - lastTimeSendVel_).count() > 50){
                lastTimeSendVel_ = t1;
                pubCommand_->publish(msg);
            }
        }
    });
    #endif

    #ifdef MGUI_USE_ROS
        mgui::VelocityData srv;
        srv.request.req = true;
        srv.request.continually = true;

        srv.request.velocity.twist.linear.x = sendVelUAV_.x;
        srv.request.velocity.twist.linear.y = sendVelUAV_.y;
        srv.request.velocity.twist.linear.z = sendVelUAV_.z;

        if(velocitySrv_.call(srv)){
            if(srv.response.success){
                std::cout << "Service of Send Velocity success" << std::endl;
            }else{
                std::cout << "Service of Send Velocity failed" << std::endl;
            }
        }else{
            std::cout << "Failed to call service of Send Velocity" << std::endl;
        }

    #endif
    
    ui->Run_vel->setVisible(0);
    ui->Stop_vel->setVisible(1);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::stop_velocity(){

    sendVelocity_ = false;

    #ifdef MGUI_USE_FASTCOM
        velocityThread_->join();
        delete velocityThread_;
    #endif

    #ifdef MGUI_USE_ROS
        mgui::VelocityData srv;
        srv.request.req = true;
        srv.request.continually = false;

        srv.request.velocity.twist.linear.x = 0;
        srv.request.velocity.twist.linear.y = 0;
        srv.request.velocity.twist.linear.z = 0;

        if(velocitySrv_.call(srv)){
            if(srv.response.success){
                std::cout << "Service of Send Velocity success" << std::endl;
            }else{
                std::cout << "Service of Send Velocity failed" << std::endl;
            }
        }else{
            std::cout << "Failed to call service of Send Velocity" << std::endl;
        }

    #endif

    ui->Run_vel->setVisible(1);
    ui->Stop_vel->setVisible(0);

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::run_wayPoints(){

    sendNextWP_ = false;

    sendWPThread_ = std::thread([&](){
        // for(int i = waypoints_.size()-1 ; i > -1; i--){ // SE LEEN AL REVES! BE CAREFULLY 
        for(int i = 0 ; i < waypoints_.size(); i++){
            if(stopAll_){
                break;
            }

            #ifdef MGUI_USE_FASTCOM
                command msg;
                msg.type = 4;
                msg.x = waypoints_[i].second[0];
                msg.y = waypoints_[i].second[1];
                msg.z = waypoints_[i].second[2];
                if(!sendVelocity_){
                    pubCommand_->publish(msg);
                }else{
                    std::cout << "Cant send position while you are sending velocity" << std::endl;
                }
                while(!sendNextWP_ && !stopAll_){
                    std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
                }
                sendNextWP_ = false;
            #endif
            
            #ifdef MGUI_USE_ROS
                mgui::WaypointData srv;
                srv.request.req = true;

                srv.request.poseWP.header.stamp = ros::Time::now();
                srv.request.poseWP.header.frame_id = "map";
                srv.request.poseWP.pose.position.x = waypoints_[i].second[0];
                srv.request.poseWP.pose.position.y = waypoints_[i].second[1];
                srv.request.poseWP.pose.position.z = waypoints_[i].second[2];  
                srv.request.poseWP.pose.orientation.x = 0; 
                srv.request.poseWP.pose.orientation.y = 0; 
                srv.request.poseWP.pose.orientation.z = 0;  
                srv.request.poseWP.pose.orientation.w = 1; 

                if(positionSrv_.call(srv)){
                    if(srv.response.success){
                        std::cout << "Service of Send Position success" << std::endl;
                    }else{
                        std::cout << "Service of Send Position failed" << std::endl;
                    }
                }else{
                    std::cout << "Failed to call service of Send Position" << std::endl;
                }

            #endif

        }
    });
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::delete_waypoints(){

    QList<QListWidgetItem*> items = ui->listWidget_WayPoints->selectedItems();
    foreach(QListWidgetItem * item, items){
        int index = ui->listWidget_WayPoints->row(item);
        //std::cout << "index:" << index << std::endl;
        waypoints_.erase(waypoints_.begin() + index);
        delete ui->listWidget_WayPoints->takeItem(ui->listWidget_WayPoints->row(item));
    }

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::emergencyStop(){

    stopAll_ = true;

    #ifdef MGUI_USE_FASTCOM
        command msg;
        msg.type = 6;
        sendVelocity_ = false;
        pubCommand_->publish(msg);
    #endif

    #ifdef MGUI_USE_ROS
        mgui::CommandData srv;
        srv.request.req = true;
        srv.request.type = 6;

        if(commandSrv_.call(srv)){
            if(srv.response.success){
                std::cout << "Service of Send Command success" << std::endl;
            }else{
                std::cout << "Service of Send Command failed" << std::endl;
            }
        }else{
            std::cout << "Failed to call service of Send Command" << std::endl;
        }

    #endif

}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::updateLocalPose(){
    objectLockPose_.lock();
    ui->lineEdit_j1->setText(QString::number(poseUAV_.x));
    ui->lineEdit_j2->setText(QString::number(poseUAV_.y));
    ui->lineEdit_j3->setText(QString::number(poseUAV_.z));
    objectLockPose_.unlock();
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::updateVel(){
    objectLockVel_.lock();
    ui->lineEdit_v1->setText(QString::number(velUAV_.x));
    ui->lineEdit_v2->setText(QString::number(velUAV_.y));
    ui->lineEdit_v3->setText(QString::number(velUAV_.z));
    objectLockVel_.unlock();
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::updateState(){
    objectLockState_.lock();
    ui->lineEdit_M->setText(QString::fromStdString(stateUAV_));
    objectLockState_.unlock();
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::updateListWP(){
    //objectLock_.lock();
    int id = idWP_-1;
    float x = waypoints_[id].second[0];
    float y = waypoints_[id].second[1];
    float z = waypoints_[id].second[2];
    std::string swaypoint = "ID: " + std::to_string(id) + " , " + "X: " + std::to_string(x) + " , " +  "Y: " + std::to_string(y) + " , " + "Z: " + std::to_string(z);
    ui->listWidget_WayPoints->addItem(QString::fromStdString(swaypoint));
    //objectLock_.unlock();
}

//---------------------------------------------------------------------------------------------------------------------
#ifdef MGUI_USE_ROS
    void UAV_gui::CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& _msg){
        objectLockPose_.lock();
        poseUAV_.x = _msg->pose.position.x;
        poseUAV_.y = _msg->pose.position.y;
        poseUAV_.z = _msg->pose.position.z;
        objectLockPose_.unlock();
        
    }

    //---------------------------------------------------------------------------------------------------------------------
    void UAV_gui::CallbackVel(const geometry_msgs::TwistStamped::ConstPtr& _msg){
        objectLockVel_.lock();
        velUAV_.x = _msg->twist.linear.x;
        velUAV_.y = _msg->twist.linear.y;
        velUAV_.z = _msg->twist.linear.z;
        objectLockVel_.unlock();

    }

    //---------------------------------------------------------------------------------------------------------------------
    void UAV_gui::CallbackState(const std_msgs::UInt8::ConstPtr& _msg){
        bool dataChanged = false;
        unsigned int cpy = _msg->data;
        if(cpy != oldData_){
            if(cpy == 1){
                objectLockState_.lock();
                stateUAV_ = "WAIT";
                objectLockState_.unlock();
                dataChanged = true;
            }else if(cpy == 2){
                objectLockState_.lock();
                stateUAV_ = "TAKEOFF";
                objectLockState_.unlock();
                dataChanged = true;
            }else if(cpy == 3){                    
                objectLockState_.lock();
                stateUAV_ = "LAND";
                objectLockState_.unlock();
                dataChanged = true;
            }else if(cpy == 4){                    
                objectLockState_.lock();
                stateUAV_ = "MOVE_POSITION";
                objectLockState_.unlock();
                dataChanged = true;
            }else if(cpy == 5){                    
                objectLockState_.lock();
                stateUAV_ = "MOVE_VELOCITY";
                objectLockState_.unlock();
                dataChanged = true;
            }else if(cpy == 6){                   
                objectLockState_.lock();
                stateUAV_ = "EXIT";
                objectLockState_.unlock();
                dataChanged = true;
            }else{                   
                objectLockState_.lock();
                stateUAV_ = "UNRECOGNIZED";
                objectLockState_.unlock();
                dataChanged = true;
                std::cout << "Received unrecognized state" << std::endl;
            }
            oldData_ = cpy;
        }
        
        if(dataChanged){
            emit stateChanged(); 
        } 

    }
    
    //---------------------------------------------------------------------------------------------------------------------
    bool UAV_gui::CallbackWP(mgui::WaypointData::Request &_req, mgui::WaypointData::Response &_res){

        if(_req.req){
            std::cout << "Received WP" << std::endl;
            float x = _req.poseWP.pose.position.x;
            float y = _req.poseWP.pose.position.y;
            float z = _req.poseWP.pose.position.z;

            if((x != 0) && (y != 0) && (z != 0)){
                int id = idWP_;
                idWP_++;
                std::vector<double> point = {x, y, z};
                waypoints_.push_back(std::make_pair(id, point));

                emit listWPChanged();
            }
        }
        _res.success = true;

        return true;
    }

#endif
