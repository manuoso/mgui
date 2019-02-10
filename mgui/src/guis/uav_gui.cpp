#include "uav_gui.h"
#include "ui_uav_gui.h"
#include <QTextStream>

#include <iostream>
#include <ros/ros.h>
#include <thread>
#include <chrono>
#include <fstream>

UAV_gui::UAV_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::UAV_gui)
    {

    ui->setupUi(this);

    connect(ui->takeOff, SIGNAL(clicked()), this, SLOT(takeOffClicked()));
    connect(ui->land, SIGNAL(clicked()), this, SLOT(landClicked()));
    connect(ui->Run_pose, SIGNAL(clicked()), this, SLOT(Run_poseClicked()));
    connect(ui->Run_customPose, SIGNAL(clicked()), this, SLOT(Run_customPoseClicked()));

    mMapWidget= new Marble::MarbleWidget();
    mMapWidget->setProjection(Marble::Mercator);
    mMapWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    mMapWidget->show();

    ui->horizontalLayout->addWidget(mMapWidget);

    }

//---------------------------------------------------------------------------------------------------------------------
UAV_gui::~UAV_gui()
{
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool UAV_gui::configureGUI(std::vector<std::pair<std::string, std::string>> _config, int _argcCopy, char ** _argvCopy)
{
    int argc;
    char **argv;
    for( int i = 0; i < _config.size(); i++){
        if( _config[i].first == "idUAV"){
            mIdUAV = _config[i].second;
            ui->lineEdit_ID->setText(QString::fromStdString(mIdUAV));
        }
    }
    
    // TODO: SET ID TO UAV FOR MULTIPLE UAV

    //mUal = new grvc::ual::UAL(_argcCopy, _argvCopy);
    //while (!mUal->isReady() && ros::ok()) {
    //    sleep(1);
    //}
    std::this_thread::sleep_for(std::chrono::seconds(1));

    return true;

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::takeOffClicked()
{
    QString qTakeOff;
    qTakeOff = ui->lineEdit_takeoff->text();
    float takeOff;
    takeOff = qTakeOff.toFloat();
    std::cout << "TakeOff, with height: " << takeOff << std::endl;
    mUal->takeOff(takeOff);
    std::cout << "Finished TakeOff" << std::endl;
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::landClicked()
{
    std::cout << "Landing..." <<std::endl;
    mUal->land(true);
    std::cout << "Landed!" <<std::endl;
}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::Run_poseClicked()
{
    std::cout << "Show pose from UAL" << std::endl;
    mPose = mUal->pose();
    ui->lineEdit_j1->setText(QString::number(mPose.pose.position.x));
    ui->lineEdit_j2->setText(QString::number(mPose.pose.position.y));
    ui->lineEdit_j3->setText(QString::number(mPose.pose.position.z));

}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::Run_customPoseClicked()
{   
    QString qX, qY, qZ;
    qX = ui->lineEdit_a1->text();
    qY = ui->lineEdit_a2->text();
    qZ = ui->lineEdit_a3->text();

    float x, y, z;
    x = qX.toFloat();
    y = qY.toFloat();
    z = qZ.toFloat();

    mPose = mUal->pose();
    std::cout << "You wrote: " << x << ", " << y << ", " << z << std::endl;
    std::cout << "And current pose is: " << mPose.pose.position.x << ", " << mPose.pose.position.y << ", " << mPose.pose.position.z << std::endl;

    std::cout << "Moving..."<< std::endl;
    auto targetPose = mUal->pose();
    targetPose.pose.position.x = x;
    targetPose.pose.position.y = y;
    targetPose.pose.position.z = z;
    mUal->goToWaypoint(targetPose);

}
