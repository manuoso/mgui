/////////////////////////////////////////////////////////////////
//															   //
//                       Source UAV GUI                        //
//															   //
//				Author: Manuel P. J. (aka. manuoso)			   //
//															   //
/////////////////////////////////////////////////////////////////

#include <mgui/guis/uav_gui.h>
#include <mgui/guis/ui_uav_gui.h>

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
    connect(ui->Add_pose, SIGNAL(clicked()), this, SLOT(add_localPose()));

    connect(ui->Run_customPose, SIGNAL(clicked()), this, SLOT(run_customPose()));
    connect(ui->Add_customPose, SIGNAL(clicked()), this, SLOT(add_customPose()));

    connect(ui->Run_vel, SIGNAL(clicked()), this, SLOT(run_velocity()));
    connect(ui->Stop_vel, SIGNAL(clicked()), this, SLOT(stop_velocity()));

    connect(ui->Run_waypoints, SIGNAL(clicked()), this, SLOT(run_waypoints()));
    connect(ui->DeleteWP, SIGNAL(clicked()), this, SLOT(delete_waypoints()));

    mMapWidget= new Marble::MarbleWidget();
    mMapWidget->setProjection(Marble::Mercator);
    mMapWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    mMapWidget->show();

    ui->horizontalLayout->addWidget(mMapWidget);

    }

//---------------------------------------------------------------------------------------------------------------------
UAV_gui::~UAV_gui(){
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool UAV_gui::configureGUI(int _argc, char **_argv){
    
    

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


}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::land(){



}

//---------------------------------------------------------------------------------------------------------------------
void run_gpsPose(){



}

//---------------------------------------------------------------------------------------------------------------------
void UAV_gui::run_localPose(){

    ui->lineEdit_j1->setText(QString::number( ));
    ui->lineEdit_j2->setText(QString::number( ));
    ui->lineEdit_j3->setText(QString::number( ));

}

//---------------------------------------------------------------------------------------------------------------------
void add_localPose(){



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
void run_velocity(){



}

//---------------------------------------------------------------------------------------------------------------------
void stop_velocity(){



}

//---------------------------------------------------------------------------------------------------------------------
void run_waypoints(){



}

//---------------------------------------------------------------------------------------------------------------------
void delete_waypoints(){



}

