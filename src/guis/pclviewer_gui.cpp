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

#include <mgui/guis/pclviewer_gui.h>
#include <guis/ui_pclviewer_gui.h>

//---------------------------------------------------------------------------------------------------------------------
// PUBLIC 
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
PCLViewer_gui::PCLViewer_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLViewer_gui)
    {

    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");

    connect(ui->DeleteSphere, SIGNAL(clicked()), this, SLOT(deleteSphere()));
    connect(ui->Run_gtray, SIGNAL(clicked()), this, SLOT(run_generateTray()));
    connect(ui->Run_sendM, SIGNAL(clicked()), this, SLOT(run_sendMision()));
    connect(ui->AddWP, SIGNAL(clicked()), this, SLOT(addWaypoint()));

    connect(this, &PCLViewer_gui::poseUAVchanged , this, &PCLViewer_gui::updateObjectUAV);
    
    }

//---------------------------------------------------------------------------------------------------------------------
PCLViewer_gui::~PCLViewer_gui()
{
    delete ui;
}

//---------------------------------------------------------------------------------------------------------------------
bool PCLViewer_gui::configureGUI(int _argc, char **_argv)
{
    std::string type;
    std::string dir = "";

    std::ifstream rawFile(_argv[1]);
    if (!rawFile.is_open()) {
        std::cout << "Error opening config file" << std::endl;
        return false;
    }

    std::stringstream strStream;
    strStream << rawFile.rdbuf(); 
    std::string json = strStream.str(); 

    if(mConfigFile.Parse(json.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    mConvertAndSave = mConfigFile["save_pcd_ply"].GetBool();

    mUsePCD = mConfigFile["use_pcd"].GetBool();
    mDirPCD = mConfigFile["dir_pcd"].GetString();

    mUsePLY = mConfigFile["use_ply"].GetBool();
    mDirPLY = mConfigFile["dir_ply"].GetString();

    mUseTXT = mConfigFile["use_txt"].GetBool();
    mDirTXT = mConfigFile["dir_txt"].GetString();
    
    mTypePoint = mConfigFile["type_point"].GetString();

    mTypeCallbackPose = mConfigFile["type_callback"].GetString();
    mNameCallbackPose = mConfigFile["callback_name"].GetString();
    mPortCallbackPose = mConfigFile["callback_port"].GetInt();

    mTypeModelPose = mConfigFile["type_model_pose"].GetString();
    mPathModelPose = mConfigFile["model_pose"].GetString();

    if(mUsePCD && !mUsePLY && !mUseTXT){
        extractPointCloud(mDirPCD);
    }else if(!mUsePCD && mUsePLY && !mUseTXT){
        extractPointCloud(mDirPLY);
    }else if(!mUsePCD && !mUsePLY && mUseTXT){
        extractPointCloud(mDirTXT);
    }else{
        std::cout << "ERROR! You use more than one dir" << std::endl;
        return false;
    }

    if(mTypeCallbackPose == "fastcom"){


    }else if(mTypeCallbackPose == "ros"){


    }else{

    }

    mLastTimePose = std::chrono::high_resolution_clock::now();
    
    return true;
}

//---------------------------------------------------------------------------------------------------------------------
// SLOTS
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::addWaypoint(){
    
    QString qx, qy, qz;
    qx = ui->lineEdit_MX->text();
    qy = ui->lineEdit_MY->text();
    qz = ui->lineEdit_MZ->text();

    double x, y, z, xq, yq, zq, wq;
    x = qx.toDouble();
    y = qy.toDouble();
    z = qz.toDouble();
    xq = 0.0;
    yq = 0.0;
    zq = 0.0;
    wq = 1.0;

    int id = mContSpheres;

    std::vector<double> point = {x, y, z, xq, yq, zq, wq};
    mWayPoints.push_back(std::make_pair(id, point));

    std::string swaypoint = "ID: " + std::to_string(id) + " , " + "X: " + std::to_string(x) + " , " +  "Y: " + std::to_string(y) + " , " + "Z: " + std::to_string(z) + " , " + "QX: " + std::to_string(xq) + " , " + "QY: " + std::to_string(yq) + " , " + "QZ: " + std::to_string(zq) + " , " + "QW: " + std::to_string(wq);
    
    ui->listWidget_WayPoints->addItem(QString::fromStdString(swaypoint));
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::run_generateTray(){

    // motion_planner

}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::run_sendMision(){

    

}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::deleteSphere(){
    
    //mWayPoints.clear();

    // for(int i = 0; i < mContSpheres; i++){
    //     std::string removeSphere = "sphere" + std::to_string(i);
    //     mViewer->removeShape(removeSphere);
    // }
    // mContSpheres = 0;
    ui->qvtkWidget->update();

}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
bool PCLViewer_gui::extractPointCloud(std::string _dir)
{   
    // Set up the QVTK window
    mViewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(mViewer->getRenderWindow());
    mViewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    ui->qvtkWidget->update();

    if(_dir == mDirPCD){
        if(mTypePoint == "PointXYZ"){
            mCloudT1.reset (new PointCloudT1);
            if (pcl::io::loadPCDFile<PointT1>(_dir, *mCloudT1) == -1){
                PCL_ERROR ("Couldn't read file PCD \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT1->width*mCloudT1->height << std::endl;
            mViewer->addPointCloud(mCloudT1, "cloud");
        }else if(mTypePoint == "PointXYZRGB"){
            mCloudT2.reset (new PointCloudT2);
            if (pcl::io::loadPCDFile<PointT2>(_dir, *mCloudT2) == -1){
                PCL_ERROR ("Couldn't read file PCD \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT2->width*mCloudT2->height << std::endl;
            mViewer->addPointCloud(mCloudT2, "cloud");
        }
    }else if(_dir == mDirTXT){
        mCloudT1.reset (new PointCloudT1);
        mCloudT2.reset (new PointCloudT2);
        if(convertToPointCloud(mDirTXT)){
            if(mTypePoint == "PointXYZ"){
                //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT1->width*mCloudT1->height << std::endl;
                mViewer->addPointCloud(mCloudT1, "cloud");
            }else if(mTypePoint == "PointXYZRGB"){
                //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT2->width*mCloudT2->height << std::endl;
                mViewer->addPointCloud(mCloudT2, "cloud");
            }
        }
    }else if(_dir == mDirPLY){
        if(mTypePoint == "PointXYZ"){
            mCloudT1.reset (new PointCloudT1);
            if (pcl::io::loadPLYFile<PointT1>(_dir, *mCloudT1) == -1){
                PCL_ERROR ("Couldn't read file PLY \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT1->width*mCloudT1->height << std::endl;
            mViewer->addPointCloud(mCloudT1, "cloud");
        }else if(mTypePoint == "PointXYZRGB"){
            mCloudT2.reset (new PointCloudT2);
            if (pcl::io::loadPLYFile<PointT2>(_dir, *mCloudT2) == -1){
                PCL_ERROR ("Couldn't read file PLY \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << mCloudT2->width*mCloudT2->height << std::endl;
            mViewer->addPointCloud(mCloudT2, "cloud");
        }
    }
    
    mViewer->resetCamera();
    mViewer->registerPointPickingCallback(&PCLViewer_gui::pointPickingOccurred, *this);

    if(mNameCallbackPose != ""){
        if(mTypeModelPose == "OBJ"){
            pcl::io::loadPolygonFileOBJ(mPathModelPose, mUntransformedUav);
        }else if(mTypeModelPose == "STL" ){
            pcl::io::loadPolygonFileSTL(mPathModelPose, mUntransformedUav);
        }else{
            std::cout << "Type of model pose unrecognised!" << std::endl;
            return false;
        }
        
        mViewer->addPolygonMesh(mUntransformedUav, "uav_pose");
        ui->qvtkWidget->update();
    

    }

    return true;
}

//---------------------------------------------------------------------------------------------------------------------
bool PCLViewer_gui::convertToPointCloud(std::string _dir)  
{   

    std::ifstream file_(_dir);
    if(file_.is_open()){
        for (std::string line_; std::getline(file_, line_); ) {
            std::vector<std::string> tokens;		
            split(line_, tokens, ' ');
            
            if(tokens.size() == 7){
                PointT1 pclPoint;
                PointT2 pclPointRGB;

                float pX, pY, pZ, pR, pG, pB;
                std::stringstream ssX, ssY, ssZ, ssR, ssG, ssB;
                ssX << tokens[0]; ssX >> pX;
                ssY << tokens[1]; ssY >> pY;
                ssZ << tokens[2]; ssZ >> pZ;
                ssR << tokens[4]; ssR >> pR; 
                ssG << tokens[5]; ssG >> pG;
                ssB << tokens[6]; ssB >> pB;

                pclPoint.x = pX;
                pclPoint.y = pY;
                pclPoint.z = pZ;
                pclPointRGB.x = pX;
                pclPointRGB.y = pY;
                pclPointRGB.z = pZ;
                pclPointRGB.r = pR;
                pclPointRGB.g = pG;
                pclPointRGB.b = pB;
                    
                if(mTypePoint == "PointXYZ"){
                    mCloudT1->push_back(pclPoint);
                }else if(mTypePoint == "PointXYZRGB"){
                    mCloudT2->push_back(pclPointRGB);
                }
            }	
        }
        file_.close();
    }else{
        std::cout << "File NOT open" << std::endl;
        return false;
    }

    if(mConvertAndSave){
        if(mTypePoint == "PointXYZ"){
            pcl::io::savePCDFileASCII("poindCloud.pcd", *mCloudT1);
            pcl::io::savePLYFileASCII("poindCloud.ply", *mCloudT1);
        }else if(mTypePoint == "PointXYZRGB"){
            pcl::io::savePCDFileASCII("poindCloudRGB.pcd", *mCloudT2);
            pcl::io::savePLYFileASCII("poindCloudRGB.ply", *mCloudT2);
        }
    }
    
    return true; 
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::pointPickingOccurred(const pcl::visualization::PointPickingEvent &_event, void* _args){
    
    int idx = _event.getPointIndex();
    if(idx == -1){
        return;
    }

    float x, y, z;
    _event.getPoint(x, y, z);
    //std::cout << "Position (" << x << ", " << y << ", " << z << ")" << std::endl;

    QString qRadSphere;
    qRadSphere = ui->lineEdit_RadSphere->text();
    double radSphere;
    radSphere = qRadSphere.toDouble(); 

    std::string sSphere = "sphere" + std::to_string(mContSpheres);
    //std::cout << "sSphere: " << sSphere << std::endl;
    if(mTypePoint == "PointXYZ"){
        mViewer->addSphere(mCloudT1->points[idx], radSphere, 1, 0, 0, sSphere);
        ui->qvtkWidget->update();
    }else if(mTypePoint == "PointXYZRGB"){
        mViewer->addSphere(mCloudT2->points[idx], radSphere, 1, 0, 0, sSphere);
        ui->qvtkWidget->update();
    }
    mContSpheres++;

    ui->lineEdit_MX->setText(QString::number(x));
    ui->lineEdit_MY->setText(QString::number(y));
    ui->lineEdit_MZ->setText(QString::number(z));

}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::updateObjectUAV(){
    
    mObjectLock.lock();
    Eigen::Quaternionf q;
    q.x() = mPoseOX;
    q.y() = mPoseOY;
    q.z() = mPoseOZ;
    q.w() = mPoseOW;    
    Eigen::Matrix3f R = q.normalized().toRotationMatrix();

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3,3>(0,0) = R;
    pose(0,3) = mPoseX;
    pose(1,3) = mPoseY;
    pose(2,3) = mPoseZ;
    mObjectLock.unlock();

    Eigen::Affine3f transform(pose);

    pcl::PolygonMesh meshUav = mUntransformedUav;

    PointCloudT1 pointCloud;
    pcl::fromPCLPointCloud2(meshUav.cloud, pointCloud);
    
    pcl::transformPointCloud(pointCloud, pointCloud, transform);
        
    pcl::toPCLPointCloud2(pointCloud, meshUav.cloud);

    // if(mTypePoint == "PointXYZ"){
    //     PointCloudT1 pointCloud;
    //     pcl::fromPCLPointCloud2(meshUav.cloud, pointCloud);
    
    //     pcl::transformPointCloud(pointCloud, pointCloud, transform);
        
    //     pcl::toPCLPointCloud2(pointCloud, meshUav.cloud);
    // }else if(mTypePoint == "PointXYZRGB"){
    //     PointCloudT2 pointCloud;
    //     pcl::fromPCLPointCloud2(meshUav.cloud, pointCloud);
    
    //     pcl::transformPointCloud(pointCloud, pointCloud, transform);
        
    //     pcl::toPCLPointCloud2(pointCloud, meshUav.cloud);
    // }
    
    mViewer->updatePolygonMesh(meshUav, "uav_pose");
    ui->qvtkWidget->update();

}

//---------------------------------------------------------------------------------------------------------------------
size_t PCLViewer_gui::split(const std::string &txt, std::vector<std::string> &strs, char ch) {
    size_t pos = txt.find( ch );
    size_t initialPos = 0;
    strs.clear();

    // Decompose statement
    while( pos != std::string::npos ) {
        strs.push_back( txt.substr( initialPos, pos - initialPos ) );
        initialPos = pos + 1;

        pos = txt.find( ch, initialPos );
    }

    // Add the last one
    strs.push_back( txt.substr( initialPos, std::min( pos, txt.size() ) - initialPos + 1 ) );

    return strs.size();
}
