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

#include <pcl/features/normal_3d.h>
#include <random>

#include <pcl/filters/voxel_grid.h>
#include <motion_planning/planners/tsp.h>

#include <uav_abstraction_layer/GoToWaypoint.h>

//---------------------------------------------------------------------------------------------------------------------
// PUBLIC 
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
PCLViewer_gui::PCLViewer_gui(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::PCLViewer_gui),
    octree_(0.2)
    {

    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");

    connect(ui->DeleteSphere, SIGNAL(clicked()), this, SLOT(deleteSphere()));
    connect(ui->Run_gtray, SIGNAL(clicked()), this, SLOT(run_generateTray()));
    connect(ui->Run_sendM, SIGNAL(clicked()), this, SLOT(run_sendMision()));
    connect(ui->AddWP, SIGNAL(clicked()), this, SLOT(addWaypoint()));

    connect(this, &PCLViewer_gui::poseUAVchanged , this, &PCLViewer_gui::updateObjectUAV);
    connect(this, &PCLViewer_gui::qvtkChanged , this, &PCLViewer_gui::updateQVTK);

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

    if(configFile_.Parse(json.c_str()).HasParseError()){
        std::cout << "Error parsing json" << std::endl;
        return false;
    }

    convertAndSave_ = configFile_["save_pcd_ply"].GetBool();
    drawTrajectory_ = configFile_["draw_complete_tree"].GetBool();
    safeDistance_ = configFile_["safe_distance"].GetDouble();
    stepSize_ = configFile_["step_size"].GetDouble();
    appPointDistance_ = configFile_["app_point_distance"].GetDouble();
    
    usePCD_ = configFile_["use_pcd"].GetBool();
    dirPCD_ = configFile_["dir_pcd"].GetString();

    usePLY_ = configFile_["use_ply"].GetBool();
    dirPLY_ = configFile_["dir_ply"].GetString();

    useTXT_ = configFile_["use_txt"].GetBool();
    dirTXT_ = configFile_["dir_txt"].GetString();
    
    typePoint_ = configFile_["type_point"].GetString();

    leicaX_ = configFile_["leica_init_x"].GetDouble();
    leicaY_ = configFile_["leica_init_y"].GetDouble();
    leicaZ_ = configFile_["leica_init_z"].GetDouble();

    iterations_ = configFile_["iterations_traj"].GetInt();
    useSpline_ = configFile_["use_spline"].GetBool();

    typeCallbackPose_ = configFile_["type_callback"].GetString();
    nameCallbackPose_ = configFile_["callback_pose"].GetString();
    nameWPSrv_ = configFile_["wp_srv"].GetString();
    ipCallbackPose_ = configFile_["callback_ip"].GetString();
    portCallbackPose_ = configFile_["callback_port"].GetInt();
    portWaypoint_ = configFile_["wp_port"].GetInt();

    typeModelPose_ = configFile_["type_model_pose"].GetString();
    pathModelPose_ = configFile_["model_pose"].GetString();

    if(usePCD_ && !usePLY_ && !useTXT_){
        extractPointCloud(dirPCD_);
    }else if(!usePCD_ && usePLY_ && !useTXT_){
        extractPointCloud(dirPLY_);
    }else if(!usePCD_ && !usePLY_ && useTXT_){
        extractPointCloud(dirTXT_);
    }else{
        std::cout << "ERROR! You use more than one dir" << std::endl;
        return false;
    }

    #ifdef MGUI_USE_FASTCOM
        if(typeCallbackPose_ == "fastcom"){
            subsPose_ = new fastcom::Subscriber<pose>(ipCallbackPose_, portCallbackPose_);
            pubWP_ = new fastcom::Publisher<pose>(portWaypoint_);
        }
    #endif
    
    #ifdef MGUI_USE_ROS
        if(typeCallbackPose_ == "ros"){
            ros::NodeHandle nh;
            poseSub_ = nh.subscribe(nameCallbackPose_, 1, &PCLViewer_gui::CallbackPose, this);
            wpSrv_ = nh.serviceClient<uav_abstraction_layer::GoToWaypoint>(nameWPSrv_);
            posePublisherMission_ = nh.advertise<geometry_msgs::PoseStamped>("/ual/set_pose", 1) ;
        }
    #endif

    if((typeCallbackPose_ != "ros") && (typeCallbackPose_ != "fastcom")){
        std::cout << "ERROR! You use a unrecognized type of callback" << std::endl;
        return false;
    }

    lastTimePose_ = std::chrono::high_resolution_clock::now();

    #ifdef MGUI_USE_FASTCOM
        // Callback of received pose
        subsPose_->attachCallback([&](pose &_data){
            objectLock_.lock();
            poseX_ = _data.x;
            poseY_ = _data.y;
            poseZ_ = _data.z;
            objectLock_.unlock();

            if(pathModelPose_ != ""){
                auto t1 = std::chrono::high_resolution_clock::now();
                if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - lastTimePose_).count() > 100){
                    lastTimePose_ = t1;
                    emit poseUAVchanged(); 
                }
            }
        });
    #endif
    
    initPlannerVariables();

    LogTray::init("Trajectory_" + std::to_string(time(NULL)));

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

    double x, y, z;
    x = qx.toDouble();
    y = qy.toDouble();
    z = qz.toDouble();

    int id = contSpheres_ - 1;

    std::vector<double> point = {x, y, z};
    waypoints_.push_back(std::make_pair(id, point));

    std::string swaypoint = "ID: " + std::to_string(id) + " , " + "X: " + std::to_string(x) + " , " +  "Y: " + std::to_string(y) + " , " + "Z: " + std::to_string(z);
    
    ui->listWidget_WayPoints->addItem(QString::fromStdString(swaypoint));

    removeOldSphere_ = false;
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::run_generateTray(){
    // Clear trajectory vector
    trajectory_.clear();

    trajThread_ = std::thread([&](){
        // motion_planner
        if(typePoint_ == "PointXYZ"){
            std::cout << "Not implemented YET" << std::endl;
        }else if(typePoint_ == "PointXYZRGB"){
            // Remove previous trayectories if exists
            int oldCont = 0;
            while(oldCont<=cont_ ){
                deleteTrajectory("traj_"+std::to_string(oldCont));
                oldCont++;
            }
            cont_ = 0;

            if(waypoints_.size() > 0){
                // Create approximation points.
                std::vector<std::vector<float>> targetPoints = computeApproachingPoints(octree_);
                std::cout<< std::endl << "Computed approaching points" << std::endl;
                std::map<unsigned, std::map<unsigned, mp::Trajectory>> trajectories;
                Eigen::MatrixXf graph;
                buildGraphCosts(targetPoints, graph, trajectories);
                std::cout << "Built graph costs" << std::endl;
                std::cout << graph << std::endl;
                std::vector<int> optimalTrajectory = optimizeTrajectory(graph);
                std::cout << "Optimized trajectory" << std::endl;

                for(unsigned ti = 1; ti < optimalTrajectory.size(); ti++){
                    int id0 = optimalTrajectory[ti-1];
                    int id1 = optimalTrajectory[ti];
                    auto points = trajectories[id0][id1].points();
                    finalMission_.push_back(trajectories[id0][id1]);
                    
                    drawTrajectory(points, "traj_"+std::to_string(cont_));
                    cont_++;
                }
            }
        }
    });
}


void PCLViewer_gui::initPlannerVariables(){
    // Add constraint
    octree_.setInputCloud(cloudT2_);
    octree_.addPointsFromInputCloud();
    // Intantiate constraints
    mp::Constraint c1 = [&](const Eigen::Vector3f &_orig, const Eigen::Vector3f &_dest){
        pcl::PointXYZRGB query;
        query.x = _dest[0];
        query.y = _dest[1];
        query.z = _dest[2];
        std::vector<int> index;
        std::vector< float > dist;
        int npoints = octree_.nearestKSearch(query, 1, index, dist);
        
        if(npoints > 0){
            return *std::min_element(dist.begin(), dist.end()) > safeDistance_;
        }else{
            return true;
        }
    };
    mp::Constraint c2 = [&](const Eigen::Vector3f & _old, const Eigen::Vector3f &_new){
        pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::AlignedPointTVector intersections;
        octree_.getIntersectedVoxelCenters({_old[0], _old[1], _old[2]}, {leicaX_-_old[0], leicaY_-_old[1], leicaZ_-_old[2]}, intersections);
        
        return intersections.size() == 0;
    };

    constraints_.push_back(c1);
    constraints_.push_back(c2);
}

//---------------------------------------------------------------------------------------------------------------------
std::vector<std::vector<float>> PCLViewer_gui::computeApproachingPoints(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> &_octree){
    std::vector<std::vector<float>> targetPoints;

        targetPoints.push_back({
            poseX_,
            poseY_,
            poseZ_
        });

    for(unsigned i = 0; i < waypoints_.size(); i++){
        std::vector<int> pointsIds;
        std::vector<float> distances;
        pcl::PointXYZRGB p;
        p.x = waypoints_[i].second[0];
        p.y = waypoints_[i].second[1];
        p.z = waypoints_[i].second[2];
        
        _octree.radiusSearch(p, 0.3, pointsIds, distances);
        Eigen::Vector4f plane_parameters; 
        float curvature;
        pcl::computePointNormal(*cloudT2_, pointsIds, plane_parameters, curvature);

        Eigen::Vector3f newTarget = {
            p.x + plane_parameters[0]*appPointDistance_*1.1,
            p.y + plane_parameters[1]*appPointDistance_*1.1,
            p.z + plane_parameters[2]*appPointDistance_*1.1
        };

        for(auto &c:constraints_){
            if(!c(newTarget, newTarget)){
                newTarget = {
                    p.x - plane_parameters[0]*appPointDistance_*1.1,
                    p.y - plane_parameters[1]*appPointDistance_*1.1,
                    p.z - plane_parameters[2]*appPointDistance_*1.1
                };
                break;
            }
        }

        targetPoints.push_back({
                                newTarget[0],
                                newTarget[1],
                                newTarget[2]
                                });

        std::string sSphere = "sphere" + std::to_string(contSpheres_);
        QString qRadSphere;
        qRadSphere = ui->lineEdit_RadSphere->text();
        double radSphere;
        radSphere = qRadSphere.toDouble(); 

        viewer_->addSphere(pcl::PointXYZ(
                                            newTarget[0],
                                            newTarget[1],
                                            newTarget[2]
                                        ), radSphere, 0, 1, 0, sSphere);
        emit qvtkChanged();
        contSpheres_++;

        std::vector<double> pTray = {newTarget[0], newTarget[1], newTarget[2]};
        trajectory_.push_back(std::make_pair(idTray_, pTray));
        idTray_++;
    }

    return targetPoints;
}

//---------------------------------------------------------------------------------------------------------------------
std::vector<int> PCLViewer_gui::optimizeTrajectory(Eigen::MatrixXf &_graph){
    mp::TSP tsp(_graph);

    auto traj = tsp.compute();
    std::vector<int> indices = {0};
    for(auto &p: traj.points()){
        indices.push_back((int) p[0]);
    }
    indices.push_back(0);

    return indices;
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::drawTrajectory(const std::vector<Eigen::Vector3f> &_points, std::string _name){
    // Variables
    vtkSmartPointer<vtkPolyData> treeBase;
    vtkSmartPointer<vtkPoints> covisibilityNodes;

    vtkSmartPointer<vtkPolyData> covisibilityGraph;
    vtkSmartPointer<vtkPoints> covisibilityNodesTraj;
    vtkSmartPointer<vtkUnsignedCharArray> covisibilityNodeColors;

    treeBase = vtkSmartPointer<vtkPolyData>::New();
    treeBase->Allocate();
    covisibilityNodes = vtkSmartPointer<vtkPoints>::New();

    // Create new graph
    covisibilityGraph = vtkSmartPointer<vtkPolyData>::New();
    covisibilityGraph->Allocate();

    covisibilityNodeColors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    covisibilityNodeColors->SetNumberOfComponents(3);
    covisibilityNodeColors->SetName("Colors");

    covisibilityNodesTraj = vtkSmartPointer<vtkPoints>::New();

    if(useSpline_ && _points.size() > 20){
        Spline<Eigen::Vector3f, float> spl(20);
        spl.set_ctrl_points(_points);
        int nPoints = _points.size()*10;
        for(unsigned i = 0; i < nPoints; i++){
            auto p = spl.eval_f(1.0 / nPoints* i );

            std::string stringTray = std::to_string(p[0]) + " " + std::to_string(p[1]) + " " + std::to_string(p[2]);
            LogTray::get()->message(stringTray, false);

            const unsigned char green[3] = {0, 255, 0};
            covisibilityNodesTraj->InsertNextPoint( p[0], 
                                                    p[1], 
                                                    p[2]);
            covisibilityNodeColors->InsertNextTupleValue(green);
            if(i > 0){
                vtkIdType connectivity[2];
                connectivity[0] = i-1;
                connectivity[1] = i;
                covisibilityGraph->InsertNextCell(VTK_LINE,2,connectivity);
            }
        }
    }else{
        for(unsigned i = 0; i <  _points.size(); i++){

            std::string stringTray = std::to_string(_points[i][0]) + " " + std::to_string(_points[i][1]) + " " + std::to_string(_points[i][2]);
            LogTray::get()->message(stringTray, false);

            const unsigned char green[3] = {0, 255, 0};
            covisibilityNodesTraj->InsertNextPoint( _points[i][0], 
                                                    _points[i][1], 
                                                    _points[i][2]);
            covisibilityNodeColors->InsertNextTupleValue(green);
            if(i > 0){
                vtkIdType connectivity[2];
                connectivity[0] = i-1;
                connectivity[1] = i;
                covisibilityGraph->InsertNextCell(VTK_LINE,2,connectivity);
            }
        }            
    }

    //std::cout << "Drawing traj " << cont_ << std::endl;
    covisibilityGraph->SetPoints(covisibilityNodesTraj);
    covisibilityGraph->GetPointData()->SetScalars(covisibilityNodeColors);
    viewer_->addModelFromPolyData(covisibilityGraph, _name);
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, _name);
    emit qvtkChanged();
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::deleteTrajectory(std::string _name){
    viewer_->removeShape(_name);
    emit qvtkChanged();
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::buildGraphCosts(    std::vector<std::vector<float>> _targetPoints, 
                                        Eigen::MatrixXf &_graph, 
                                        std::map<unsigned, std::map<unsigned, mp::Trajectory>> &_trajectories){
    _graph = Eigen::MatrixXf::Zero(_targetPoints.size(), _targetPoints.size());

    std::mutex counterGuard;
    int finishedWorks = 0;
    int launchedWorks = 0;

    for(unsigned i = 0; i < _targetPoints.size(); i++){
        for(unsigned j = i+1; j < _targetPoints.size(); j++){
            launchedWorks++;
            std::thread worker([&](int _i, int _j){
                std::stringstream ss;
                ss << "Started worker: " << _i << ", " << _j << std::endl;
                std::cout << ss.str();

                mp::RRTStar planner(stepSize_);
                planner.initPoint({_targetPoints[_i][0], _targetPoints[_i][1], _targetPoints[_i][2]});
                planner.targetPoint({_targetPoints[_j][0], _targetPoints[_j][1], _targetPoints[_j][2]});
                
                float minx = std::min(_targetPoints[_i][0], _targetPoints[_j][0]); minx = minx<0?minx*1.4: minx*0.6;
                float miny = std::min(_targetPoints[_i][1], _targetPoints[_j][1]); miny = miny<0?miny*1.4: miny*0.6;
                float minz = std::min(_targetPoints[_i][2], _targetPoints[_j][2]); minz = minz<0?minz*1.4: minz*0.6;
                float maxx = std::max(_targetPoints[_i][0], _targetPoints[_j][0]); maxx = maxx<0?maxx*0.6: maxx*1.4;
                float maxy = std::max(_targetPoints[_i][1], _targetPoints[_j][1]); maxy = maxy<0?maxy*0.6: maxy*1.4;
                float maxz = std::max(_targetPoints[_i][2], _targetPoints[_j][2]); maxz = maxz<0?maxz*0.6: maxz*1.4;

                planner.dimensions( minx, miny, minz, maxx, maxy, maxz);
                
                planner.iterations(iterations_);
                for(auto &c:constraints_){
                    planner.addConstraint(c);
                }

                auto traj = planner.compute();
                // planner.tree(nodes, nodesInfo);
                if(traj.distance() == 0){
                    _graph(_i,_j) = std::numeric_limits<float>::max();
                    _graph(_j,_i) = std::numeric_limits<float>::max();
                } else{
                    _graph(_i,_j) = traj.distance();
                    _graph(_j,_i) = _graph(_i,_j);
                } 

                counterGuard.lock();
                _trajectories[_i][_j] = traj;
                _trajectories[_j][_i] = traj;
                finishedWorks++;
                counterGuard.unlock();

                std::cout << "Prepared trajectory between points " << _i << " and " << _j << ". Dist: " << traj.distance() << std::endl;
            }, i, j);

            worker.detach();
        }   
    }

    std::cout << "Waiting for Workers to finish" << std::endl;
    while(finishedWorks != launchedWorks){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    /*std::cout << "displaying trajs" << std::endl;
    for(unsigned i = 0; i < _targetPoints.size(); i++){
        for(unsigned j = i+1; j < _targetPoints.size(); j++){
            drawTrajectory(_trajectories[i][j].points(), "test_traj");
            std::this_thread::sleep_for(std::chrono::seconds(3));
            deleteTrajectory("test_traj");
        }
    }*/

}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::run_sendMision(){

    missionThread_ = std::thread([&](){

        #ifdef MGUI_USE_FASTCOM
            for(unsigned i = 0; i < trajectory_.size(); i++){
                pose msg;
                msg.x = trajectory_[i].second[0];
                msg.y = trajectory_[i].second[1];
                msg.z = trajectory_[i].second[2];
                pubWP_->publish(msg);
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
            }

            // 666 TODO: NOT WORKING GET POINTS TRAYECTORY

            pose msgFinalLand;
            msgFinalLand.x = poseX_;
            msgFinalLand.y = poseY_;
            msgFinalLand.z = poseZ_ + 1.5;
            pubWP_->publish(msgFinalLand);
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        #endif
        
        #ifdef MGUI_USE_ROS
            uav_abstraction_layer::GoToWaypoint srv;
            geometry_msgs::PoseStamped poseMsg;

            /*for(unsigned i = 0; i < trajectory_.size(); i++){
                srv.request.poseWP.header.stamp = ros::Time::now();
                srv.request.poseWP.header.frame_id = "map";
                srv.request.poseWP.pose.position.x = trajectory_[i].second[0];
                srv.request.poseWP.pose.position.y = trajectory_[i].second[1];
                srv.request.poseWP.pose.position.z = trajectory_[i].second[2];  
                srv.request.poseWP.pose.orientation.x = 0; 
                srv.request.poseWP.pose.orientation.y = 0; 
                srv.request.poseWP.pose.orientation.z = 0;  
                srv.request.poseWP.pose.orientation.w = 1; 

                if(wpSrv_.call(srv)){
                    if(srv.response.success){
                        std::cout << "Service of Send Waypoints success" << std::endl;
                    }else{
                        std::cout << "Service of Send Waypoints failed" << std::endl;
                    }
                }else{
                    std::cout << "Failed to call service of Send Waypoints" << std::endl;
                }	
                std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
            }

            // 666 TODO: NOT WORKING GET POINTS TRAYECTORY

            srv.request.poseWP.pose.position.x = poseX_;
            srv.request.poseWP.pose.position.y = poseY_;
            srv.request.poseWP.pose.position.z = poseZ_ + 1.5; 
            srv.request.poseWP.pose.orientation.x = 0; 
            srv.request.poseWP.pose.orientation.y = 0; 
            srv.request.poseWP.pose.orientation.z = 0;  
            srv.request.poseWP.pose.orientation.w = 1; 

            if(wpSrv_.call(srv)){
                if(srv.response.success){
                    std::cout << "Service of Send Waypoints success" << std::endl;
                }else{
                    std::cout << "Service of Send Waypoints failed" << std::endl;
                }
            }else{
                std::cout << "Failed to call service of Send Waypoints" << std::endl;
            }	
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
            */
            for(unsigned i = 0; i < finalMission_.size(); i++){
                auto wps = finalMission_[i].points();
                float dist1 = (Eigen::Vector3f(wps[0][0], wps[0][1], wps[0][2]) - Eigen::Vector3f(poseX_ ,poseY_, poseZ_)).norm();
                float dist2 = (Eigen::Vector3f(wps[wps.size()-1][0], wps[wps.size()-1][1], wps[wps.size()-1][2]) - Eigen::Vector3f(poseX_ ,poseY_, poseZ_)).norm();
                if(dist1 < dist2){
                    for(unsigned j = 0; j < wps.size() ; j++){
                        poseMsg.header.stamp = ros::Time::now();
                        poseMsg.header.frame_id = "odom";
                        poseMsg.pose.position.x = wps[j][0];
                        poseMsg.pose.position.y = wps[j][1];
                        poseMsg.pose.position.z = wps[j][2];  
                        poseMsg.pose.orientation.x = 0; 
                        poseMsg.pose.orientation.y = 0; 
                        poseMsg.pose.orientation.z = 0;  
                        poseMsg.pose.orientation.w = 1;  
                        //std::cout <<  wps[j][0] <<  "\t" <<  wps[j][1] << "\t" <<  wps[j][2] <<std::endl;
                        srv.request.blocking = false; 

                        float errorDist = 1000;
                        do{
                            posePublisherMission_.publish(poseMsg);
                            std::this_thread::sleep_for(std::chrono::milliseconds(30)); 
                            Eigen::Vector3f pose = {poseX_,poseY_, poseZ_};
                            Eigen::Vector3f target = {wps[j][0], wps[j][1], wps[j][2]};
                            errorDist = (target - pose).norm();
                            std::cout << errorDist << std::endl;
                        } while(errorDist > 0.3);
                        std::cout <<" wps " << j <<" mission " <<i << std::endl;
                        std::cout << "--------------------" << std::endl;
                    }
                }else{
                    for(unsigned j = wps.size()-1; j > 0 ; j--){
                        poseMsg.header.stamp = ros::Time::now();
                        poseMsg.header.frame_id = "odom";
                        poseMsg.pose.position.x = wps[j][0];
                        poseMsg.pose.position.y = wps[j][1];
                        poseMsg.pose.position.z = wps[j][2];  
                        poseMsg.pose.orientation.x = 0; 
                        poseMsg.pose.orientation.y = 0; 
                        poseMsg.pose.orientation.z = 0;  
                        poseMsg.pose.orientation.w = 1;  
                        //std::cout <<  wps[j][0] <<  "\t" <<  wps[j][1] << "\t" <<  wps[j][2] <<std::endl;
                        srv.request.blocking = false; 

                        float errorDist = 1000;
                        do{
                            posePublisherMission_.publish(poseMsg);
                            std::this_thread::sleep_for(std::chrono::milliseconds(30)); 
                            Eigen::Vector3f pose = {poseX_,poseY_, poseZ_};
                            Eigen::Vector3f target = {wps[j][0], wps[j][1], wps[j][2]};
                            errorDist = (target - pose).norm();
                            std::cout << errorDist << std::endl;
                        } while(errorDist > 0.4);
                        std::cout <<" wps " << j <<" mission " <<i << std::endl;
                        std::cout << "--------------------" << std::endl;
                    }
                }

                srv.request.waypoint.header.stamp = ros::Time::now();
                srv.request.waypoint.header.frame_id = "odom";
                if(dist1 < dist2){
                    srv.request.waypoint.pose.position.x = wps[wps.size()-1][0];
                    srv.request.waypoint.pose.position.y = wps[wps.size()-1][1];
                    srv.request.waypoint.pose.position.z = wps[wps.size()-1][2];      
                }else{
                    srv.request.waypoint.pose.position.x = wps[0][0];
                    srv.request.waypoint.pose.position.y = wps[0][1];
                    srv.request.waypoint.pose.position.z = wps[0][2];  
                }
                // Compute orientation 
                srv.request.waypoint.pose.orientation.x = 0; 
                srv.request.waypoint.pose.orientation.y = 0; 
                srv.request.waypoint.pose.orientation.z = 0;
                srv.request.waypoint.pose.orientation.w = 1; 
                srv.request.blocking = true; 
                if(wpSrv_.call(srv)){
                    std::cout << "Service of Send Waypoints success" << std::endl;
                }else{
                    std::cout << "Failed to call service of Send Waypoints" << std::endl;
                }	
                std::cout << "waiting" << std::endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
            }

            srv.request.waypoint.header.stamp = ros::Time::now();
            srv.request.waypoint.pose.position.x = 0;
            srv.request.waypoint.pose.position.y = 0;
            srv.request.waypoint.pose.position.z = 2;  

            // Compute orientation 
            srv.request.waypoint.pose.orientation.x = 0; 
            srv.request.waypoint.pose.orientation.y = 0; 
            srv.request.waypoint.pose.orientation.z = 0;
            srv.request.waypoint.pose.orientation.w = 1; 
            srv.request.blocking = true; 
            if(wpSrv_.call(srv)){
                std::cout << "Service of Send Waypoints success" << std::endl;
            }else{
                std::cout << "Failed to call service of Send Waypoints" << std::endl;
            }	
            std::cout << "waiting" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(3000)); 
        #endif

    });

    missionThread_.detach();
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::deleteSphere(){
    
    QList<QListWidgetItem*> items = ui->listWidget_WayPoints->selectedItems();
    foreach(QListWidgetItem * item, items){
        int index = ui->listWidget_WayPoints->row(item);
        waypoints_.erase(waypoints_.begin() + index); 
        delete ui->listWidget_WayPoints->takeItem(ui->listWidget_WayPoints->row(item));
        std::string removeSphere = "sphere" + std::to_string(index);
        viewer_->removeShape(removeSphere);
    }
    ui->qvtkWidget->update();

}

//---------------------------------------------------------------------------------------------------------------------
// PRIVATE
//---------------------------------------------------------------------------------------------------------------------

//---------------------------------------------------------------------------------------------------------------------
bool PCLViewer_gui::extractPointCloud(std::string _dir)
{   
    // Set up the QVTK window
    viewer_.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer_->setBackgroundColor(0.6, 0.6, 0.6);
    ui->qvtkWidget->SetRenderWindow(viewer_->getRenderWindow());
    viewer_->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    emit qvtkChanged();

    if(_dir == dirPCD_){
        if(typePoint_ == "PointXYZ"){
            cloudT1_.reset (new PointCloudT1);
            if (pcl::io::loadPCDFile<PointT1>(_dir, *cloudT1_) == -1){
                PCL_ERROR ("Couldn't read file PCD \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << cloudT1_->width*cloudT1_->height << std::endl;
            viewer_->addPointCloud(cloudT1_, "cloud");
        }else if(typePoint_ == "PointXYZRGB"){
            cloudT2_.reset (new PointCloudT2);
            if (pcl::io::loadPCDFile<PointT2>(_dir, *cloudT2_) == -1){
                PCL_ERROR ("Couldn't read file PCD \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << cloudT2_->width*cloudT2_->height << std::endl;
            viewer_->addPointCloud(cloudT2_, "cloud");
        }
    }else if(_dir == dirTXT_){
        cloudT1_.reset (new PointCloudT1);
        cloudT2_.reset (new PointCloudT2);
        if(convertToPointCloud(dirTXT_)){
            if(typePoint_ == "PointXYZ"){
                //std::cout << "Loaded PointCloud with Number of Points: " << cloudT1_->width*cloudT1_->height << std::endl;
                viewer_->addPointCloud(cloudT1_, "cloud");
            }else if(typePoint_ == "PointXYZRGB"){
                //std::cout << "Loaded PointCloud with Number of Points: " << cloudT2_->width*cloudT2_->height << std::endl;
                viewer_->addPointCloud(cloudT2_, "cloud");
            }
        }
    }else if(_dir == dirPLY_){
        if(typePoint_ == "PointXYZ"){
            cloudT1_.reset (new PointCloudT1);
            if (pcl::io::loadPLYFile<PointT1>(_dir, *cloudT1_) == -1){
                PCL_ERROR ("Couldn't read file PLY \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << cloudT1_->width*cloudT1_->height << std::endl;
            viewer_->addPointCloud(cloudT1_, "cloud");
        }else if(typePoint_ == "PointXYZRGB"){
            cloudT2_.reset (new PointCloudT2);
            if (pcl::io::loadPLYFile<PointT2>(_dir, *cloudT2_) == -1){
                PCL_ERROR ("Couldn't read file PLY \n");
                return false;
            }
            //std::cout << "Loaded PointCloud with Number of Points: " << cloudT2_->width*cloudT2_->height << std::endl;
            viewer_->addPointCloud(cloudT2_, "cloud");
        }
    }
    
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (cloudT2_);
    sor.setLeafSize (0.2f, 0.2f, 0.2f);
    sor.filter(*cloudT2_);

    viewer_->resetCamera();
    viewer_->registerPointPickingCallback(&PCLViewer_gui::pointPickingOccurred, *this);

    if(pathModelPose_ != ""){
        if(typeModelPose_ == "OBJ"){
            pcl::io::loadPolygonFileOBJ(pathModelPose_, untransformedUav_);
        }else if(typeModelPose_ == "STL" ){
            pcl::io::loadPolygonFileSTL(pathModelPose_, untransformedUav_);
        }else{
            std::cout << "Type of model pose unrecognised!" << std::endl;
            return false;
        }

        if(typePoint_ == "PointXYZ"){
            PointCloudT1::Ptr cloudUAV;
            cloudUAV.reset (new PointCloudT1);
            pcl::fromPCLPointCloud2(untransformedUav_.cloud, *cloudUAV);
            viewer_->addPointCloud(cloudUAV, "uav_pose");
        }else if(typePoint_ == "PointXYZRGB"){
            PointCloudT2::Ptr cloudUAV;
            cloudUAV.reset (new PointCloudT2);
            pcl::fromPCLPointCloud2(untransformedUav_.cloud, *cloudUAV);
            for(size_t i = 0; i < cloudUAV->points.size(); i++){
                cloudUAV->points[i].r = 255;
                cloudUAV->points[i].g = 255;
                cloudUAV->points[i].b = 255;
            }
            viewer_->addPointCloud(cloudUAV, "uav_pose");
        }
        emit qvtkChanged();
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
                    
                if(typePoint_ == "PointXYZ"){
                    cloudT1_->push_back(pclPoint);
                }else if(typePoint_ == "PointXYZRGB"){
                    cloudT2_->push_back(pclPointRGB);
                }
            }	
        }
        file_.close();
    }else{
        std::cout << "File NOT open" << std::endl;
        return false;
    }

    if(convertAndSave_){
        if(typePoint_ == "PointXYZ"){
            pcl::io::savePCDFileASCII("poindCloud.pcd", *cloudT1_);
            pcl::io::savePLYFileASCII("poindCloud.ply", *cloudT1_);
        }else if(typePoint_ == "PointXYZRGB"){
            pcl::io::savePCDFileASCII("poindCloudRGB.pcd", *cloudT2_);
            pcl::io::savePLYFileASCII("poindCloudRGB.ply", *cloudT2_);
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

    if(removeOldSphere_){
        std::string removeSphere = "sphere" + std::to_string(contSpheres_-1);
        viewer_->removeShape(removeSphere);
        contSpheres_--;
    }

    QString qRadSphere;
    qRadSphere = ui->lineEdit_RadSphere->text();
    double radSphere;
    radSphere = qRadSphere.toDouble(); 

    std::string sSphere = "sphere" + std::to_string(contSpheres_);
    //std::cout << "sSphere: " << sSphere << std::endl;
    if(typePoint_ == "PointXYZ"){
        viewer_->addSphere(cloudT1_->points[idx], radSphere, 1, 0, 0, sSphere);
    }else if(typePoint_ == "PointXYZRGB"){
        viewer_->addSphere(cloudT2_->points[idx], radSphere, 1, 0, 0, sSphere);
    }
    emit qvtkChanged();
    contSpheres_++;

    ui->lineEdit_MX->setText(QString::number(x));
    ui->lineEdit_MY->setText(QString::number(y));
    ui->lineEdit_MZ->setText(QString::number(z));

    removeOldSphere_ = true;
}

//---------------------------------------------------------------------------------------------------------------------
#ifdef MGUI_USE_ROS
    void PCLViewer_gui::CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& _msg){
        
        objectLock_.lock();
        poseX_ = _msg->pose.position.x;
        poseY_ = _msg->pose.position.y;
        poseZ_ = _msg->pose.position.z;
        poseOX_ = _msg->pose.orientation.x;
        poseOY_ = _msg->pose.orientation.y;
        poseOZ_ = _msg->pose.orientation.z;
        poseOW_ = _msg->pose.orientation.w;
        objectLock_.unlock();

        if(pathModelPose_ != ""){
            auto t1 = std::chrono::high_resolution_clock::now();
            if(std::chrono::duration_cast<std::chrono::milliseconds>(t1 - lastTimePose_).count() > 100){
                lastTimePose_ = t1;
                emit poseUAVchanged(); 
            }
        }
    }
#endif

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::updateObjectUAV(){
    
    objectLock_.lock();
    Eigen::Quaternionf q;
    q.x() = poseOX_;
    q.y() = poseOY_;
    q.z() = poseOZ_;
    q.w() = poseOW_;    
    Eigen::Matrix3f R = q.normalized().toRotationMatrix();

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    pose.block<3,3>(0,0) = R;
    pose(0,3) = poseX_;
    pose(1,3) = poseY_;
    pose(2,3) = poseZ_;
    objectLock_.unlock();

    Eigen::Affine3f transform(pose);
    viewer_->updatePointCloudPose("uav_pose", transform);

    ui->qvtkWidget->update();
}

//---------------------------------------------------------------------------------------------------------------------
void PCLViewer_gui::updateQVTK(){
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
