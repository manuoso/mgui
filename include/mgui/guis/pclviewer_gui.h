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

#ifndef GUIS_PCLVIEWER_GUI_H
#define GUIS_PCLVIEWER_GUI_H

#include <QMainWindow>

#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <mutex>
#include <thread>
#include <chrono>

#include <rapidjson/document.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <vtkRenderWindow.h>

#ifdef MGUI_USE_FASTCOM
    #include <fastcom/Subscriber.h>
    #include <fastcom/Publisher.h>
#endif

#ifdef MGUI_USE_ROS
    #include <ros/ros.h>
    #include <geometry_msgs/PoseStamped.h>
    #include <mgui/WaypointData.h>
#endif

#include <motion_planning/planners/rrtstar.h>
#include <motion_planning/utils/splines.hpp>

#include <mgui/LogTray.h>

typedef pcl::PointXYZ PointT1;
typedef pcl::PointCloud<PointT1> PointCloudT1;

typedef pcl::PointXYZRGB PointT2;
typedef pcl::PointCloud<PointT2> PointCloudT2;

namespace Ui{
    class PCLViewer_gui;
}

class PCLViewer_gui : public QMainWindow {
    Q_OBJECT

public:
    /// Constructor
    explicit PCLViewer_gui(QWidget *parent = 0);

    /// Destructor
    ~PCLViewer_gui ();

    /// Method that configure PCL GUI
    /// \param _argc: from main
    /// \param _argv: from main
    /// \return true if params are good or without errors, false if something failed
    bool configureGUI(int _argc, char **_argv);

    /// Struct for received pose and send the velocity of the UAV
    struct pose{
        float x;
        float y;
        float z;
    };

signals:
    /// Signal that warns that there is a change in the pose of the uav
    void poseUAVchanged();

    /// Signal that warns that there is a change in qvtk widget and update it
    void qvtkChanged();

private slots:
    /// Slot that add waypoint into a vector of waypoints
    void addWaypoint();

    /// Slot that generate and visualize the trayectory
    void run_generateTray();

    /// Slot that send mission
    void run_sendMision();

    /// Slot for delete sphere in PCL GUI
    void deleteSphere();

private:
    /// Method for extract the pointcloud in TXT file, PCD or PLY
    /// \param _dir: file path
    /// \return true if params are good or without errors, false if something failed
    bool extractPointCloud(std::string _dir);

    /// Method for convert the pointcloud in TXT file for visualize in PCL GUI
    /// \param _dir: file path
    /// \return true if params are good or without errors, false if something failed
    bool convertToPointCloud(std::string _dir);

    /// Method that capture x, y and z from a mouse clic
    /// \param _event: x, y and z captured and other info
    /// \param _args: args from picking
    void pointPickingOccurred(const pcl::visualization::PointPickingEvent &_event, void* _args);

    /// Method that add a object to PCL GUI visualizer
    /// \param _name: name of the object to add
    /// \param _mesh: object to add
    void addObject(std::string _name, pcl::PolygonMesh _mesh);

    #ifdef MGUI_USE_ROS
        /// Method for visualize a change of pose from a topic of ROS
        /// \param _msg: data receive to update pose
        void CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& _msg);
    #endif
    
    /// Method that update a UAV object in PCL GUI visualizer
    void updateObjectUAV();   

    /// Method that update QVTK widget in PCL GUI visualizer
    void updateQVTK();   

    /// Method that split a string
    /// \param txt: line to split
    /// \param strs: vector that contains the extracted information
    /// \param ch: used separator
    /// \return the size of the vector that contains the extracted information
    size_t split(const std::string &txt, std::vector<std::string> &strs, char ch);


    void initPlannerVariables();
    std::vector<std::vector<float>> computeApproachingPoints(pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> &_octree);
    void buildGraphCosts(std::vector<std::vector<float>> _targetPoints, Eigen::MatrixXf &_graph, std::map<unsigned, std::map<unsigned, mp::Trajectory>> &_trajectories);
    std::vector<int> optimizeTrajectory(Eigen::MatrixXf &_graph);

    void drawTrajectory(const std::vector<Eigen::Vector3f> &_points, std::string _name);
    void deleteTrajectory(std::string _name);

    // Variables for the planner
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB> octree_;
    std::vector<mp::Constraint> constraints_;
private:
    Ui::PCLViewer_gui *ui;

    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    PointCloudT1::Ptr cloudT1_, cloudT1Filtered_;
    PointCloudT2::Ptr cloudT2_, cloudT2Filtered_;
    pcl::PolygonMesh untransformedUav_;

    std::chrono::time_point<std::chrono::high_resolution_clock> lastTimePose_;
    
    #ifdef MGUI_USE_FASTCOM
        fastcom::Subscriber<pose> *subsPose_ = nullptr;
        fastcom::Publisher<pose> *pubWP_ = nullptr;
    #endif

    #ifdef MGUI_USE_ROS
        ros::Subscriber poseSub_;
        ros::ServiceClient wpSrv_;
    #endif
    
    bool convertAndSave_ = false;
    bool usePCD_ = false;
    bool usePLY_ = false;
    bool useTXT_ = false;
    std::string dirPCD_ = "";
    std::string dirTXT_ = "";
    std::string dirPLY_ = "";
    std::string typePoint_ = "";
    std::string typeCallbackPose_ = "";
    std::string nameCallbackPose_ = "";
    std::string nameWPSrv_ = "";
    std::string ipCallbackPose_ = "";
    int portCallbackPose_ = 0;
    int portWaypoint_ = 0;
    std::string typeModelPose_ = "";
    std::string pathModelPose_ = "";

    rapidjson::Document configFile_;

    float safeDistance_ = 0.5;
    float appPointDistance_ = 0.5;
    float stepSize_ = 0.1;
    bool drawTrajectory_ = false;
    int iterations_ = 0;
    int contSpheres_ = 0;
    int cont_ = 0;
    bool endSub_ = false;
    bool useSpline_ = false;
    bool removeOldSphere_ = false;

    std::vector<std::pair<int, std::vector<double>>> waypoints_;

    int idTray_ = 0;
    std::vector<std::pair<int, std::vector<double>>> trajectory_;
    
    float poseX_ = 0.0, poseY_ = 0.0, poseZ_ = 0.0, poseOX_ = 0.0, poseOY_ = 0.0, poseOZ_ = 0.0, poseOW_ = 1.0;
    float leicaX_ = 0.0, leicaY_ = 0.0, leicaZ_ = 0.0;

    std::thread trajThread_;
    std::mutex objectLock_;

};

#endif // GUIS_PCLVIEWER_GUI_H
