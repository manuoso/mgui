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

#include <fastcom/Subscriber.h>
#include <fastcom/Publisher.h>

#include <motion_planning/planners/rrtstar.h>

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

    /// Struct for received pose of the UAV
    struct pose{
		float x;
		float y;
		float z;
	    };

signals:
    /// Signal that warns that there is a change in the pose of the uav
    void poseUAVchanged();

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

    /// Method that update a UAV object in PCL GUI visualizer
    void updateObjectUAV();   

    /// Method that split a string
    /// \param txt: line to split
    /// \param strs: vector that contains the extracted information
    /// \param ch: used separator
    /// \return the size of the vector that contains the extracted information
    size_t split(const std::string &txt, std::vector<std::string> &strs, char ch);

private:
    Ui::PCLViewer_gui *ui;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    PointCloudT1::Ptr cloudT1_, cloudT1Filtered_;
    PointCloudT2::Ptr cloudT2_, cloudT2Filtered_;
    pcl::PolygonMesh untransformedUav_;

    std::chrono::time_point<std::chrono::high_resolution_clock> lastTimePose_;
    fastcom::Subscriber<pose> *subsPose_;

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
    std::string ipCallbackPose_ = "";
    int portCallbackPose_ = 0;
    std::string typeModelPose_ = "";
    std::string pathModelPose_ = "";

    rapidjson::Document configFile_;

    int contSpheres_ = 1;
    bool endSub_ = false;

    std::vector<std::pair<int, std::vector<double>>> waypoints_;
    float poseX_ = 0.0, poseY_ = 0.0, poseZ_ = 0.0, poseOX_ = 0.0, poseOY_ = 0.0, poseOZ_ = 0.0, poseOw_ = 1.0;

    std::mutex objectLock_;

};

#endif // GUIS_PCLVIEWER_GUI_H
