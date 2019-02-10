/////////////////////////////////////////////////////////////////
//															   //
//                  Header PCL Viewer GUI                      //
//															   //
//				Author: Manuel P. J. (aka. manuoso)			   //
//															   //
/////////////////////////////////////////////////////////////////

#ifndef PCLVIEWER_GUI_H
#define PCLVIEWER_GUI_H

#include <iostream>
#include <QMainWindow>
#include <fstream>
#include <string>
#include <sstream>
#include <mutex>
#include <thread>
#include <chrono>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/conversions.h>

#include <vtkRenderWindow.h>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/PoseStamped.h>

#include <rapidjson/document.h>

#include <pointcloudGUI/WaypointsData.h>
#include <std_srvs/SetBool.h>

typedef pcl::PointXYZ PointT1;
typedef pcl::PointCloud<PointT1> PointCloudT1;

typedef pcl::PointXYZRGB PointT2;
typedef pcl::PointCloud<PointT2> PointCloudT2;

namespace Ui
{
    class PCLViewer_gui;
}

class PCLViewer_gui : public QMainWindow
{
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

signals:
    /// Signal that warns that there is a change in the pose of the uav
    void poseUAVchanged();

private slots:
    /// Slot that export to TXT file the waypoints
    void exportToTXT();

    /// Slot that load from TXT file the waypoints to go and put it in PCL GUI visualizer
    void loadFromTXT();

    /// Slot for delete spheres in PCL GUI and delete vector of waypoints
    void Delete_SpheresClicked();

    /// Slot that add waypoint into a vector of waypoints
    void addWaypoint();

    /// Slot that send waypoints through a custom ROS Service
    void runGoToWaypoints();

    /// Slot that send order to fly through a bool ROS Service
    void runFly();

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

    /// Method for visualize a pose from a topic of ROS
    /// \param _msg: data receive to update pose
    void CallbackPose(const geometry_msgs::PoseStamped::ConstPtr& _msg);

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

    boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
    PointCloudT1::Ptr mCloudT1;
    PointCloudT2::Ptr mCloudT2;
    pcl::PolygonMesh mUntransformedUav;

    std::chrono::time_point<std::chrono::high_resolution_clock> mLastTimePose;

    bool mConvertAndSave = false;
    bool mUsePCD = false;
    bool mUsePLY = false;
    bool mUseTXT = false;
    std::string mDirPCD = "";
    std::string mDirTXT = "";
    std::string mDirPLY = "";
    std::string mTypePoint = "";
    std::string mNameCallbackPose = "";
    std::string mTypeModelPose = "";
    std::string mPathModelPose = "";

    rapidjson::Document mConfigFile;

    int mContSpheres = 1;
    bool mEndSub = false;

    std::vector<std::pair<int, std::vector<double>>> mWayPoints;
    float mPoseX = 0.0, mPoseY = 0.0, mPoseZ = 0.0, mPoseOX = 0.0, mPoseOY = 0.0, mPoseOZ = 0.0, mPoseOW = 0.0;

    std::mutex mObjectLock;

    ros::Subscriber mPoseSubscriber;
    ros::ServiceClient mWPReq, mFlyReq;

};


#endif // PCLVIEWER_GUI_H
