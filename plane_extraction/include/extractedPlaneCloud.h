#pragma once
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

//Eigen libraries
#include <eigen3/Eigen/Dense>

//PCL specific libraries
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

//World Origin libraries
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>

//Global Grid
#include <globalOccupancyGrid.h>

#define EPS_Z 0.1
#define EPS_O 0.1
#define PI 3.14159265

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class GlobalOccupancyGrid;

class ExtractedPlaneCloud{

private:

    //Original PointCloud (taken from Euclid)
    PointCloud::Ptr vertices;

    //Homogeneous matrix and the PointCloud with that transformation
    Eigen::Matrix4f planeReference;           //Tp^-1 * vertices = planeReferenceVertices
    PointCloud::Ptr planeReferenceVertices;
    Eigen::Matrix4f worldReference;           //Tw^-1 * vertices = worldReferenceVertices
    PointCloud::Ptr worldReferenceVertices;
    Eigen::Matrix4f tfReference;

    pcl::ModelCoefficients::Ptr coef;

    pcl::PointXYZ minPoint;                   //Data taken from the Euclid: Up and right means incremental X and decremental Y
    pcl::PointXYZ maxPoint;
    float height;
    float width;
    Eigen::MatrixXi grid;                     //Occupancy grid. Each cell represents 1cm² and contains the number of points within that cell


public:
    ExtractedPlaneCloud();

    //Original PointCloud
    int getNumberOfVertices();
    PointCloud::Ptr getVertices();
    void setPointCloud(const PointCloud::ConstPtr& input);
    void setWorldOrigin(const std_msgs::Float32MultiArrayConstPtr &worldPos);
    void setVertices(const pcl::PointXYZ a);
    pcl::PointXYZ getCoefficients();
    Eigen::Vector4f getZCoefficients();
    void setFrameID(std::string s);
    void setCoefficients(const pcl::ModelCoefficients::ConstPtr& c);
    void clear();

    //Plane extraction
    void extractPlane();
    void computePlane(ExtractedPlaneCloud extracted);


    //Transformed PointCloud
    int getNumberOfPlaneReferenceVertices();
    PointCloud::Ptr getPlaneReferenceVertices();
    PointCloud::Ptr getWorldReferenceVertices();
    geometry_msgs::TransformStamped getTF();



    //Sizing the occupancy grid
    float getWidth();
    float getHeight();
    int getWCells();    //Closest int number (always greater than the result, i.e 1.01=2)
    int getHCells();
    int getCell(int h, int w);
    pcl::PointXYZ getMaxPoint();
    pcl::PointXYZ getMinPoint();


    //Filling the grid
    void getGridToFile();
    void setGrid(int gridNumber);
    Eigen::MatrixXi checkIfEmptyColumns(Eigen::MatrixXi matrix);


    //Calculating the homogeneous matrix
    pcl::PointXYZ getCloserPoint();
    void setMinMaxPoint(int numGrid);
    void transform();
    Eigen::Matrix4f getWorldMatrix();
    Eigen::Matrix4f getPlaneMatrix();
    Eigen::MatrixXi getOccupancyGrid();
    void printMatrix();
    void matchGrids(int numGrid);
    Eigen::Matrix4f planeToOccupancyGrid(int gridNum);



    //Occupancy grid matching
    pcl::PointXYZ getPlaneWorldZAxis();
    int isCoplanar();
    float axisAreTheSame(int i);
    pcl::PointXYZ originsVector(int num);


    //Utils
    float module (pcl::PointXYZ res);
    float dotProduct (pcl::PointXYZ a, pcl::PointXYZ b);
    float dotProduct(pcl::PointXYZ a, Eigen::Vector3f b);
    bool samePoint (pcl::PointXYZ a, pcl::PointXYZ b);
    pcl::PointXYZ unit(pcl::PointXYZ a, pcl::PointXYZ b);
    pcl::PointXYZ unit(Eigen::Vector4f a, Eigen::Vector4f b);
    pcl::PointXYZ crossProduct(pcl::PointXYZ a, pcl::PointXYZ b);
    pcl::PointXYZ vectorize(Eigen::Vector4f a, Eigen::Vector4f b);
    Eigen::Vector4f vectorizeEigen(Eigen::Vector4f a, Eigen::Vector4f b);
    bool sameNormal(Eigen::Vector3f extractedNormal, Eigen::Vector3f gridNormal);


};  //end of class ExtractedPlaneCloud

ExtractedPlaneCloud* cloud (new ExtractedPlaneCloud);

std::vector<ExtractedPlaneCloud> extractedPlanes;
std::vector<GlobalOccupancyGrid> occupancyGrids;
