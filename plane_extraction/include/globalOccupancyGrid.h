#pragma once
#include <extractedPlaneCloud.h>


class ExtractedPlaneCloud;

class GlobalOccupancyGrid{

private:
  pcl::PointXYZ planeNormal;
  float height;
  float width;
  Eigen::MatrixXi grid;
  Eigen::Matrix4f planeMatrix;
  Eigen::Matrix4f worldMatrix;
  int numberOfPlanes;
  Eigen::Vector4f minPoint, maxPoint;

public:

  GlobalOccupancyGrid();
  pcl::PointXYZ getPlaneNormal();
  int getHeight();
  int getWidth();
  Eigen::MatrixXi getGrid();
  Eigen::Matrix4f getPlaneMatrix();
  Eigen::Matrix4f getWorldMatrix();
  Eigen::Vector4f getMinPoint();
  Eigen::Vector4f getMaxPoint();
  void updateGrid(ExtractedPlaneCloud input);
  void getGridToFile();
  int fill(ExtractedPlaneCloud plane);
};
