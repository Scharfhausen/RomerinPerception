#include <globalOccupancyGrid.h>

int numPlane=0;


GlobalOccupancyGrid::GlobalOccupancyGrid(){

  width=0;
  height=0;
  numberOfPlanes=1;
  planeNormal.x=0;
  planeNormal.y=0;
  planeNormal.z=0;

}

//Getters
pcl::PointXYZ GlobalOccupancyGrid::getPlaneNormal(){

  return planeNormal;

}
int GlobalOccupancyGrid::getHeight(){

  return height;

}
int GlobalOccupancyGrid::getWidth(){

  return width;

}
Eigen::Matrix4f GlobalOccupancyGrid::getPlaneMatrix(){

  return planeMatrix;

}
Eigen::Matrix4f GlobalOccupancyGrid::getWorldMatrix(){

  return worldMatrix;

}
Eigen::Vector4f GlobalOccupancyGrid::getMinPoint(){

  return minPoint;

}
Eigen::Vector4f GlobalOccupancyGrid::getMaxPoint(){

  return maxPoint;

}
void GlobalOccupancyGrid::getGridToFile(){

  ofstream myfile;
  pcl::PCDWriter w;
  std::stringstream f;
  f<<"res/globalOccupancyGrid_"<<numPlane++<<".txt";
  std::string file_path = f.str();
  myfile.open (file_path.c_str());
  for (int i=0;i<grid.rows();i++){
    for (int j=0;j<grid.cols();j++){
       myfile << grid(i,j) << "\t"; // behaves like cout - cout is also a stream
    }
  myfile << "\n";
  }
  myfile.close();

  if(numPlane==1)numPlane=-1;

  }
Eigen::MatrixXi GlobalOccupancyGrid::getGrid(){

  return grid;

}

//Filling and matching grids
void GlobalOccupancyGrid::updateGrid(ExtractedPlaneCloud input){

  minPoint=input.getMinPoint().getArray4fMap();
  maxPoint=input.getMaxPoint().getArray4fMap();

  width=input.getWCells();
  height=input.getHCells();

  grid = input.getOccupancyGrid();    //Saves the updated grid, with the info of the previous grids and the new one
  std::cout<<" is made of "<<numberOfPlanes++<<" planes"<<std::endl;
  getGridToFile();

}
int GlobalOccupancyGrid::fill(ExtractedPlaneCloud plane){

  width=plane.getWCells();
  height=plane.getHCells();
  planeNormal=plane.getPlaneWorldZAxis();
  grid=plane.getOccupancyGrid();
  planeMatrix=plane.getPlaneMatrix();
  worldMatrix=plane.getWorldMatrix();
  minPoint=plane.getMinPoint().getVector4fMap();
  maxPoint=plane.getMaxPoint().getVector4fMap();

  occupancyGrids.push_back(*this);

  return occupancyGrids.size()-1;

}
