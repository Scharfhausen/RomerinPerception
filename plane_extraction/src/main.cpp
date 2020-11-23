#include <extractedPlaneCloud.h>
#include <eigen_conversions/eigen_msg.h>
#include <plane_extraction/globalOccupancyGrid.h>

//Publisher & Subscribers
ros::Subscriber sub;
ros::Subscriber world;
ros::Publisher pub;

void pointEigenToMsg(Eigen::Vector4f e, geometry_msgs::Point m)
{
  m.x = e(0,0);
  m.y = e(1,0);
  m.z = e(2,0);
}

//template <class Derived>
void matrixEigenToMsg(const Eigen::MatrixXi &e, std_msgs::Float32MultiArray &m)
{
  if (m.layout.dim.size() != 2)
    m.layout.dim.resize(2);
  m.layout.dim[0].stride = e.rows() * e.cols();
  m.layout.dim[0].size = e.rows();
  m.layout.dim[1].stride = e.cols();
  m.layout.dim[1].size = e.cols();
  if ((int)m.data.size() != e.size())
    m.data.resize(e.size());
  int ii = 0;
  for (int i = 0; i < e.rows(); ++i)
    for (int j = 0; j < e.cols(); ++j)
      m.data[ii++] = e.coeff(i, j);
}


int main (int argc, char** argv){

    ros::init(argc, argv, "plane_extraction");

    plane_extraction::globalOccupancyGrid grid_msg;

    ros::NodeHandle nh;   //Only one Node Handle for both subscribing and publishing

    world=nh.subscribe<std_msgs::Float32MultiArray>("/world_origin", 1, &ExtractedPlaneCloud::setWorldOrigin, cloud);
    //extractedPlanes.clear();

    sub=nh.subscribe<PointCloud>("/camera/depth/points", 1, &ExtractedPlaneCloud::setPointCloud, cloud);
    pub=nh.advertise<plane_extraction::globalOccupancyGrid>("/extracted_plane", 1);
    int i=0;


    while(ros::ok()){


      if(occupancyGrids.size()>0){
        grid_msg.header.stamp = ros::Time::now();
        grid_msg.header.frame_id = "camera_depth_optical_frame";
        pointEigenToMsg(occupancyGrids.back().getMaxPoint(),grid_msg.maxPoint);
        pointEigenToMsg(occupancyGrids.back().getMinPoint(),grid_msg.minPoint);
        //matrixEigenToMsg(occupancyGrids.back().getGrid(),grid_msg.occupancyGrid);

        //tf::matrixEigenToMsg(occupancyGrids.back().getGrid(), grid_msg.occupancyGrid)
        //Eigen::MatrixXi::Map(grid_msg.occupancyGrid, occupancyGrids.back().getGrid().rows(), occupancyGrids.back().getGrid().cols()) = occupancyGrids.back().getGrid();
        //grid_msg.occupancyGrid = *occupancyGrids.back().getGrid().data();
        ///int *c_ptr = occupancyGrids.back().getGrid().data();
        pub.publish(grid_msg);
      }


        ros::spinOnce();
        /*for(i;i<extractedPlanes.size();i++){        //extractedPlanes is a vector of ExtractedPlaneCloud instantiated in the header

            ros::Rate(2).sleep();
        }
        //if(occupancyGrids.size()>0)   std::cerr<<"Vector size is: "<<occupancyGrids.size()<<std::endl;*/




   }

    return(0);
}

