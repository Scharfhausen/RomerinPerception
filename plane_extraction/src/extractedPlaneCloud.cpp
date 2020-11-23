#include <extractedPlaneCloud.h>

int numeroPlane=0;
int par=0;
int numberOfIterations = 0;

ExtractedPlaneCloud::ExtractedPlaneCloud(){

  vertices = PointCloud::Ptr(new PointCloud);
  planeReferenceVertices = PointCloud::Ptr(new PointCloud);
  worldReferenceVertices = PointCloud::Ptr(new PointCloud);
  coef = pcl::ModelCoefficients::Ptr (new pcl::ModelCoefficients);
  vertices->header.frame_id="camera_depth_optical_frame";
  planeReferenceVertices->header.frame_id="camera_depth_optical_frame";
  worldReferenceVertices->header.frame_id="camera_depth_optical_frame";

}


int ExtractedPlaneCloud::getNumberOfVertices(){

  return (int)vertices->points.size();

}
PointCloud::Ptr ExtractedPlaneCloud::getVertices(){

  return vertices;

}
void ExtractedPlaneCloud::setPointCloud(const PointCloud::ConstPtr& input){

  pcl::copyPointCloud(*input,*vertices);
  extractPlane();

}
void ExtractedPlaneCloud::setWorldOrigin(const std_msgs::Float32MultiArrayConstPtr& worldPos){


  //if(occupancyGrids.size()>0){
    std::cout<<worldPos->data[0]<<std::endl<<worldPos->data[3]<<std::endl;
    std::cout<<"The process will resume in 10 seconds...\n";
    ros::Rate(0.1).sleep();
  //}
  Eigen::Matrix4f rotation, translation;
  rotation<<cos(worldPos->data[3]*PI/180.0f), 0,  sin(worldPos->data[3]*PI/180.0f), 0,
            0,                                1,  0,                                0,
            -sin(worldPos->data[3]*PI/180.0f),0,  cos(worldPos->data[3]*PI/180.0f), 0,
            0,                                0,  0,                                1;

  translation<<1, 0, 0, worldPos->data[0],
               0, 1, 0, worldPos->data[1],
               0, 0, 1, worldPos->data[2],
               0, 0, 0, 1;


  tfReference = rotation;
  worldReference = translation*rotation;
  std::cerr<<"World Reference:\n"<<worldReference<<std::endl;

  /*//Rotation
  worldReference(0,0)=cos(worldPos->data[3]*PI/180.0f);
  worldReference(0,2)=sin(worldPos->data[3]*PI/180.0f);
  worldReference(2,0)=-sin(worldPos->data[3]*PI/180.0f);
  worldReference(2,2)=cos(worldPos->data[3]*PI/180.0f);

  //Translation
  worldReference(0,3)=worldPos->data[0];
  worldReference(1,3)=worldPos->data[1];
  worldReference(2,3)=worldPos->data[2];*/

  Eigen::Matrix4f temp=worldReference.inverse();
  worldReference=temp;

 /* if(occupancyGrids.size()==2){
      std::cout<<worldPos->data[0]<<std::endl;
      std::cout<<"The process will resume in 10 seconds...\n";
      ros::Rate(0.1).sleep();
    }
    worldReference = Eigen::Matrix4f::Identity();
    worldReference(0,3)=worldPos->data[0];
    worldReference(1,3)=worldPos->data[1];
    worldReference(2,3)=worldPos->data[2];

    Eigen::Matrix4f temp=worldReference.inverse();
    worldReference=temp;*/


}
void ExtractedPlaneCloud::setVertices(const pcl::PointXYZ a){

  vertices->push_back(a);

}
pcl::PointXYZ ExtractedPlaneCloud::getCoefficients(){

  pcl::PointXYZ temp (coef->values[0], coef->values[1], coef->values[2]);
  return temp;

}
Eigen::Vector4f ExtractedPlaneCloud::getZCoefficients(){

  Eigen::Vector4f temp;

  temp(0,0)=coef->values[0];
  temp(1,0)=coef->values[1];
  temp(2,0)=coef->values[2];
  temp(3,0)=1;


  return temp;
}
void ExtractedPlaneCloud::setFrameID(std::string s){

  vertices->header.frame_id=s;

}
void ExtractedPlaneCloud::setCoefficients(const pcl::ModelCoefficients::ConstPtr& c){

  *coef=*c;

}
void ExtractedPlaneCloud::clear(){

  vertices->clear();

}


//Plane Extraction & Computing
void ExtractedPlaneCloud::extractPlane(){

      //Necessary pointclouds
      PointCloud::Ptr cloud_filtered (new PointCloud),
                      cloud_projected (new PointCloud),
                      cloud_p (new PointCloud),
                      cloud_f (new PointCloud);


      // Create the filtering object
      pcl::ExtractIndices<pcl::PointXYZ> extract;


      // Build a filter to remove spurious NaNs
      pcl::PassThrough<pcl::PointXYZ> pass;
      pass.setInputCloud (vertices);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0, 1);  //Filters out everything further than 1 in Z-axis.
      pass.filter (*cloud_filtered);

      //    Segmentation object is created. It stores all of the inlying points
      //    on the plane to inliers and it stores the coefficients of the plane (Ax + By + Cz = D) in coefficients
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (0.01);


      int i = 0, nr_points = (int) cloud_filtered->points.size ();

      // While 30% of the original cloud is still there

      while (cloud_filtered->points.size () > 0.3 * nr_points){

            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);


            // Extract the inliers
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);	// The resulting cloud_p contains all points of cloud_filtered that are indexed by indices "inliers"

            // This part projects the inliers onto the plane model defined by
            // coefficients and creates another cloud (cloud_p->cloud_projected)
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            proj.setModelType (pcl::SACMODEL_PLANE);
            proj.setIndices (inliers);
            proj.setInputCloud (cloud_filtered);
            proj.setModelCoefficients (coefficients);
            proj.filter (*cloud_projected);

            /*pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;    //TODO: Find the way to avoid empty columns or rows
            // build the filter
            outrem.setInputCloud(cloud_projected);
            outrem.setRadiusSearch(0.1);
            outrem.setMinNeighborsInRadius (20);
            // apply filter
            outrem.filter (*cloud_projected);*/

            ExtractedPlaneCloud temp;

            /* Create the filtering object:
             * When set to TRUE, it stores every point that is NOT an inlier
             * in this case into cloud_f, which later swaps with cloud_filtered,
             * for later inspection of the resulting points*/


            extract.setNegative (true);
            extract.filter (*cloud_f);

            for(int i=0; i<cloud_p->points.size ();i++) temp.setVertices(cloud_p->points[i]);

            cloud_filtered.swap (cloud_f);

            temp.setCoefficients(coefficients);
            computePlane(temp);

            i++;
       }

}
void ExtractedPlaneCloud::computePlane(ExtractedPlaneCloud extracted){

  //Calculate transformation matrix to the World and to the Plane

  extracted.transform();

  //To check if it's coplanar to other planes, their normals and their X or Y axis must coincide within a defined range (EPS_Z and EPS_0)

  int matchingPlane=extracted.isCoplanar();

  /*The points are now referenced to the World Coordinate System and the Plane Coordinate System, as well as the Euclid itself.
   *The method setMinMaxPoint calculates the top-left & bottom-right or top-right bottom-left points with reference to the plane, i.e
   *giving the size of the diagonal of the plane.*/

  extracted.setMinMaxPoint(matchingPlane);

  /* If this plane is coplanar to other, the previous method will have referenced its Euclid points to that previous Plane Reference System.
   * That way, when the occupancy grid is created, the reference will be the same and it will be possible to match both grids.
   * If it didn't find any other similar planes, the Plane Reference System will be the one calculated in the transform method.
   * The grid will now take into account the width (W) and height (H) given by the setMinMaxPoint() to create a matrix that contains
   * WxH cells, each one representing 1cm² of the scene, and counting how many of the extracted points lie in each one. For a value close to
   * zero, it means that there's an object, that the plane is not available or simply that it's over. Once again, the grid will *ALWAYS*
   * be referenced to the Plane Reference System, whether it's its own one or the matching plane one.*/

  extracted.setGrid(matchingPlane);
  extracted.getGridToFile();

  /* The method isCoplanar() returns the value of the coplanar plane position inside the extractedPlanes vector, so that the plane
   * 'extracted' knows which one it's merging into. What this 'matchGrids()' method does is comparing the Normal to the plane in reference
   * to the world that is stored in the GlobalOccupancyGrid object with its Normal. In case they are similar (10% max difference), both
   * grids merge, resizing the global grid if needed and combining the previous values with the ones given by this 'extracted' plane.
   * In case it doesn't find any similar Normals in the 'globalOccupancyGrids' vector, it fills a new GlobalOccupancyGrid object, which
   * then pushes itself back into the vector.*/

  extracted.matchGrids(matchingPlane);

  
  //Lastly, the extracted plane pushes itself back into the extractedPlanes vector.

  extractedPlanes.push_back(extracted);

  std::cout<<"\n\n----------------------\n\n";

}


//Transformed PointCloud
int ExtractedPlaneCloud::getNumberOfPlaneReferenceVertices(){

  return (int)planeReferenceVertices->points.size();

}
PointCloud::Ptr ExtractedPlaneCloud::getPlaneReferenceVertices(){


  return planeReferenceVertices;

}
PointCloud::Ptr ExtractedPlaneCloud::getWorldReferenceVertices(){

  return worldReferenceVertices;

}


//Sizing the occupancy grid
float ExtractedPlaneCloud::getWidth(){

  float temp=100.0f*(getMaxPoint().x-getMinPoint().x);    //width in centimeters
  if(temp<0)return temp*(-1);
  else return temp;

}
float ExtractedPlaneCloud::getHeight(){

  float temp=100.0f*(getMaxPoint().y-getMinPoint().y);    //height in centimeters; Y axis is upside down
  if(temp<0)return -temp;
  else return temp;

}
int ExtractedPlaneCloud::getWCells(){

  return std::ceil(getWidth());                            //Closest int number (always greater than the result, i.e 1.01=2)

}
int ExtractedPlaneCloud::getHCells(){

  return std::ceil(getHeight());

}
int ExtractedPlaneCloud::getCell(int h, int w){

  return grid(h,w);

}
pcl::PointXYZ ExtractedPlaneCloud::getMaxPoint(){

  return maxPoint;

}
pcl::PointXYZ ExtractedPlaneCloud::getMinPoint(){

  return minPoint;

}


//Filling the grid
void ExtractedPlaneCloud::getGridToFile(){

  ofstream myfile;
  pcl::PCDWriter w;
  std::stringstream f;
  f<<"res/matrix_"<<numeroPlane++<<".txt";
  std::string file_path = f.str();    //Find better method
  myfile.open (file_path.c_str());
  for (int i=0;i<getHCells();i++){
    for (int j=0;j<getWCells();j++){
       myfile << grid(i,j) << "\t";
    }
  myfile << "\n";
}
  myfile.close();

  if(numeroPlane==1)numeroPlane=0;

 }
void ExtractedPlaneCloud::setGrid(int gridNumber){
  int w, h, count=0;


  grid = Eigen::MatrixXi::Zero(getHCells(),getWCells());
  Eigen::MatrixXi temp = grid;


  //Fill each cell with the number of points per square centimeter (the size of the cell represents 1cm x 1cm)

  for(int i=0; i<getNumberOfVertices();i++){
      w = std::abs((int)std::ceil((planeReferenceVertices->points[i].x - getMinPoint().x)*( 100.0f*getWCells()/getWidth() ))-1);
      h = std::abs((int)std::ceil((planeReferenceVertices->points[i].y - getMinPoint().y)*( 100.0f*getHCells()/getHeight() ))-1);
      if(w>=0 && w<getWCells() && h>=0 && h<getHCells()) {
        temp(h,w)+=1;
      }
  }

  //If there's an associated occupancy grid, reposition it
  if(gridNumber!=-1){

    Eigen::MatrixXi gridMatrix = occupancyGrids[gridNumber].getGrid();


    if(minPoint.x<occupancyGrids[gridNumber].getMinPoint()(0,0)){

      if (maxPoint.y>occupancyGrids[gridNumber].getMaxPoint()(1,0))
        grid.topRightCorner(gridMatrix.rows(), gridMatrix.cols()) = gridMatrix;
      else grid.bottomRightCorner(gridMatrix.rows(), gridMatrix.cols()) = gridMatrix;

    }
    else{
      if (maxPoint.y>occupancyGrids[gridNumber].getMaxPoint()(1,0))
        grid.topLeftCorner(gridMatrix.rows(), gridMatrix.cols()) = gridMatrix;
      else grid.bottomLeftCorner(gridMatrix.rows(), gridMatrix.cols()) = gridMatrix;
    }

  }

  for(int i=0; i<grid.rows();i++)
    for(int j=0;j<grid.cols();j++)
      grid(i,j)=std::max(grid(i,j),temp(i,j));
}

//Calculating the homogeneous matrix
pcl::PointXYZ ExtractedPlaneCloud::getCloserPoint(){

  pcl::PointXYZ temp(vertices->points[0]);

  for(int i=0;i<getNumberOfVertices();i++)
    if(module(vertices->points[i])<module(temp)) temp=vertices->points[i];

  Eigen::Vector4f temp_vector(temp.x,temp.y,temp.z,1);

  //std::cout<<"\t\tThe closer point to the Euclid is:\n"<<worldReference.inverse()*temp_vector<<std::endl;
  return temp;

}
void ExtractedPlaneCloud::setMinMaxPoint(int numGrid){

  pcl::PointXYZ temp_max, temp_min;
  pcl::getMinMax3D(*planeReferenceVertices, temp_min, temp_max);    // VERY unefficient method, use it as less as possible
  minPoint.x=temp_min.x;
  minPoint.y=temp_min.y;
  minPoint.z=0;

  maxPoint.x=temp_max.x;
  maxPoint.y=temp_max.y;
  maxPoint.z=0;


  if(numGrid!=-1){

    //Min & Max points of grid and plane in reference to the World

    Eigen::Vector4f gridMinPoint = occupancyGrids[numGrid].getMinPoint();
    Eigen::Vector4f gridMaxPoint = occupancyGrids[numGrid].getMaxPoint();

    //Check if the min and max points belong to the grid and transform it back to the plane reference

    if(gridMinPoint(0,0)<minPoint.x) minPoint.x = gridMinPoint(0,0);
    if(gridMinPoint(1,0)<minPoint.y) minPoint.y = gridMinPoint(1,0);
    if(gridMaxPoint(0,0)>maxPoint.x) maxPoint.x = gridMaxPoint(0,0);
    if(gridMaxPoint(1,0)>maxPoint.y) maxPoint.y = gridMaxPoint(1,0);


  }


}
void ExtractedPlaneCloud::transform(){

  worldReference=cloud->getWorldMatrix();

  pcl::PointXYZ closerPoint = getCloserPoint();
  pcl::PointXYZ origin (0,0,0);
  pcl::PointXYZ ZAxis(getCoefficients());
  pcl::PointXYZ closerPointToOriginVector(unit(origin,closerPoint));
  pcl::PointXYZ YAxis= crossProduct(ZAxis, closerPointToOriginVector);
  pcl::PointXYZ XAxis= crossProduct(YAxis, ZAxis);

  planeReference(0,0)= XAxis.x/module(XAxis);  planeReference(0,1)=YAxis.x/module(YAxis);   planeReference(0,2)= ZAxis.x;   planeReference(0,3)=closerPoint.x;
  planeReference(1,0)= XAxis.y/module(XAxis);  planeReference(1,1)=YAxis.y/module(YAxis);   planeReference(1,2)= ZAxis.y;   planeReference(1,3)=closerPoint.y;
  planeReference(2,0)= XAxis.z/module(XAxis);  planeReference(2,1)=YAxis.z/module(YAxis);   planeReference(2,2)= ZAxis.z;   planeReference(2,3)=closerPoint.z;
  planeReference(3,0)= 0;                      planeReference(3,1)=0;                       planeReference(3,2)= 0;         planeReference(3,3)=1;

  /**** All this code is to transform the extracted plane to a "normal" plane, meaning its axis are 1,0,0 ; 0,1,0 ; 0,0,1**********
   **** Without it, when the user inputs the values for the world matrix, the data won't coincide (check it in RViz)***************
   **** It's crucial that, with help of the Euclid's IMU, the extracted plane always manages to "stabilize" and be horizontal******/
  Eigen::Matrix4f rotateXAxis = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f rotateZAxis = rotateXAxis;


  float angleX = PI/2 - acos(ZAxis.z);
  rotateXAxis(1,1) = cos(angleX);
  rotateXAxis(1,2) = -sin(angleX);
  rotateXAxis(2,1) = sin(angleX);
  rotateXAxis(2,2) = cos(angleX);


  float angleZ =  -PI/2 + acos((rotateXAxis*planeReference)(2,0));

  rotateZAxis(0,0) = cos(angleZ);
  rotateZAxis(0,1) = -sin(angleZ);
  rotateZAxis(1,0) = sin(angleZ);
  rotateZAxis(1,1) = cos(angleZ);

  std::cerr<<"Plane Transformation Matrix:\n"<<rotateXAxis*planeReference;
  planeReference= rotateXAxis*planeReference;   //This needs some coding. It only works when the Euclid is perfectly horizontal.

  pcl::transformPointCloud (*vertices, *vertices, rotateXAxis);     //TODO: Improve transformation using IMU. Choose between transforming absolute coordinates to the plane coordinates or transform the plane to X = (1,0,0) ; Y = (0,1,0); Z= (0,0,1)
  /********************************************************************************************/

  pcl::transformPointCloud (*vertices, *planeReferenceVertices, planeReference.inverse());
  for(int i=0;i<planeReferenceVertices->size();i++) planeReferenceVertices->points[i].z=0; //Z coordinate is almost never 0. It has trash data (i.e 2*10⁻32), so it cleans it and leaves it with 0.

  pcl::transformPointCloud (*vertices, *worldReferenceVertices, worldReference.inverse());

}
void ExtractedPlaneCloud::printMatrix(){

  for(int i=0;i<4;i++){
    for(int j=0;j<4;j++){
      std::cout<<planeReference(i,j)<<"\t";
    }
    std::cout<<"\n";
  }

}
Eigen::Matrix4f ExtractedPlaneCloud::getWorldMatrix(){

  return worldReference;

}
Eigen::Matrix4f ExtractedPlaneCloud::getPlaneMatrix(){

  return planeReference;

}
Eigen::MatrixXi ExtractedPlaneCloud::getOccupancyGrid(){

  return grid;

}


//Occupancy grid matching
pcl::PointXYZ ExtractedPlaneCloud::getPlaneWorldZAxis(){

  Eigen::Vector4f tempVector(0,0,1,0);
  pcl::PointXYZ tempPCL;
  Eigen::Matrix4f worldWithNoRotation = worldReference;
  worldWithNoRotation(0,0)=1;
  worldWithNoRotation(0,2)=0;
  worldWithNoRotation(2,0)=0;
  worldWithNoRotation(2,2)=1;

  //std::cerr<<"World: \n"<<worldReference<<"\nPlane:\n"<<planeReference<<"\nResult ONE:\n"<<planeReference*tempVector;

  tempVector= worldWithNoRotation.inverse()*planeReference*tempVector;

  //std::cerr<<"\nZ World Axis:\n"<<tempVector<<std::endl;


  tempPCL.x= tempVector(0,0);
  tempPCL.y= tempVector(1,0);
  tempPCL.z= tempVector(2,0);

  return tempPCL;

}
int ExtractedPlaneCloud::isCoplanar(){

    std::cout<<"Number of occupancy grids: "<<occupancyGrids.size()<<std::endl;

    for(int i=0; i<occupancyGrids.size();i++){   //The first added plane is taken as reference for future matchings. If the criteria should change to the last one, replace the for-loop
        std::cerr<<"Coeficientes: "<<dotProduct(getPlaneWorldZAxis(), occupancyGrids[i].getPlaneNormal())<<"\nAxis: "<<axisAreTheSame(i)<<std::endl;
        if(dotProduct(getPlaneWorldZAxis(), occupancyGrids[i].getPlaneNormal())>(1-EPS_Z) && std::abs(axisAreTheSame(i))<EPS_O){
            pcl::transformPointCloud(*vertices, *planeReferenceVertices, occupancyGrids[i].getPlaneMatrix().inverse()*occupancyGrids[i].getWorldMatrix()*worldReference.inverse());
            planeReference=occupancyGrids[i].getPlaneMatrix();
            for(int i=0;i<planeReferenceVertices->size();i++) planeReferenceVertices->points[i].z=0;

            return i;
        }

    }

  std::cout<<"No coplanar ..."<<std::endl;
  return -1;

}
float ExtractedPlaneCloud::axisAreTheSame(int i){
  //std::cerr<<"OriginsVector:\n"<<originsVector(i)<<std::endl;
  return dotProduct(originsVector(i),getPlaneWorldZAxis());

}
pcl::PointXYZ ExtractedPlaneCloud::originsVector(int num){

  //Take the origin of the Plane Coordinate system and transform it to World Coordinate System (Tw⁻¹*Tp*planePoint)

  Eigen::Vector4f planeOriginCurrent, planeOriginPrevious;
  Eigen::Vector4f planeOrigin(0,0,0,1);   //The last component is 1 for the homogeneous matrix product

  //Transform to World Coordinate System
  planeOriginCurrent  = worldReference.inverse()*planeReference*planeOrigin;
  planeOriginPrevious = occupancyGrids[num].getWorldMatrix().inverse()*occupancyGrids[num].getPlaneMatrix()*planeOrigin;

 //std::cerr<<"Current:\n"<<planeOriginCurrent<<"\nPrevious\n"<<planeOriginPrevious<<std::endl;
  return vectorize(planeOriginCurrent, planeOriginPrevious);

}
void ExtractedPlaneCloud::matchGrids(int numGrid){

  if(numGrid!=-1){std::cout<<"The grid "<<numGrid;occupancyGrids[numGrid].updateGrid(*this);}
  else{GlobalOccupancyGrid temp;temp.fill(*this);}

}
Eigen::Matrix4f ExtractedPlaneCloud::planeToOccupancyGrid(int gridNum){

  Eigen::Matrix4f gridPlaneMatrix=occupancyGrids[gridNum].getPlaneMatrix();

  Eigen::Vector4f gridXAxis(gridPlaneMatrix.col(0));
  Eigen::Vector4f planeXAxis(planeReference.col(0));
  Eigen::Vector4f originVector = originsVector(gridNum).getVector4fMap(); originVector(3,0)=0;

  originVector=occupancyGrids[gridNum].getPlaneMatrix().inverse()*occupancyGrids[gridNum].getWorldMatrix()*originVector;

  gridXAxis=occupancyGrids[gridNum].getPlaneMatrix().inverse()*gridXAxis;
  planeXAxis=occupancyGrids[gridNum].getPlaneMatrix().inverse()*occupancyGrids[gridNum].getWorldMatrix()*worldReference.inverse()*planeXAxis;

  float angle= acos(  (planeXAxis(0,0)*gridXAxis(0,0))+
                      (planeXAxis(1,0)*gridXAxis(1,0))/
                      (planeXAxis.head<2>().norm()*gridXAxis.head<2>().norm()));

  if(angle!=angle) angle=0;     //if angle is a NaN


  Eigen::Matrix4f transformToFirstGrid;
  transformToFirstGrid(0,0)=cos(angle);     transformToFirstGrid(0,1)=-sin(angle);     transformToFirstGrid(0,2)= 0;       transformToFirstGrid(0,3)=originVector(0,0);
  transformToFirstGrid(1,0)=sin(angle);     transformToFirstGrid(1,1)=cos(angle);      transformToFirstGrid(1,2)= 0;       transformToFirstGrid(1,3)=originVector(1,0);
  transformToFirstGrid(2,0)=0;              transformToFirstGrid(2,1)=0;               transformToFirstGrid(2,2)= 1;       transformToFirstGrid(2,3)=0;
  transformToFirstGrid(3,0)=0;              transformToFirstGrid(3,1)=0;               transformToFirstGrid(3,2)= 0;       transformToFirstGrid(3,3)=1;

  Eigen::Matrix4f temp =transformToFirstGrid*occupancyGrids[gridNum].getPlaneMatrix().inverse()*occupancyGrids[gridNum].getWorldMatrix()*worldReference.inverse()*planeReference;
  //if(temp(1,0)>0){
    //std::cout<</*"Plane Reference Matrix:\n"<<occupancyGrids[gridNum].getPlaneMatrix()<<*/"\nX AXIS:\n"<<gridXAxis<<"\nPlane AXIS without transformation:\n"<<planeXAxis<<"\nPlane AXIS with transformation:\n"<<transformToFirstGrid*planeXAxis<<std::endl;
    //std::cout<<"AXIS ARE:\n"<<gridXAxis<<std::endl<<planeXAxis<<std::endl;
    //std::cout<<"NUMERADOR: "<<gridXAxis(0,0)*planeXAxis(0,0)+gridXAxis(1,0)*planeXAxis(1,0)+gridXAxis(2,0)*planeXAxis(2,0)<<"  DENOMINADOR: "<<gridXAxis.head<3>().norm()*planeXAxis.head<3>().norm()<<std::endl;
    std::cout<<"\t\tCurrent Plane:\n"<<temp<<"\n\t\tOccupancy Grid:\n"<<gridPlaneMatrix.inverse()*gridPlaneMatrix<<std::endl;
    std::cout<<"Planes distance:\n"<<originVector<<"\nAngle: "<<angle<<std::endl;

    //std::cout<<"TRANSFORM MATRIX: \n"<<transformToFirstGrid<<std::endl;
    //ros::Rate(0.1).sleep();
  //}

  return transformToFirstGrid;
  //if(transformToFirstGrid.isApprox(Eigen::MatrixXf::Identity(4,4), 0.1)) return transformToFirstGrid;
  //else return Eigen::MatrixXf::Identity(4,4);

}


//Utils
float ExtractedPlaneCloud::module(pcl::PointXYZ res){
  return sqrt(res.x*res.x+res.y*res.y+res.z*res.z);
}
float ExtractedPlaneCloud::dotProduct(pcl::PointXYZ a, pcl::PointXYZ b){
  return a.x*b.x+a.y*b.y+a.z*b.z;
}
float ExtractedPlaneCloud::dotProduct(pcl::PointXYZ a, Eigen::Vector3f b){
  return a.x*b(0,0)+a.y*b(1,0)+a.z*b(2,0);
}
pcl::PointXYZ ExtractedPlaneCloud::crossProduct(pcl::PointXYZ a, pcl::PointXYZ b)
{
  pcl::PointXYZ c (a.y*b.z - a.z*b.y, -((a.x*b.z)-(b.x*a.z)), a.x*b.y - a.y*b.x);
  return c;
}
bool ExtractedPlaneCloud::samePoint(pcl::PointXYZ a, pcl::PointXYZ b){
  if((a.x>0.90*b.x && a.x<1.1+b.x) && (a.y>0.9*b.y && a.y<1.1+b.y) && (a.z>0.9*b.z && a.z<1.1+b.z) && a.x*b.x>0 && a.y*b.y>0 && a.z*b.z>0)
    return true;		//Each component has a 10% tolerance and same sign of both components is checked
  else return false;
}
pcl::PointXYZ ExtractedPlaneCloud::unit(pcl::PointXYZ a, pcl::PointXYZ b){
  pcl::PointXYZ result;
  result.x = b.x-a.x;
  result.y = b.y-a.y;
  result.z = b.z-a.z;
  float mod = module(result);
  result.x/=mod;
  result.y/=mod;
  result.z/=mod;

  return result;
}
pcl::PointXYZ ExtractedPlaneCloud::unit(Eigen::Vector4f a, Eigen::Vector4f b){
  pcl::PointXYZ result;
  result.x = b(1,0)-a(1,0);
  result.y = b(2,0)-a(2,0);
  result.z = b(3,0)-a(3,0);
  float mod = module(result);
  result.x/=mod;
  result.y/=mod;
  result.z/=mod;

  return result;
}
pcl::PointXYZ ExtractedPlaneCloud::vectorize(Eigen::Vector4f a, Eigen::Vector4f b){
  pcl::PointXYZ result;
  result.x = b(0,0)-a(0,0);
  result.y = b(1,0)-a(1,0);
  result.z = b(2,0)-a(2,0);
  return result;
}
Eigen::Vector4f ExtractedPlaneCloud::vectorizeEigen(Eigen::Vector4f a, Eigen::Vector4f b){
  Eigen::Vector4f result;
  result(0,0) = b(0,0)-a(0,0);
  result(1,0) = b(1,0)-a(1,0);
  result(2,0) = b(2,0)-a(2,0);
  result(3,0) = b(3,0)-a(3,0);
  return result;
}
bool ExtractedPlaneCloud::sameNormal(Eigen::Vector3f extractedNormal, Eigen::Vector3f gridNormal){

    if(  std::abs(gridNormal(0,0)-extractedNormal(0,0))<0.1  &&
         std::abs(gridNormal(1,0)-extractedNormal(1,0))<0.1  &&
         std::abs(gridNormal(2,0)-extractedNormal(2,0))<0.1)
              return true;

    else      return false;

}

