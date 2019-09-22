/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "kdtree.h"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }
 
    return cars;
}



void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool> processed, KdTree* tree, float distanceTol)
{
  processed[indice]=true;
  cluster.push_back(indice);
  
  std::vector<int> nearest = tree->search(points[indice],distanceTol);
  
  for(int id : nearest)
  {
    if(!processed[id])
    {
      clusterHelper(id, points, cluster, processed, tree, distanceTol);

    }
  }

}

std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
  
    std::vector<bool> processed(points.size(),false);
  
    int i = 0;
    while(i < points.size())
    {
      if(processed[i])
      {
        i++;
        continue;
      }
     
      std::vector<int> cluster;
      clusterHelper(i, points, cluster, processed, tree, distanceTol);
      clusters.push_back(cluster);  
      i++;
        
    }
  
	return clusters;

}

 std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> MyClustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance ){
 
     // Time clustering process
    auto startTime = std::chrono::steady_clock::now();
  
    //float clusterTolerance=0.53;

    //std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters;
  
    KdTree* tree = new KdTree;
  
    std::vector<std::vector<float>> points;
  
    for(int index=0; index<cloud->points.size(); index++)
    {
      std::vector<float> point;

      point.push_back(cloud->points[index].x);
      point.push_back(cloud->points[index].y);
      point.push_back(cloud->points[index].z);

      points.push_back(point); 
      
    }
  
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i); 
  
    std::vector<std::vector<int>> int_clusters = euclideanCluster(points, tree, clusterTolerance);
  
    int clusterId = 0;
  
  	for(std::vector<int> int_cluster : int_clusters)
    {
  		typename pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());

  		for(int indice: int_cluster)
        {
  			pcl::PointXYZI point;
            point.x=points[indice][0];
            point.y=points[indice][1];
            point.z=points[indice][2];
            
            clusterCloud->points.push_back(point);
        	clusterCloud->width = clusterCloud->points.size ();
      		clusterCloud->height = 1;
      		clusterCloud->is_dense = true;
        
        }
      
   		cloudClusters.push_back(clusterCloud);
      
  		++clusterId;
  	} 
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << cloudClusters.size() << " clusters" << std::endl;
   
   return cloudClusters;
 
 }

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------
  
  // Show input
  //renderPointCloud(viewer,inputCloud,"inputCloud"); //Keep input display off
  
  // Filtering
  pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor->FilterCloud(inputCloud, 0.3 , Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f ( 30, 8, 1, 1)); 
  
  // Segmentation 
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor->SegmentPlane(filterCloud, 25, 0.3); 
 
  // Show render_plane (green)
  renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

  
 // Clustering
 // Select "pcl library" logic or "my segmentation" logic 
 // Remove the comment out either of the two.
  
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 0.53, 10, 500);
  
  //std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = MyClustering(segmentCloud.first, 0.53);

  // Displsy the clustering result
  
  int clusterId2 = 0;
  std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)}; //red, yellow, blue

  for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
  {
      
      std::cout << "cluster size ";
      pointProcessor->numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId2),colors[clusterId2%colors.size()]); 
      
      Box box = pointProcessor->BoundingBox(cluster);
      renderBox(viewer,box,clusterId2);        
       
    ++clusterId2;
    
  }  
  
    
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene =false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    //renderRays(viewer, lidar->position, inputCloud);
    renderPointCloud(viewer, inputCloud, "inputCloud");

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud= pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
  
//-------------------------   
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
      
      if(true)//render_clusters)
      {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
      }
      
      if(true)//render_box)
      {
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);        
      }      
      ++clusterId;
    }
  
    renderPointCloud(viewer,segmentCloud.second,"planeCloud");
    
//-------------------------   
  
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
  
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
  
     while (!viewer->wasStopped ())
    {

      // Clear viewer
      viewer->removeAllPointClouds();
      viewer->removeAllShapes();

      // Load pcd and run obstacle detection process
      inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
      cityBlock(viewer, pointProcessorI, inputCloudI);

      streamIterator++;
      if(streamIterator == stream.end())
        streamIterator = stream.begin();

      viewer->spinOnce ();
    }  

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}

