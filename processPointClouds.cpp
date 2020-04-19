// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  	pcl::VoxelGrid<PointT> sor; // Create voxel grid object.
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
  	sor.setInputCloud (cloud); //Give our point cloud for processing
  	sor.setLeafSize (filterRes, filterRes, filterRes); // Cell size for our voxel grid
  	sor.filter (*cloud_filtered);
  
  	// Creating region of interest
  	typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>); // Creating new point cloud.
  	pcl::CropBox< PointT > region (true); // Create cropbox object.
  	region.setMin(minPoint); // Set min (x, y, z) coordinates a point should have to be inside RoI
  	region.setMax(maxPoint); // Set max (x, y, z) coordinates a point can have to be inside RoI
  	region.setInputCloud(cloud_filtered);
  	region.filter(*cloudRegion);
  
  	// Find roof points from our ego car
  	std::vector<int> indices;
  	pcl::CropBox< PointT > roof (true);
  	roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
  	roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
  	roof.setInputCloud(cloudRegion);
  	roof.filter(indices);

  	// Iterate through roof indices and Converting int vector of indices to pcl pointindices format
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	for (int point : indices) {
      inliers->indices.push_back(point);
    }
  	// Finally, remove roof indices from RoI cloudRegion
  	pcl::ExtractIndices<PointT> extractedIndices;
  	extractedIndices.setInputCloud(cloudRegion);
  	extractedIndices.setIndices(inliers);
  	extractedIndices.setNegative(true);
  	extractedIndices.filter(*cloudRegion);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  	typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>);
  	typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>);
    for (int index : inliers->indices){
    	planeCloud->points.push_back(cloud->points[index]);
    } //Above for loop will populate planeCloud with inliers
  	// Lets populate obstacle cloud now
  	pcl::ExtractIndices<PointT> extract;
  	extract.setInputCloud (cloud);
  	extract.setIndices (inliers);
  	extract.setNegative (true);
  	extract.filter (*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // Will use this later to break point clouds in two sections
    // TODO:: Fill in this function to find inliers for the cloud.
	pcl::SACSegmentation<PointT> seg; // Create the segmentation object
  	seg.setOptimizeCoefficients (true);
  
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (maxIterations);
  	seg.setDistanceThreshold (distanceThreshold);
  
  	seg.setInputCloud (cloud); // // Segment the largest planar component from input cloud
  	seg.segment (*inliers, *coefficients); // Generate the points which lie in the plane (inliers) and coefficient of the plane itself
  	if (inliers->indices.size () == 0){
    	std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      	//break;
    }
  // Extract the inliers
  	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
 
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::My_SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	//srand(time(NULL));
	auto start_time = std::chrono::steady_clock::now();
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
  	for (int i = 0; i < maxIterations; i++){
    	std::unordered_set<int> inliers; //inlier set to keep count of inliers found in this iteration
      while (inliers.size() != 3)	{
      	inliers.insert(rand() % cloud->points.size()); // Add a random inlier
        }
      float x1, y1, z1, x2, y2, z2, x3, y3, z3; // variables to hold coordinates of our line points.
      auto itr = inliers.begin();
      x1 = cloud->points[*itr].x;
      y1 = cloud->points[*itr].y;
      z1 = cloud->points[*itr].z;
      itr++;
      x2 = cloud->points[*itr].x;
      y2 = cloud->points[*itr].y;
      z2 = cloud->points[*itr].z;
      itr++;
      x3 = cloud->points[*itr].x;
      y3 = cloud->points[*itr].y;
      z3 = cloud->points[*itr].z;
      
      // Get line equation coefficients
      float a = (y2 - y1)*(z3-z1) - (z2-z1)*(y3-y1);
      float b = (z2 - z1)*(x3-x1) - (x2-x1)*(z3-z1);
      float c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
      float d = -(a*x1 + b*y1 + c*z1);
      // Iterate through point cloud
      for (int index = 0; index < cloud->points.size(); index++){
      	if (inliers.count(index) > 0) // Maybe this step can be avoided to fasten up the algo
          continue;
        
        PointT point = cloud->points[index];
        float x4 = point.x; // x coordinate of the point which will check whether its an inlier or not
        float y4 = point.y; // y coordinate of the point which will check whether its an inlier or not
        float z4 = point.z;
        
        //float d = fabs( ((a*x4) + (b*y4) + (c*z4) + d ) / (sqrt((a*a) + (b*b) + (c*c))));
        float d = fabs(a*x4 + b * y4 + c * z4 + d) / sqrt (a*a + b*b + c*c);
        if (d < distanceTol)
          inliers.insert(index);
      }
      if (inliers.size() > inliersResult.size())
        inliersResult = inliers;
    }

// TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  	typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>);
  	typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>);

    for (int i=0; i < cloud->points.size(); i++){
      if (inliersResult.count(i)){
        planeCloud->points.push_back(cloud->points[i]);
      }
      else{
        obstCloud->points.push_back(cloud->points[i]);
      }
    }
    // Calculate time
    auto end_time = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "My Segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    // Create set of both point clouds
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;

    
	/* Failed attempt
  pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
  for (auto i = inliersResult.begin(); i != inliersResult.end(); ++i) {
    inliers->indices.push_back(inliersResult(i)); 
}*/
	
  // Failed attempt
  //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
   
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

	typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>); // Creating the KdTree object
  	tree->setInputCloud (cloud); // feed in point cloud to be processed
  
  	std::vector<pcl::PointIndices> cluster_indices;
  	pcl::EuclideanClusterExtraction<PointT> ec; // Create clustering object
  	ec.setClusterTolerance (clusterTolerance);
  	ec.setMinClusterSize (minSize);
  	ec.setMaxClusterSize (maxSize);
  	ec.setSearchMethod (tree);
  	ec.setInputCloud (cloud);
  	ec.extract (cluster_indices);
  
  	// Iterate through found indices
  	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    	typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
      	for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          cloud_cluster->points.push_back (cloud->points[*pit]);
      
      	cloud_cluster->width = cloud_cluster->points.size ();
      	cloud_cluster->height = 1;
      	cloud_cluster->is_dense = true;
      	clusters.push_back(cloud_cluster);
    }
  
  
  auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}