#include "SimpleVisionProcessor.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

SimpleVisionProcessor::SimpleVisionProcessor()
: interface(NULL),
viewer("PCL OpenNI Viewer"),
vox_grid() {
}

SimpleVisionProcessor::~SimpleVisionProcessor() {
  stop();
}

void SimpleVisionProcessor::processCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
  //downsample point cloud
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA > ());
  vox_grid.setInputCloud(cloud);
  vox_grid.filter(*cloud_filtered);
  printf("point cloud size raw: %d downsampled: %d\n", cloud->points.size(), cloud_filtered->points.size());

  if (!viewer.wasStopped()) {
    viewer.showCloud(cloud_filtered);
    //just here for debugging
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  //find ground plane (assumes biggest plane is ground plane)
  //normal estimation
  //NOTE: there's also a plane segmentation that doesn't require normal estimation
  pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA > ());
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal > ());
  ne.setSearchMethod(tree);
  ne.setInputCloud(cloud_filtered);
  ne.setKSearch(25);
  ne.compute(*cloud_normals);

  //perform segmentation
  pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coeffs_plane(new pcl::ModelCoefficients());
  //printf("segmenting plane ... \n");
  segmentation.setInputCloud(cloud_filtered);
  segmentation.setInputNormals(cloud_normals);
  segmentation.segment(*inliers_plane, *coeffs_plane);
  //printf("...done segmenting plane.\n");
  //printf("plane coeffs: %fx + %fy + %fz +%f\n",coeffs_plane->values[0], coeffs_plane->values[1], coeffs_plane->values[2], coeffs_plane->values[3]);

  //printf("Num plane inliers: %d.\n", inliers_plane->indices.size());
  if (inliers_plane->indices.size() < 2) {
    return;
  }
  
  //remove ground plane
  // Extract the non-plane points
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_minus_plane(new pcl::PointCloud<pcl::PointXYZRGBA > ());
  pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers_plane);
  extract.setNegative(true);
  extract.filter(*cloud_minus_plane);

  if (!viewer.wasStopped()) {
    viewer.showCloud(cloud_minus_plane);
    //just here for debugging
    boost::this_thread::sleep(boost::posix_time::seconds(1));
  }

  //cluster remaining points
  pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGBA > ());
  tree2->setInputCloud(cloud_minus_plane);
  std::vector<pcl::PointIndices> cluster_indices;
  euc_cluster.setSearchMethod(tree);
  euc_cluster.setInputCloud(cloud_minus_plane);
  euc_cluster.extract(cluster_indices);

  //heuristic to get most likely enemy robot cluster
  //currently just picking cluster with most points
  std::vector<pcl::PointIndices>::const_iterator biggest_cluster_it = cluster_indices.end();
  int max_cluster_size = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    if (it->indices.size() > max_cluster_size) {
      max_cluster_size = it->indices.size();
      biggest_cluster_it = it;
    }
  }

  //make new point cloud from winning cluster
  //NOTE: this probably isn't necessary, more for debugging
  if (biggest_cluster_it != cluster_indices.end()) {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBA>);
    for (std::vector<int>::const_iterator pit = biggest_cluster_it->indices.begin(); pit != biggest_cluster_it->indices.end(); pit++) {
      cloud_cluster->points.push_back(cloud_minus_plane->points[*pit]); //*
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    if (!viewer.wasStopped()) {
      viewer.showCloud(cloud_cluster);
      //just here for debugging
      boost::this_thread::sleep(boost::posix_time::seconds(1));
    }
  }

  //TODO: add heuristic to determine if cluster is actually robot
}

void SimpleVisionProcessor::run() {
  if (interface != NULL) {
    return;
  }

  interface = new pcl::OpenNIGrabber();

  boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&) > f =
          boost::bind(&SimpleVisionProcessor::processCloud, this, _1);

  interface->registerCallback(f);

  initParameters();

  interface->start();
}

void SimpleVisionProcessor::stop() {
  if (interface == NULL) {
    return;
  }
  interface->stop();
  delete interface;
  interface = NULL;
}

void SimpleVisionProcessor::initParameters() {
  //voxel grid downsampling
  vox_grid.setLeafSize(0.01, 0.01, 0.01); //meter leaf size
  
  // initialize the segmentation object for plane segmentation and set all the parameters
  segmentation.setOptimizeCoefficients(true);
  segmentation.setModelType(pcl::SACMODEL_NORMAL_PLANE);
  //segmentation.setModelType(pcl::SACMODEL_PLANE);
  segmentation.setNormalDistanceWeight(0.05); //0.1
  segmentation.setMethodType(pcl::SAC_RANSAC);
  segmentation.setMaxIterations(100);
  segmentation.setDistanceThreshold(0.02); //0.01
  
  //euclidean clustering
  euc_cluster.setClusterTolerance(0.02); // 2cm
  euc_cluster.setMinClusterSize(100); //num points
  euc_cluster.setMaxClusterSize(25000);
}