/* 
 * File:   SimpleVisionProcesor.hpp
 * Author: evan
 *
 * Created on July 2, 2014, 10:37 PM
 */

#ifndef SIMPLEVISIONPROCESOR_HPP
#define	SIMPLEVISIONPROCESOR_HPP


#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

class SimpleVisionProcessor {
public:

  SimpleVisionProcessor();
  ~SimpleVisionProcessor();

  void run();
  void stop();

private:
  void processCloud(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
  void initParameters();

  pcl::Grabber* interface;
  pcl::visualization::CloudViewer viewer;

  pcl::VoxelGrid<pcl::PointXYZRGBA> vox_grid;
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGBA, pcl::Normal> segmentation;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> euc_cluster;
};


#endif	/* SIMPLEVISIONPROCESOR_HPP */

