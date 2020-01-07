// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "include/render/box.h"
#include <unordered_set>
#include "include/kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();




    void numPoints(typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud);

    typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> FilterCloud(typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint,Eigen::Vector4f roof_minPoint, Eigen::Vector4f roof_maxPoint);

    std::pair<typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>, typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> SeparateClouds(pcl::PointIndices::Ptr inliers, typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud);

    std::pair<typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>, typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> SegmentPlane(typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud, int maxIterations, float distanceThreshold);

    std::vector<typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> Clustering(typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cluster);

    void savePcd(typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud, std::string file);

    typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    std::unordered_set<int> Ransac3D(typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud, int maxIterations, float distanceTol);

    std::pair<typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>, typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> RANSAC_Segmentation(typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud, int maxIterations, float distanceTol);
  
    std::vector<typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> euclideanCluster(typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud, KdTree<PointT>* tree, float distanceTol, int minSize, int maxSize,int dim);

    void Proximity(int i,typename boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud,std::vector<int> &cluster,std::vector<bool> &processed,KdTree<PointT>* tree,float distanceTol, int dim) ;
    
};
#endif /* PROCESSPOINTCLOUDS_H_ */

