#include <ros/ros.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/point_cloud.h>
#include "ground_segmentation.cc"
#include "segment.cc"
#include "bin.cc"
#include "include/render/render.cpp"
#include "include/params.h"
#include "processPointClouds.cpp"

static std::string paramsFilePath = "/home/cedric/PERCEPTION_LIDAR/src/lidar_package/segmentation_clustering/include/params.txt";
static PipelineParams params;

class SegmentationNode {
  ros::Publisher ground_pub_;
  ros::Publisher obstacle_pub_;
  GroundSegmentationParams params_;
  pcl::visualization::PCLVisualizer::Ptr viewer;

public:
  SegmentationNode(ros::NodeHandle& nh,
                   const std::string& ground_topic,
                   const std::string& obstacle_topic,
                   const GroundSegmentationParams& params,
                   const pcl::visualization::PCLVisualizer::Ptr &view,
                   const bool& latch = false) : params_(params), viewer(view) {
    ground_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(ground_topic, 1, latch);
    obstacle_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(obstacle_topic, 1, latch);

    //viewer
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
  }

  void scanCallback(const pcl::PointCloud<pcl::PointXYZ>& cloud) {

      if (!params.fromFile(paramsFilePath))
      {
          std::cerr << "Error reading params file" << std::endl;
          return;
      }

      std::cout << "Max iterations :  " << params.max_iterations << std::endl;
    //viewer specific

    auto pointProcessorI = boost::make_shared<ProcessPointClouds<pcl::PointXYZ>>();
    //Clear viewer
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    //Algorithm Ground removal 
    GroundSegmentation segmenter(params_);
    std::vector<int> labels;

    segmenter.segment(cloud, &labels);
    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud (new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud (new pcl::PointCloud<pcl::PointXYZ>); 
    ground_cloud->header = cloud.header;
    obstacle_cloud->header = cloud.header;
    for (size_t i = 0; i < cloud.size(); ++i) {
      if (labels[i] == 1) ground_cloud->push_back(cloud[i]);
      else obstacle_cloud->push_back(cloud[i]);
    }



      pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud;
      filterCloud = pointProcessorI->FilterCloud(obstacle_cloud, params.filter_resolution , params.crop_min_point, params.crop_max_point,Eigen::Vector4f (-2.0, -1.5, -2, 1), Eigen::Vector4f ( 2.7, 1.5, 0, 1));
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessorI->Clustering(filterCloud,params.cluster_tolerance ,params.cluster_min_size,params.cluster_max_size);

      renderPointCloud(viewer,filterCloud,"obstCloud",Color(1,0,0));
      renderPointCloud(viewer,ground_cloud,"planeCloud",Color(0,1,0));

    int clusterId = 0;
    std::vector<Color> colors = {Color(0,0,1),Color(1,0,1),Color(1,1,0)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster:cloudClusters)  //loop through clusters
      {
          std::cout << "Cluster Size ";
          pointProcessorI->numPoints(cluster);
          renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
          //render BB
          Box box = pointProcessorI->BoundingBox(cluster);
          float width =  box.x_max - box.x_min;          
          float length = box.y_max - box.y_min;  
          float height = box.z_max - box.z_min; 
          std::cout << "Box width :" << width << std::endl;
          std::cout << "Box height : " << height << std::endl; 
          std::cout << "Box height : " << length << std::endl; 
          if(length < params.length && width < params.width && height < params.height && box.z_min < params.z_min) 
              renderBox(viewer,box,clusterId);
          ++clusterId;
      }


    viewer->spinOnce ();
    ground_pub_.publish(ground_cloud);
    obstacle_pub_.publish(filterCloud);
  }
};

int main(int argc, char** argv) {
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  ros::init(argc, argv, "ground_segmentation");
  
  //google::InitGoogleLogging(argv[0]);

  ros::NodeHandle nh("~");

  // Do parameter stuff.
  GroundSegmentationParams params;
  nh.param("visualize", params.visualize, params.visualize);
  nh.param("n_bins", params.n_bins, params.n_bins);
  nh.param("n_segments", params.n_segments, params.n_segments);
  nh.param("max_dist_to_line", params.max_dist_to_line, params.max_dist_to_line);
  nh.param("max_slope", params.max_slope, params.max_slope);
  nh.param("long_threshold", params.long_threshold, params.long_threshold);
  nh.param("max_long_height", params.max_long_height, params.max_long_height);
  nh.param("max_start_height", params.max_start_height, params.max_start_height);
  nh.param("sensor_height", params.sensor_height, params.sensor_height);
  nh.param("line_search_angle", params.line_search_angle, params.line_search_angle);
  nh.param("n_threads", params.n_threads, params.n_threads);
  // Params that need to be squared.
  double r_min, r_max, max_fit_error;
  if (nh.getParam("r_min", r_min)) {
    params.r_min_square = r_min*r_min;
  }
  if (nh.getParam("r_max", r_max)) {
    params.r_max_square = r_max*r_max;
  }
  if (nh.getParam("max_fit_error", max_fit_error)) {
    params.max_error_square = max_fit_error * max_fit_error;
  }

  std::string ground_topic, obstacle_topic, input_topic;
  bool latch;
  nh.param<std::string>("input_topic", input_topic, "input_cloud");
  nh.param<std::string>("ground_output_topic", ground_topic, "ground_cloud");
  nh.param<std::string>("obstacle_output_topic", obstacle_topic, "obstacle_cloud");
  nh.param("latch", latch, false);

  // Start node.
  SegmentationNode node(nh, ground_topic, obstacle_topic, params, viewer,latch);
  ros::Subscriber cloud_sub;
  cloud_sub = nh.subscribe("/velodyne_points", 1, &SegmentationNode::scanCallback, &node);
  ros::spin();
}
