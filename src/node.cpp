#include <chrono>
#include <memory>
#include <iostream>
#include <vector>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/distances.h> 
#include <Eigen/Core>
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;
using namespace std;

class LaserscanToPointcloud : public rclcpp::Node 
{
  public:
    LaserscanToPointcloud()
    : Node("laserscan_to_pointcloud")
    {
      auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
      
      // LiDAR Subscriber
      laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", qos_profile, std::bind(&LaserscanToPointcloud::laser_callback, this, std::placeholders::_1));
      
      // Point Cloud Publisher
      cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud", 10);
    }

  private :
    void laser_callback(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {  
      //RCLCPP_INFO(get_logger(), "start");
      std::vector<pair<double, double>> scan_map;
      std::string hd = msg->header.frame_id;
      double angle = msg->angle_min;
      for (int i=0; i<(int) msg->ranges.size(); i++) 
      {
        if (!isinf(msg->ranges[i])) 
        {
          double px = cos(angle) * msg->ranges[i];
          double py = sin(angle) * msg->ranges[i];
          scan_map.push_back(pair<double, double>(px, py));
        }
        angle += msg->angle_increment;
      }
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZ>);
      // RCLCPP_INFO(get_logger(),"123");
      for (int i=0; i<(int) scan_map.size(); i++) 
      {
        pcl::PointXYZ pt;
        // pt = pcl::PointXYZRGB(255, 255, 255);
        pt.x = scan_map[i].first;
        pt.y = scan_map[i].second;
        pt.z = 0.0;
        cloud_->points.push_back(pt);
      }

      pcl::PointCloud<pcl::PointXYZ>::Ptr filter(new pcl::PointCloud<pcl::PointXYZ>);
      // 通过体素滤波过滤一部分的点
      pcl::VoxelGrid<pcl::PointXYZ> sor;
      sor.setInputCloud (cloud_);
      sor.setLeafSize (0.004f, 0.004f, 0.004f);
      sor.filter (*filter);
      pc2_msg_ = std::make_shared<sensor_msgs::msg::PointCloud2>();

      // 利用欧式聚类将点云分割
      // 创建KdTreee对象作为搜索方法
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud(filter);
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance(0.01);  // 1cm
      ec.setMinClusterSize(10);  // 最大点云数量
      ec.setMaxClusterSize(15);  // 最小点云数量
      ec.setSearchMethod(tree);  // 搜索方法
      ec.setInputCloud(filter);  // 处理对象
      // 聚类抽取结果保存在一个数组中，数组中每个元素代表抽取的一个组件点云的下标
      ec.extract(cluster_indices);

      // 创建保存点云族的点云
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      // 两个端点
      pcl::PointXYZ min_pt, max_pt;
      // 端点的序号
      int min_index, max_index;
      // int j = 0;
      // 遍历抽取结果
      for (const auto& cluster : cluster_indices)
      {
        for (const auto& idx : cluster.indices) 
        {
          cloud_cluster->push_back((*filter)[idx]);
        }
        // 求点云中最远的两点
        double dis = -1;
        // 随便找一个点云中的点
        pcl::PointXYZ p0 = cloud_cluster->points[3];
        //利用循环寻找点云中离这个点最远的点
        for (int i = 0; i < cloud_cluster->size(); i++)
        {
          pcl::PointXYZ p1 = cloud_cluster->points[i];
          double temp = pcl::squaredEuclideanDistance(p0, p1);//省一个开方运算
          if (temp > dis)
          {
            dis = temp;
            max_pt = p1;
            max_index = i;
          }
        }
        dis = -1;
        // 再利用一个循环寻找离这个最远点最远的另一个最远点
        for (int i = 0; i < cloud_cluster->size(); i++)
        {
          pcl::PointXYZ p1 = cloud_cluster->points[i];
          double temp = pcl::squaredEuclideanDistance(max_pt, p1);
          if (temp > dis)
          {
            dis = temp;
            min_pt = p1;
            min_index = i;
          }
        }
        // 最远两点之间的距离
        auto distance = pcl::euclideanDistance(min_pt, max_pt);
        float a = 2.0f;
        // 最远两点连成直线的中点
        pcl::PointXYZ p2;
        p2.x = (min_pt.x + max_pt.x) / a;
        p2.y = (min_pt.y + max_pt.y) / a;
        p2.z = 0.f;
        // 距中点最近的点
        pcl::PointXYZ p_min;
        // 利用kd树寻找离中点最近的点
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud (cloud_cluster);
        int K =1;
        vector<int> pointIdxKNNSearch(K);
        vector<float> pointKNNSquaredDistance(K);
        if ( kdtree.nearestKSearch (p2, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
        {
          for (size_t i = 0; i < pointIdxKNNSearch.size (); ++i)
          {
            p_min.x = cloud_cluster->points[pointIdxKNNSearch[i]].x;
            p_min.y = cloud_cluster->points[pointIdxKNNSearch[i]].y;
            p_min.z = cloud_cluster->points[pointIdxKNNSearch[i]].z;
          }
        }
        // 一类点云中所有的点到最远两点直线距离之和
        double dis_add = 0;
        // 最远点连成直线的方向向量
        Eigen::Vector4f line_dir((min_pt.x - max_pt.x),(min_pt.y - max_pt.y),0,1.0);
        for (int i = 0; i < cloud_cluster->size(); i++)
        {
          // 最远两点中的一点与弧线上的点
          Eigen::Vector4f min_p,cluster_p;
          //设置这两个点的信息
          min_p<<min_pt.x,min_pt.y,0,1.0;
          cluster_p<<cloud_cluster->points[i].x,cloud_cluster->points[i].y,0,1.0;
          // 利用公式计算圆弧上的点到最远两点连成的直线的距离
          double dis_squared = pcl::sqrPointToLineDistance(cluster_p, min_p, line_dir);
          // 将距离累加
          dis_add += dis_squared;
        }
        // 以点到直线距离之和与最远两点的之间的距离比值作为判断条件
        double ratio = dis_add / distance;
        // 原点（雷达坐标）
        pcl::PointXYZ p;
        p.x = 0;
        p.y = 0;
        p.z = 0;
        // 最远两点连线的中点到到中点最近的点的距离
        float dis1 = pcl::euclideanDistance(p2,p_min);
        // 半径
        float r = distance / a;
        // 雷达到识别物体的距离
        float real_distance = pcl::euclideanDistance(p_min,p);
        // 识别多个柱子或小球则将它们到雷达的距离放入一个容器
        vector<float> real_dis;
        // 识别条件
        if(dis1 <= r && real_distance >0.2f && real_distance < 1.f &&
            distance >= 0.02f && distance <= 0.05f && ratio > 6.0e-07 && ratio < 1.0)
        {
          
          // 将点云信息格式转换为ROS2信息格式发布，在RVIZ中可以显示处理结果
          pcl::toROSMsg(*cloud_cluster, *pc2_msg_);
          pc2_msg_->header.frame_id = hd;
          pc2_msg_->header.stamp = now();
          cloud_pub_->publish(*pc2_msg_);
          // 识别到圆柱或小球的时候将距离放进容器
          real_dis.push_back(real_distance);
        }
        // 场上有多个小球和圆柱时通过循环找到离小车最近的圆柱或小球
        for(int i;i <= real_dis.size();i++)
        {
          float temp;
          if(real_dis[i] < real_dis[i+1])
          {
            temp = real_dis[i];
          }
          else if(real_dis[i+1] < real_dis[i])
          {
            temp = real_dis[i+1];
          }
        }
      }
      // 清空保存点云族的点云
      cloud_cluster->clear();
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserscanToPointcloud>());
  rclcpp::shutdown();
  return 0;
}
