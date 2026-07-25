#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>
#include "msg_tool/msg/pole.hpp"
#include "msg_tool/msg/pole_detections.hpp"

// PCL点云处理相关头文件
#include <pcl/point_types.h>       // 点类型定义
#include <pcl/point_cloud.h>       // 点云数据结构
#include <pcl/common/common.h>     // 公共计算方法
#include <pcl/segmentation/extract_clusters.h> // 聚类分割
#include <pcl/kdtree/kdtree.h>     // KD树加速搜索
#include <pcl_conversions/pcl_conversions.h> // ROS与PCL转换 

// 系统头文件
#include <mutex>                   // 互斥锁
#include <memory>                  // 智能指针
#include <cmath>                   // 数学计算
#include <algorithm>               // 算法函数
#include <vector>                  // 向量容器

/**
 * @class PoleDetectionNode
 * @brief 杆状物检测节点，用于从激光雷达点云中检测单根垂直杆状物体，运行这个节点必须要发布pointCloud2消息，并且这个节点会发布杆子的位置，即自定义消息类型pole，那么他会发几个杆子阿，如果他有好几个杆子该怎么选择正确道路，打出正确航点
 */
class PoleDetectionNode : public rclcpp::Node {
public:
  // 初始化函数
  PoleDetectionNode() : Node("pole_detection_node") 
  {
    // 初始化参数
    declareParameters();//此处声明了过滤参数

    // 创建激光雷达点云订阅器
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "livox/pointcloud2", rclcpp::QoS(10), 
      std::bind(&PoleDetectionNode::lidarCallback, this, std::placeholders::_1));
    
    // 创建杆状物检测结果发布器 - 
    pole_publisher_ = this->create_publisher<msg_tool::msg::PoleDetections>(
      "/detected_poles", rclcpp::QoS(10));
    
    // 创建处理定时器
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&PoleDetectionNode::processPointCloud, this));
      
    // 初始化单个滤波器
    
    RCLCPP_INFO(this->get_logger(), "单杆检测节点已启动");
  }
  float calculate_distance(float x,float y)
  {
    return std::sqrt(x*x + y*y);
  }
private:

  /**
   * @brief 杆状物信息结构
   */
  struct PoleInfo 
  {
    int id;                   //id号码
    int lost_count;           //丢失检测次数
    float x;                  // X 坐标
    float y;                  // Y 坐标
    bool is_updated;          //用于在更新数据时检查是否那次被更新过
    bool is_initialized;      //这个用于查看这个此前是否被更新
    bool is_used=false;       //这个是给我们每次新发现的
    bool detected = false;            //检查是否被跟踪到这次
    float diameter;           // 直径
    float distance;           // 到机器人的距离
    // 默认构造函数
    PoleInfo() 
      : id(0), lost_count(0), x(0.0f), y(0.0f), is_initialized(false),is_updated(false),diameter(0.0f), distance(0.0f){}
    // 带参数的构造函数
    PoleInfo(float _x, float _y, float _d) 
      : id(0), lost_count(0), x(_x), y(_y),is_initialized(false), is_updated(false),diameter(_d), distance(std::sqrt(_x*_x + _y*_y)) {}
  };
  // 这是已经存在的杆子的列表
  std::vector<PoleInfo> PoleList;
  int pole_num_count = 0;//这个用来统计累计的杆子数目用于作为id
  

  // ROS2通信接口
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;  // 点云订阅器
  rclcpp::Publisher<msg_tool::msg::PoleDetections>::SharedPtr pole_publisher_;            // 检测结果发布器
  rclcpp::TimerBase::SharedPtr timer_;                                          // 处理定时器
  
  // 数据存储
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(); // 当前点云
  std::mutex mutex_;          // 数据访问互斥锁
  bool new_data_ = false;      // 新数据到达标志，有数据到达就是true
  
  // 杆状物跟踪
  
  // 算法参数
  double cluster_tolerance_;   // 聚类距离阈值(米)
  int min_cluster_size_;       // 最小聚类点数
  int max_cluster_size_;       // 最大聚类点数
  double pole_radius_threshold_; // 杆状物半径阈值(米)
  double min_valid_distance_;  // 最小有效距离(米)
  double min_height_;          // 最小高度要求(米)
  double pole_association_threshold_; // 杆关联距离阈值
  double filter_alpha_;        // 低通滤波系数
  
  // 发布范围限制参数
  double publish_x_min_;       // X坐标最小值(米)
  double publish_x_max_;       // X坐标最大值(米)
  double publish_y_min_;       // Y坐标最小值(米)
  double publish_y_max_;       // Y坐标最大值(米)

  /**
   * @brief 声明并加载所有参数
   */
  void declareParameters() {
    // 声明聚类参数
    this->declare_parameter("cluster_tolerance", 0.1);  // 聚类距离阈值(米)
    this->declare_parameter("min_cluster_size", 1);     // 最小聚类点数
    this->declare_parameter("max_cluster_size", 10000); // 最大聚类点数
    this->declare_parameter("pole_radius_threshold", 0.3); // 杆状物半径阈值
    this->declare_parameter("min_valid_distance", 0.1);  // 最小有效距离(米)
    this->declare_parameter("min_height", 0.2);          // 最小高度要求(米)
    this->declare_parameter("pole_association_threshold", 0.5); // 杆关联距离阈值
    
    // 声明低通滤波参数
    this->declare_parameter("filter_alpha", 0.7);        // 滤波系数
    
    // 声明发布范围限制参数
    this->declare_parameter("publish_x_min", 0.1);       // X坐标最小值(米)
    this->declare_parameter("publish_x_max", 2.0);       // X坐标最大值(米)
    this->declare_parameter("publish_y_min", -1.0);      // Y坐标最小值(米)
    this->declare_parameter("publish_y_max", 1.0);       // Y坐标最大值(米)
    
    // 加载参数值
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
    pole_radius_threshold_ = this->get_parameter("pole_radius_threshold").as_double();
    min_valid_distance_ = this->get_parameter("min_valid_distance").as_double();
    min_height_ = this->get_parameter("min_height").as_double();
    pole_association_threshold_ = this->get_parameter("pole_association_threshold").as_double();
    filter_alpha_ = this->get_parameter("filter_alpha").as_double();
    
    // 加载发布范围限制参数
    publish_x_min_ = this->get_parameter("publish_x_min").as_double();
    publish_x_max_ = this->get_parameter("publish_x_max").as_double();
    publish_y_min_ = this->get_parameter("publish_y_min").as_double();
    publish_y_max_ = this->get_parameter("publish_y_max").as_double();
  }

  /**
   * @brief 检查位置是否在发布范围内
   * @param x X坐标
   * @param y Y坐标
   * @return 是否在有效范围内
   */
  bool isPositionInValidRange(double x, double y) const {
    return (x >= publish_x_min_ && x <= publish_x_max_ && 
            y >= publish_y_min_ && y <= publish_y_max_);
  }

  /**
   * @brief 激光雷达点云回调函数
   * @param msg 输入的点云消息
   */
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    pcl::fromROSMsg(*msg, *cloud_);  // 转换ROS消息为PCL格式，这步好！cloud_是本地的存储的点云库格式。
    new_data_ = true;       // 标记有新数据需要处理
  }
  
  /**
   * @brief 点云处理主函数,也是定时器回调函数，每0.1s调用一次
   */
  void processPointCloud() 
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_copy(new pcl::PointCloud<pcl::PointXYZ>);
    // 无新数据时跳过处理，下面加花括号是为了限制线程锁的生命周期。防止堵塞。
    {
      std::lock_guard<std::mutex> lock(mutex_);//锁住，
      if (new_data_ == false || cloud_->empty()) {
        return;//这个就不删除了吗？，没有直接不发送了，tmd，这我他马没想好，留个眼睛
      }
      new_data_ = false;  // 重置数据标志
      *cloud_copy = *cloud_;
    }
    
    if (cloud_copy->empty()) {
      RCLCPP_DEBUG(this->get_logger(), "点云为空，坏了，跑路了");
      return;//这个就不删除了吗？，没有直接不发送了，tmd
    }
    
    // 执行聚类分析
    std::vector<pcl::PointIndices> cluster_indices = performClustering(cloud_copy);
    
    if (cluster_indices.empty()) 
    {
      RCLCPP_DEBUG(this->get_logger(), "未找到有效聚类");
      // return;//这个需要让我们在下面的analyzeClusters中去自己淘汰这些杆子
    }
    // 其实看来我们有时会不发布的，那么写路径规划的时候注意了
    // 分析聚类，找到杆状物，然后更新到已经发现的杆子列表之中。
    analyzeClusters(cloud_copy, cluster_indices);
    // 然后发布
    publishPoleDetection(PoleList);
  }
  

  /**
   * @brief 执行欧几里得聚类
   * @param cloud 输入点云
   * @return 聚类索引列表
   */
  std::vector<pcl::PointIndices> performClustering(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) 
  {
    // 创建KD树加速最近邻搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    
    // 执行欧几里得聚类算法
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);  // 点间距阈值
    ec.setMinClusterSize(min_cluster_size_);     // 最小点数要求
    ec.setMaxClusterSize(max_cluster_size_);     // 最大点数限制
    ec.setSearchMethod(tree);                    // 设置搜索方法
    ec.setInputCloud(cloud);                     // 设置输入点云
    ec.extract(cluster_indices);                 // 执行聚类
    
    return cluster_indices;
  }
  
  
  /**
   * @brief 执行更新
   * @param  输入坐标以及需要更新的已有杆子列表
   * @return void
   */

  void update(int pole_list_position,double x,double y) 
  {
    // 低通滤波公式: output = α * input + (1-α) * previous_output，，，filter_alpha_ =0.7,在这里
    if(PoleList[pole_list_position].is_initialized == true)//如果初始化过
    {
      PoleList[pole_list_position].x = filter_alpha_ * x + (1.0 - filter_alpha_) *  PoleList[pole_list_position].x;
      PoleList[pole_list_position].y = filter_alpha_ * y + (1.0 - filter_alpha_) *  PoleList[pole_list_position].y;
    }
    else
    {
      PoleList[pole_list_position].x = x;
      PoleList[pole_list_position].y = y;
      PoleList[pole_list_position].is_initialized = true;
    }
  
  }


  /**
   * @brief 分析聚类结果，找到最佳杆状物
   * @param cloud 点云
   * @param cluster_indices 聚类索引列表
   */
  void analyzeClusters(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const std::vector<pcl::PointIndices>& cluster_indices) 
  {
    std::vector<PoleInfo> detected_poles;
    
    // 遍历所有聚类
    for (size_t i = 0; i < cluster_indices.size(); i++) 
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      // 提取当前聚类点云
      for (const auto& idx : cluster_indices[i].indices) 
      {
        cluster_cloud->push_back((*cloud)[idx]);
      }
      if (!cluster_cloud->empty()) 
      {
        // 分析当前聚类是否为杆状物并提取特征
        PoleInfo* pole_candidate = analyzePoleCandidate(cluster_cloud);//这返回一个用new申请的空间，是一个包含diameter的杆子消息
        if (pole_candidate != nullptr) 
        {
          // 检查杆是否在有效范围内，只有在范围内的杆才会被添加到候选列表
          if (isPositionInValidRange(pole_candidate->x, pole_candidate->y)) 
          {
            detected_poles.push_back(*pole_candidate);
          } 
          else 
          {
            RCLCPP_DEBUG(this->get_logger(), "检测到杆但位置超出范围，忽略: (%.2f, %.2f)", pole_candidate->x, pole_candidate->y);
          }
          delete pole_candidate;
        }
      }
    }
    // 总上，detected_poles中的是包含这diameter消息的！！！
    // 这个东西我如果发现没有找到，该怎么办，算了，我们选择所有都加一次，然后发布。
    if (detected_poles.empty()) //此次没有目标，也就不更新了
    {
      RCLCPP_DEBUG(this->get_logger(), "未找到有效范围内的杆状物");
      // return; // 不发布任何消息，这步不能有，这样子，我后续代码会给所有人++，然后发布的
    }
    
    // 如果是第一次，怎么办，不会加上去？没事的！我们下一个循环会处理他们
    int detected_poles_size = detected_poles.size();
    for(auto& pole : PoleList)
    {
      pole.is_updated = false;//开始之前先置false，代表没有被updated
    }

    for(int i = 0 ; i < PoleList.size() ; i++)//这个函数是给list中的杆子比对的，没有被收录的杆子会被直接加入
    {
      if(PoleList[i].is_updated == true) continue;
      float best_distance_to_inlist = 1000.0f;//杆子关联距离就0.5米,这个距贼记八大。不会被。。。。
      int num = -1 ;//记录谁最大
      float pole_inlist_x=PoleList[i].x;
      float pole_inlist_y=PoleList[i].y;
      for(int u=0 ; u<detected_poles_size ; u++)//找出距离最短的
      {
        if (detected_poles[u].is_used) continue;
        float pole_outlist_x=detected_poles[u].x;
        float pole_outlist_y=detected_poles[u].y;
        float distance_to_inlist = std::sqrt(std::pow(pole_outlist_x - pole_inlist_x,2)+std::pow(pole_outlist_y-pole_inlist_y,2));
        if(distance_to_inlist<best_distance_to_inlist)
        {
          best_distance_to_inlist = distance_to_inlist;
          num=u;
        }
      }
      if(num>=0 && best_distance_to_inlist < pole_association_threshold_ )//代表这是与之前是同一个杆子,这会被更新
      {
        PoleList[i].is_updated=true;
        PoleList[i].lost_count = 0;//更新一次就置0
        PoleList[i].detected = true;//改为此次被检测到
        update(i,detected_poles[num].x,detected_poles[num].y);
        PoleList[i].distance = calculate_distance(PoleList[i].x,PoleList[i].y);
        detected_poles[num].is_used = true;
      }
      else //没被更新说明没有人要了,丢失加一
      {
        PoleList[i].lost_count++;
        PoleList[i].detected = false;
      }
    }
    for (int i = PoleList.size() - 1; i >= 0; --i) //删除超时杆子
    {
      if (PoleList[i].lost_count > 5) 
      {
        PoleList.erase(PoleList.begin() + i);
      }
    } 

    //此时剩下的是新杆子，他们需要新标号
    int new_poles_count = 0;
    for(int i=0 ; i < detected_poles_size ; i++)//loop times equals to detected_poles_size
    {
      if(detected_poles[i].is_used == true ) continue;
      detected_poles[i].id=pole_num_count+new_poles_count;
      detected_poles[i].detected = true;//这个是为了给Poleist使用的，detected_poles用不到这个
      new_poles_count++;
      PoleList.push_back(detected_poles[i]);//现在ok了，这代表着这些新杆子的添加,这里是拷贝构造函数，但是detected_poles已经有数据了，所以调用默认的拷贝苟造函数也没事
      detected_poles[i].is_used = true ;//这被使用了,虽然多此一举，但是这很符合逻辑不是？
    }
    pole_num_count += new_poles_count;//这逻辑正确，这样可以完美的让id变大，不会重复，而且还会只添加新杆子的编号

  }
  
  /**
   * @brief 分析聚类是否为杆状物
   * @param cluster_cloud 聚类点云
   * @return PoleInfo指针，如果不是杆状物则返回nullptr
   */
  PoleInfo* analyzePoleCandidate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster_cloud) {
    if (cluster_cloud->empty()) {
      return nullptr;
    }
    
    // 计算点云包围盒
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*cluster_cloud, min_pt, max_pt);
    
    // 计算物理尺寸
    float width = max_pt[0] - min_pt[0];  // X轴宽度
    float depth = max_pt[1] - min_pt[1];  // Y轴深度
    float height = max_pt[2] - min_pt[2]; // Z轴高度
    
    // 计算近似直径（取最大水平尺寸）
    float diameter = std::max(width, depth);
    
    // 找到Z轴最高点作为杆的顶点
    float max_z = min_pt[2];
    float top_x = 0.0f, top_y = 0.0f;
    int top_point_count = 0;
    
    // 定义接近最高点的阈值（在最高点下方10cm内的点都认为是顶点区域）
    float top_threshold = 0.1f;
    
    // 遍历所有点，找到最高点区域的中心
    for (const auto& point : cluster_cloud->points) {
      if (point.z > max_z) {
        max_z = point.z;
      }
    }
    
    // 计算顶点区域的平均位置
    for (const auto& point : cluster_cloud->points) {
      if (point.z >= (max_z - top_threshold)) {
        top_x += point.x;
        top_y += point.y;
        top_point_count++;
      }
    }
    
    if (top_point_count == 0) {
      return nullptr;
    }
    
    // 计算顶点的平均位置
    top_x /= top_point_count;
    top_y /= top_point_count;
    
    // 计算到原点距离
    float distance = std::sqrt(top_x*top_x + top_y*top_y);
    
    // 验证杆状物特征
    bool is_pole = diameter < pole_radius_threshold_ &&     // 直径小于阈值
                   height > min_height_ &&                  // 最小高度要求
                   max_z < 2.0f &&                         // 最高点不超过2米（排除屋顶）
                   distance > min_valid_distance_;       // 最小有效距离
    
    if (is_pole) {
      // 使用顶点位置作为杆的位置
      return new PoleInfo(top_x, top_y, diameter);
    } else {
      return nullptr;
    }
  }
  
  /**
   * @brief 发布杆状物检测结果，这里还要对消息进行转换，变成ros2消息
   * @param x X坐标
   * @param y Y坐标 
   * @param detected 是否检测到杆
   */
  void publishPoleDetection(const std::vector<PoleInfo> & polelist) //小写代表是参数
  {
    int polelist_size = polelist.size();
    auto msg = std::make_unique<msg_tool::msg::PoleDetections>();//申请一个一样大小的
    msg->header.stamp = this ->get_clock()->now();
    msg->header.frame_id= "base_link";//此坐标系以雷达为中心，但是我们的雷达中心就是无人机的中心，所以是ok的！，可以这么写
    for(const auto& pole_info : polelist)//如果空，那么循环会一次都不进行
    {
      msg_tool::msg::Pole chisa;
      chisa.id = pole_info.id;
      chisa.x = pole_info.x;
      chisa.y= pole_info.y;
      chisa.detected = pole_info.detected;
      chisa.distance = pole_info.distance;
      chisa.diameter = pole_info.diameter; 
      msg->detections.push_back(chisa);
    }
    // 发布消息
    pole_publisher_->publish(std::move(msg));
    RCLCPP_DEBUG(this->get_logger(), "已经发送此次观测到的一切杆子，ciallo ~(￣▽￣)~*");
  }
  
  
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);  // 初始化ROS2
  auto node = std::make_shared<PoleDetectionNode>();  // 创建节点实例
  rclcpp::spin(node);         // 运行节点
  rclcpp::shutdown();         // 关闭节点
  return 0;
}



// for(int i = PoleList.size()-1 ; i >= 0 ; i--)
// {
//   PoleList[i].is_updated = false;
// }
// for(int i = 0 ; i<detected_poles_size ; i++)
// {
//   float distance_to_inlist = 0;
//   float best_dist=calculate_distance();
//   for(int u=0;u<detected_poles_size;u++)
//   {
//     if(PoleList[u].is_updated == true) continue;
//     float pole_inlist_x=PoleList[u].x;
//     float pole_inlist_y=PoleList[u].y;
//     float pole_outlist_x=detected_poles[u].x;
//     float pole_outlist_y=detected_poles[u].y;
//     distance_to_inlist = std::sqrt(std::pow(pole_outlist_x - pole_inlist_x,2)+std::pow(pole_outlist_y-pole_inlist_y,2));
//   }
// }

