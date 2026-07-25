#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>


// 这个节点是收到livox消息之后再去滤波之后发布pointcloud2消息的
class LivoxToPointCloud2 : public rclcpp::Node
{
public:
    LivoxToPointCloud2()
    : Node("livox_to_pointcloud2")
    {
        // 订阅 Livox CustomMsg 类型的点云数据
        subscription_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
            "/livox/lidar", 10, 
            std::bind(&LivoxToPointCloud2::custom_msg_callback, this, std::placeholders::_1)
        );

        // 发布转换后的 PointCloud2 数据，话题为"/livox/pointcloud2"
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/pointcloud2", 10);
        
        RCLCPP_INFO(this->get_logger(), "LivoxToPointCloud2 node started");
    }

private:
    void custom_msg_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr custom_msg)//接受一次发送一次
    {
        // 检查输入数据，如果是0就不处理
        if (custom_msg->point_num == 0) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Processing %d points", custom_msg->point_num);

        // 填充点云数据
        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();//这个数据类型与pointXYZ的区别是这个多了一个intensity消息，I就代表反射强度intensity
        cloud->width = custom_msg->point_num;
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);

        for (size_t i = 0; i < custom_msg->point_num; ++i) {
            cloud->points[i].x = custom_msg->points[i].x;
            cloud->points[i].y = custom_msg->points[i].y;
            cloud->points[i].z = custom_msg->points[i].z;
            cloud->points[i].intensity = custom_msg->points[i].reflectivity;
        }
        //以上是从Customsg消息转成PointXYZI
        RCLCPP_DEBUG(this->get_logger(), "Original cloud size: %zu", cloud->points.size());

        // 应用滤波器
        auto filtered_cloud = applyFilters(cloud);//在发布点云消息之前他就已经进行了滤波
        
        if (filtered_cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No points remaining after filtering");
            return;
        }

        RCLCPP_DEBUG(this->get_logger(), "Filtered cloud size: %zu", filtered_cloud->points.size());

        // 创建并发布 PointCloud2 消息
        sensor_msgs::msg::PointCloud2 pointcloud2_msg;
        pcl::toROSMsg(*filtered_cloud, pointcloud2_msg);//pcl消息转ros2消息开发
        pointcloud2_msg.header.stamp = this->get_clock()->now();
        pointcloud2_msg.header.frame_id = custom_msg->header.frame_id;

        publisher_->publish(pointcloud2_msg);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr applyFilters(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
    {
        auto current_cloud = cloud;
        
        // 直通滤波 - 分别对每个轴进行滤波，已经过滤好了
        // 定义滤波器
        std::vector<std::tuple<std::string, float, float>> filters = {
            {"x", 0, 2.0},
            {"y", -2.0, 2.0},
            {"z", -1.0, 1.0}
        };
        // 下面开始滤波
        for (const auto& filter : filters) {
            const auto& [axis, min_val, max_val] = filter;
            
            // 检查当前点云是否为空
            if (current_cloud->points.empty()) {
                RCLCPP_WARN(this->get_logger(), "Point cloud became empty during %s filtering", axis.c_str());
                break;
            }

            auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            
            pcl::PassThrough<pcl::PointXYZI> pass;
            pass.setInputCloud(current_cloud);  // 设置输入点云
            pass.setFilterFieldName(axis);
            pass.setFilterLimits(min_val, max_val);
            pass.filter(*filtered_cloud);
            
            RCLCPP_DEBUG(this->get_logger(), "After %s filter: %zu -> %zu points", 
                        axis.c_str(), current_cloud->points.size(), filtered_cloud->points.size());
            
            current_cloud = filtered_cloud;
        }

        // 体素滤波 (降低点云密度)
        if (!current_cloud->points.empty()) {
            auto downsampled_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            
            pcl::VoxelGrid<pcl::PointXYZI> voxel_grid;
            voxel_grid.setInputCloud(current_cloud);
            voxel_grid.setLeafSize(0.05f, 0.05f, 0.05f);  // 调整体素大小
            voxel_grid.filter(*downsampled_cloud);
            
            RCLCPP_DEBUG(this->get_logger(), "After voxel filter: %zu -> %zu points", 
                        current_cloud->points.size(), downsampled_cloud->points.size());
            
            current_cloud = downsampled_cloud;
        }

        return current_cloud;
    }
    //类内定义发布订阅者
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxToPointCloud2>());
    rclcpp::shutdown();
    return 0;
}