#define PCL_NO_PRECOMPILE

#include "pointcloud_type.h"

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Polygon.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/segmentation/extract_clusters.h>
#include "pcl/search/kdtree.h"
#include <common/mydisplay.h>


using namespace std;

//从一个给定的点云 cloud 中移除所有具有无限（或非有限）坐标的点，
//并返回一个新的点云 res，其中只包含有限坐标的点。

PointCloud remove_infinite_points(PointCloud cloud)
{
    PointCloud res;
    res.header = cloud.header;  //头信息

    for (auto it = cloud.points.begin(); it != cloud.points.end(); ++it)
    {    
        //来检查浮点数是否为有限值
        if (pcl_isfinite(it->x) && pcl_isfinite(it->y) && pcl_isfinite(it->z))
        {
            res.push_back(*it);
        }
    }

    return res;
}

//点云坐标转换
PointCloud CloudTf(PointCloud cp_src, string frame_id)
{
    static tf::TransformListener listener;

    PointCloud res;
    vector<int> aa;
    //来处理和过滤点云，包括移除包含 NaN（非数字）或无限大（Inf）值的点。
    //于 NaN 和 Inf 值在点云中通常被视为无效或错误的数据
    pcl::removeNaNFromPointCloud(cp_src, res, aa); //移除无效点，cp为初始点云，res为移除无效点后的点云
    
    if (cp_src.header.frame_id != frame_id)
    {
        for (auto &it:res)
        {
            geometry_msgs::PointStamped src_p, dst_p;
            src_p.header.frame_id = res.header.frame_id;

            src_p.point.x = it.x;
            src_p.point.y = it.y;
            src_p.point.z = it.z;
            //坐标转换
            //将src_p里的点转换到frame_id坐标系中
            transformPoint(frame_id, src_p, dst_p, "AAA");
            it.x = dst_p.point.x;
            it.y = dst_p.point.y;
            it.z = dst_p.point.z;
        }
        // pcl_ros::transformPointCloud(frame_id, res, res, listener); //第一个参数为目标坐标系。第二个参数为原始点云，第三个参数为目标点云，第四个参数为接受到的坐标
        res.header.frame_id = frame_id;
    }

    return res;
}

//根据给定的x、y、z坐标范围来过滤点云
PointCloud Cloud_PassThrough(PointCloud& cp_src, float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
    PointCloud res=cp_src;  //复制原始点云到结果点云中  

    pcl::PassThrough<PointType> pass; 
   
    //// 应用Z轴过滤器 
    if(z_max-z_min>0.001)
    {
        pass.setInputCloud(res.makeShared());  //指定了滤波器将要应用的字段名
        pass.setFilterFieldName("z");  //指定了滤波器将要应用的字段名
        pass.setFilterLimits(z_min, z_max);  //过滤的上下限值
        pass.setFilterLimitsNegative(false);  //滤波器将保留那些其z坐标值在[z_min, z_max]范围内的点
        pass.filter(res);  //执行过滤操作
    }
    //// 应用x轴过滤器 
    if(x_max-x_min>0.001)
    {
        pass.setInputCloud(res.makeShared());
        pass.setFilterFieldName("x");
        pass.setFilterLimits(x_min, x_max);
        pass.setFilterLimitsNegative(false);
        pass.filter(res);
    }
    //// 应用y轴过滤器 
    if(y_max-y_min>0.001)
    {
        pass.setInputCloud(res.makeShared());
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_min, y_max);
        pass.setFilterLimitsNegative(false);
        pass.filter(res);
    }

    return res;
};

//移除点云中那些在给定半径内邻居点数量小于指定阈值的点
//过滤点云
PointCloud Cloud_RadiusFilter(PointCloud cp_src, float radius, int min_count)
{
    PointCloud res=cp_src;
    if(cp_src.size()==0)  return res;

    pcl::RadiusOutlierRemoval<PointType> outrem;   //移除点云中那些在给定半径内邻居点数量少于指定阈值的点
    outrem.setInputCloud(cp_src.makeShared());  //// 设置滤波器的输入点云
    //// 设置搜索邻居点的半径。在这个半径内的点将被考虑用于计算每个点的邻居数量
    outrem.setRadiusSearch(radius);  //
    // 设置每个点必须拥有的最小邻居数量。如果点的邻居数量少于这个值，它将被视为离群点并被移除。  
    outrem.setMinNeighborsInRadius(min_count);
    outrem.filter(res);

    return res;
}

//根据点的强度值（intensity）来过滤点云
PointCloud Cloud_IntensityFilter(PointCloud cp_src, int min_value)
{
    PointCloud res;
    res.header=cp_src.header;

    for(auto it: cp_src)
    {
        if(it.intensity>min_value)  res.push_back(it);
    }

    return res;
}

//确定polygon边界的范围，即x,y的极值大小
void GetScaleXYByPolygon(geometry_msgs::Polygon polygon, float &x_min, float &x_max, float &y_min, float &y_max)
{
    x_max = -10000, x_min = 10000;
    y_max = -10000, y_min = 10000;
    for (auto pp : polygon.points)
    {
        if (x_max < pp.x)  x_max = pp.x;
        else if (x_min > pp.x)  x_min = pp.x;
        if (y_max < pp.y)  y_max = pp.y;
        else if (y_min > pp.y)  y_min = pp.y;
    }

}

//对输入的点云（cp_src）进行下采样处理
PointCloud Cloud_DownSampling(PointCloud cp_src, geometry_msgs::Point scale)
{   
    // 创建体素网格滤波器对象  
    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud(cp_src.makeShared());
    //用于设置体素网格的边长，即每个体素（或“盒子”）在三维空间中的大小。
    //这个方法通常接受三个浮点数作为参数，分别代表体素在X、Y、Z轴上的尺寸
    sor.setLeafSize(scale.x, scale.y, scale.z);
    //创建结果点云  
    PointCloud res;
    sor.filter(res);
    return res;
}

class TMinBoundingBox
{
private:
    ros::NodeHandle *nh;
    ros::Publisher markers_pub;
    PointCloud cloud;
    float precision=0.1, min_angle=-10, max_angle=10;
    void minscalebyrotate(float angle, geometry_msgs::Point &scale, geometry_msgs::Point &center);
    
public:
    geometry_msgs::PoseStamped pose;
    geometry_msgs::Point scale;
    float bouding_angle = 0;

    TMinBoundingBox(string caption);
    void Calculate(PointCloud c, float pre=0.2, float min_a=-10, float max_a=10);
    void Publish();
};

//发布caption话题
TMinBoundingBox::TMinBoundingBox(string caption)
{
    nh=new ros::NodeHandle("~");  //建了一个新的ROS节点句柄（ros::NodeHandle）对象，并将其指针赋值给成员变量nh
    //发布visualization_msgs::MarkerArray类型的消息
    markers_pub = nh->advertise<visualization_msgs::MarkerArray>(caption, 10);
};

//基于原始点云和给定的旋转角度来计算新的尺寸和中心点
//计算旋转后点云在X和Y轴方向上的最小边界框
void TMinBoundingBox::minscalebyrotate(float angle, geometry_msgs::Point &scale, geometry_msgs::Point &center)
{
    if (cloud.size() < 100)  return;

    float max_x = -1000, min_x = 1000, max_y = -1000, min_y = 1000;
    for (auto it : cloud)
    {
        float x, y;
        x = it.x * cos(angle) + it.y * sin(angle);
        y = it.y * cos(angle) - it.x * sin(angle);
        if (x > max_x)
            max_x = x;
        if (x < min_x)
            min_x = x;
        if (y > max_y)
            max_y = y;
        if (y < min_y)
            min_y = y;
    }
    scale.x = max_x - min_x;
    scale.y = max_y - min_y;

    center.x = (min_x + max_x) * 0.5;
    center.y = (min_y + max_y) * 0.5;
};

//计算并设置给定点云 c的边界框的尺寸、中心位置、以及边界框的朝向（通过旋转角度表示）
void TMinBoundingBox::Calculate(PointCloud c, float pre, float min_a, float max_a)
{
    cloud = c;
    precision = pre, min_angle = min_a, max_angle = max_a;
    //初始化 pose 的头部信息，包括参考帧ID和时间戳。
    pose.header.frame_id = c.header.frame_id;
    pose.header.stamp = ros::Time::now();

    ros::Time start_time=ros::Time::now();

    // FILE *fp=fopen("/home/wsk/test_data.txt", "w");
    
    int n = (max_angle - min_angle) / precision;
    geometry_msgs::Point bouding_scale, bouding_center;
    bouding_scale.x = bouding_scale.y = 10000;
    for (int i = 0; i < n; i++)
    {
        float angle = (min_angle + i * precision) / 180.0 * M_PI;
        geometry_msgs::Point scale, center;
        //用 minscalebyrotate 函数来计算旋转后点云在X和Y轴方向上的最小边界框的尺寸和中心点
        minscalebyrotate(angle, scale, center);
        //比较周长
        if (bouding_scale.x + bouding_scale.y > scale.x + scale.y)
        // if(bouding_scale.y>scale.y)
        {
            bouding_scale = scale;
            bouding_angle = angle;
            bouding_center = center;

        }
    }
    // fclose(fp);
    //计算Z轴方向上的包围盒尺寸 bouding_scale.z 和中心点 bouding_center.z
    float min_z = 10000, max_z = -10000;
    for (auto it : cloud)
    {
        if (it.z >= max_z)
            max_z = it.z;
        if (it.z <= min_z)
            min_z = it.z;

    }
    bouding_center.z = (max_z + min_z) * 0.5;
    bouding_scale.z = max_z - min_z;
    if (bouding_scale.z < 0.01)
        bouding_scale.z = 0.01;

    pose.header.frame_id = cloud.header.frame_id;
    pose.header.stamp = ros::Time::now();
    //计算位姿
    pose.pose.position.x = bouding_center.x * cos(bouding_angle) - bouding_center.y * sin(bouding_angle);
    pose.pose.position.y = bouding_center.x * sin(bouding_angle) + bouding_center.y * cos(bouding_angle);
    pose.pose.position.z = bouding_center.z;
    // if(bouding_scale.x<bouding_scale.y)  bouding_angle-=M_PI*0.5;
    //偏向
    //从 bouding_angle 创建一个四元数消息，该消息表示包围盒的朝向
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(bouding_angle);
    scale = bouding_scale;
    ros::Time stop_time = ros::Time::now();
    // ROS_INFO("spend time=%.3f", (stop_time-start_time).toSec());
};

//对边界框和ARROW的配置，并发布出去
void TMinBoundingBox::Publish()
{
    visualization_msgs::MarkerArray markers;

    markers.markers.clear();
    visualization_msgs::Marker marker;
    marker.header.frame_id = cloud.header.frame_id;
    marker.header.stamp = ros::Time::now();
    //边界框的配置
    marker.ns = "minboudingbox";  //命名空间
    marker.lifetime = ros::Duration(0.2);  //生命周期
    marker.frame_locked = true; //是否锁定到帧
    marker.type = visualization_msgs::Marker::CUBE;  //标记类型
    marker.action = visualization_msgs::Marker::ADD;  //动作
    //设置标记的颜色
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.2f;
    //设置标记的位姿
    marker.pose = pose.pose;
    // printf("%f\n", marker.pose.position.z);
    // marker.pose.position.z = 0.4;
    //设置标记的尺寸
    marker.scale.x = scale.x;
    marker.scale.y = scale.y;
    marker.scale.z = 0.2;
    //标记的id
    marker.id = 0;
    markers.markers.push_back(marker);  //保存
    
    //arrow配置
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0f;
    marker.scale.x = 0.5 * scale.x;
    marker.scale.y = 0.2* scale.y;
    marker.id = 1;  //设置箭头标记的ID
    markers.markers.push_back(marker);  //标记也添加到markers数组

    markers_pub.publish(markers);  //发布
}

//点云聚类
visualization_msgs::MarkerArray Clould_Cluster(pcl::PointCloud<pcl::PointXYZI> cp, float cluster_distance, int cluster_number_min, int cluster_number_max)
{
    visualization_msgs::MarkerArray markerarray;
    if (cp.size() <= 2)  return markerarray; // 点数过少，直接返回空数据

    // 建立KdTree对象用来搜索
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cp.makeShared());

    vector<pcl::PointIndices> cluster_indices;  //存储找到的聚类结果
    //欧几里得距离的聚类
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(cluster_distance); // 设置聚类最小值，较小的值可能会导致对象被划分为多个簇，而较大的值可能会连接同一簇中的对象
    ec.setMinClusterSize(cluster_number_min); // 设置聚类的最小点云数
    ec.setMaxClusterSize(cluster_number_max); // 设置聚类的最大点云数
    ec.setSearchMethod(tree); //设置用于聚类搜索的方法,采用kd_tree
    ec.setInputCloud(cp.makeShared());  //设置输入点云
    // 据之前设置的参数和搜索方法在输入点云中查找聚类，
    //并将找到的聚类索引存储在cluster_indices向量中
    ec.extract(cluster_indices); 
    // ROS_INFO("n==%d\n",cluster_indices.size());

    pcl::PointXYZI minpoint, maxpoint;
    //初始化ROS的可视化标记
    //包括设置标记的帧ID、时间戳、命名空间、生命周期、是否锁定到帧、
    //类型（在这里是立方体）、动作（添加）、颜色（红色，透明度为0.3）和ID
    visualization_msgs::Marker marker;
    marker.header.frame_id = cp.header.frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "lidar_obs";   //
    marker.lifetime = ros::Duration(1);
    marker.frame_locked = true;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.3f;
    marker.id=0;
    //从点云中提取的所有聚类的索引
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud_cluster;
        //使用聚类索引中的点来填充它
        for (auto pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
            pcl::PointXYZI p = cp[*pit];
            cloud_cluster.push_back(p);
        }
        //找到这个聚类点云中所有点在X、Y、Z坐标上的最小值和最大值
        pcl::getMinMax3D(cloud_cluster, minpoint, maxpoint);
        
        //找到的最小和最大点来计算立方体标记的中心位置和大小
        marker.pose.position.x = (maxpoint.x + minpoint.x) / 2;
        marker.pose.position.y = (maxpoint.y + minpoint.y) / 2;
        marker.pose.position.z = (maxpoint.z + minpoint.z) / 2;
        marker.scale.x = maxpoint.x - minpoint.x;
        marker.scale.y = maxpoint.y - minpoint.y;
        marker.scale.z = maxpoint.z - minpoint.z;
        //保存
        markerarray.markers.push_back(marker);
        //更新id
        marker.id++;
    }

    return markerarray;
}
