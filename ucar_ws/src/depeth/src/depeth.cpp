
#include "ros/ros.h"
#include <iostream>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/thread.hpp>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

using namespace message_filters;

void LaserScanToPointCloud(sensor_msgs::LaserScan::ConstPtr _laser_scan, pcl::PointCloud<pcl::PointXYZ> &_pointcloud)
{
    _pointcloud.clear();
    pcl::PointXYZ newPoint;
    newPoint.z = 0.0;
    double newPointAngle;

    int beamNum = _laser_scan->ranges.size();
    for (int i = 0; i < beamNum; i++)
    {
        newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i;
        newPoint.x = _laser_scan->ranges[i] * cos(newPointAngle);
        newPoint.y = _laser_scan->ranges[i] * sin(newPointAngle);

        _pointcloud.push_back(newPoint);
    }
}

//sensor_msgs::ImagePtr depeth_msg;
void my_callback(const sensor_msgs::LaserScan::ConstPtr &scan, const sensor_msgs::Image::ConstPtr &cam,
                 const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
     std::cout<<"ccc"<<std::endl;
    // pcl::PCLPointCloud2 pcl_pc2;
    // pcl_conversions::toPCL(*scan, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromPCLPointCloud2(pcl_pc2, *pc_cloud);
    LaserScanToPointCloud(scan, *pc_cloud);

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(cam, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    Eigen::Matrix<double, 3, 4> T;
    Eigen::Matrix3d R;
    Eigen::Quaterniond q(pose->pose.pose.orientation.x, pose->pose.pose.orientation.y, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);
    //单位化
    Eigen::Quaterniond qu = q.normalized();
    R = qu.matrix();
    T(0, 0) = R(0, 0);
    T(0, 1) = R(0, 1);
    T(0, 2) = R(0, 2);
    T(1, 0) = R(1, 0);
    T(1, 1) = R(1, 1);
    T(1, 2) = R(1, 2);
    T(2, 0) = R(2, 0);
    T(2, 1) = R(2, 1);
    T(2, 2) = R(2, 2);

    T(0, 3) = pose->pose.pose.position.x;
    T(1, 3) = pose->pose.pose.position.y;
    T(2, 3) = pose->pose.pose.position.z;

    cv::Mat depeth;
    cv::cvtColor(img, depeth, cv::COLOR_BGR2GRAY);

    Eigen::Matrix3d K;
    K << 430.6716997374398, 0, 322.4443114387696, 0, 429.7690419166964, 225.9135138888588, 0, 0, 1;

    Eigen::Matrix<double, 3, 4> RT1_TOP3, RT1_X_CAM;
    RT1_TOP3 << 9.8073484448069070e-03, -1.6167992715479323e-01,
        9.8679451613378066e-01, -4.4249354679139020e-02,
        9.9985809304929507e-01, 1.5103070938360830e-02,
        -7.4626410912970731e-03, 5.2701723333894390e-02,
        -1.3697068310730365e-02, 9.8672767185452503e-01,
        1.6180510472861998e-01, -8.9300815951665238e-02;
    RT1_X_CAM = K * RT1_TOP3;

    for (int pp = 0; pp < pc_cloud->size(); pp++)
    {
        Eigen::Vector4d raw1_point;
        Eigen::Vector3d trans1_point3;
        pcl::PointXYZ r;
        r = pc_cloud->points[pp];

        raw1_point(0, 0) = r.x;
        raw1_point(1, 0) = r.y;
        raw1_point(2, 0) = r.z;
        raw1_point(3, 0) = 1;
        trans1_point3 = RT1_X_CAM * raw1_point;
        int x1 = (int)(trans1_point3(0, 0) / trans1_point3(2, 0));
        int y1 = (int)(trans1_point3(1, 0) / trans1_point3(2, 0));
        if (x1 < 0 || x1 > (img.cols - 1) || y1 < 0 || y1 > (img.rows - 1))
            continue;
        else
        {

            for (int j = 0; j < img.rows; j++)
            {
                depeth.at<uchar>(j, x1) = trans1_point3(2, 0);
            }
        }
    }


    int x=320;
    int y=240;
    //图像识别，确定图像中某一个点
    Eigen::Vector4d point_cam(x, y, depeth.at<uchar>(y, x),1);
    Eigen::Vector3d point_world=T*point_cam;
    std::cout<<point_world<<std::endl;
    // depeth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", depeth).toImageMsg();
}
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "listener");

    // 创建节点句柄
    ros::NodeHandle n;
    // 需要用message_filter容器对两个话题的数据发布进行初始化，这里不能指定回调函数
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_scan(n, "/scan", 2000, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, "/ht_image_biaoding_view/ht_image_biaoding_raw", 2000, ros::TransportHints().tcpNoDelay());
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub_pose(n, "/amcl_pose", 2000, ros::TransportHints().tcpNoDelay());
 std::cout<<"aaaa"<<std::endl;
    // 将2个话题的数据进行同步
    typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image, geometry_msgs::PoseWithCovarianceStamped> syncPolicy;
    Synchronizer<syncPolicy> sync(syncPolicy(1000), sub_scan, sub_image, sub_pose);
    // 指定一个回调函数，就可以实现2个话题数据的同步获取
    sync.registerCallback(boost::bind(&my_callback, _1, _2,_3));
 std::cout<<"bbb"<<std::endl;
    // image_transport::ImageTransport it(n);
    // image_transport::Publisher pub = it.advertise("/depeth_img", 1);

    // ros::Rate rate(10.0);
    // while (ros::ok())
    // {

    //     pub.publish(depeth_msg);
    //     ros::spinOnce();

    //     rate.sleep();
    // }
    //若解除上面的注释，则注释掉下面的ros::spin();
    ros::spin();
    return 0;
}
