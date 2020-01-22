#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <sensor_msgs/LaserScan.h>

#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>

class LaserDeskew {
public:
    LaserDeskew(tf::TransformListener* tf);
    ~LaserDeskew();

    void ScanCallBack(const sensor_msgs::LaserScanPtr& scan_msg);

    bool getLaserPose(
        tf::Stamped<tf::Pose>& odom_pose,
        ros::Time dt,
        tf::TransformListener* tf_);

    void calibrateLaserMotion(
        std::vector<float>& ranges,
        std::vector<double>& angles,
        tf::Stamped<tf::Pose> base_pose,
        tf::Stamped<tf::Pose> start_pose,
        tf::Stamped<tf::Pose> end_pose,
        int start_index,
        int& beam_number);

    void calibrateLaser(
        std::vector<float>& ranges,
        std::vector<double>& angles,
        ros::Time startTime,
        ros::Time endTime,
        tf::TransformListener* tf_);

    void publishScan(
        const sensor_msgs::LaserScanPtr& scan_msg,
        std::vector<float>& ranges,
        std::vector<double>& angles);

public:
    tf::TransformListener* tf_;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher deskewed_scan_pub_;

    std::string laser_frame_id_;
};