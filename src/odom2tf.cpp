#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = msg->header.stamp;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = msg->pose.pose.position.x;
    odom_trans.transform.translation.y = msg->pose.pose.position.y;
    odom_trans.transform.translation.z = msg->pose.pose.position.z;
    odom_trans.transform.rotation = msg->pose.pose.orientation;

    static tf::TransformBroadcaster odom_broadcaster;
    odom_broadcaster.sendTransform(odom_trans);
}

int main(int argc, char** argv)
{
    
    ros::init(argc, argv, "odom2tf");
    
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("base_pose_ground_truth", 10, odometryCallback);
    while(nh.ok()){
        ros::spin();
    }
    return 0;
}