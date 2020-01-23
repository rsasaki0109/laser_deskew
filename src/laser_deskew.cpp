#include "laser_deskew.h"

    LaserDeskew::LaserDeskew(tf::TransformListener* tf)
    {
        tf_ = tf;
        nh_.getParam("laser_link", laser_frame_id_ );
        scan_sub_ = nh_.subscribe("base_scan", 10, &LaserDeskew::ScanCallBack, this);
        deskewed_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/deskewed_scan", 10);
    }

    LaserDeskew::~LaserDeskew()
    {}

    void LaserDeskew::ScanCallBack(const sensor_msgs::LaserScanPtr& scan_msg)
    {
        ros::Time startTime = scan_msg->header.stamp;
        sensor_msgs::LaserScan laserScanMsg = *scan_msg;
        int beam_num = laserScanMsg.ranges.size();
        ros::Time endTime = startTime + ros::Duration(laserScanMsg.time_increment * beam_num);

        std::vector<double> angles;
        std::vector<float> ranges;
        for (int i = beam_num-1; i > 0; i--) {
            float laser_dist = laserScanMsg.ranges[i];
            double laser_angle = scan_msg->angle_min + scan_msg->angle_increment * i;
            
            if (std::isnan(laser_dist) || std::isinf(laser_dist))
                laser_dist = 0.0;

            ranges.push_back(laser_dist);
            angles.push_back(laser_angle);
        }

        calibrateLaser(ranges, angles, startTime, endTime,tf_);

        publishScan(scan_msg, ranges, angles);
        
    }

    bool LaserDeskew::getLaserPose(
        tf::Stamped<tf::Pose>& odom_pose,
        ros::Time dt,
        tf::TransformListener* tf_)
    {
        odom_pose.setIdentity();

        tf::Stamped<tf::Pose> robot_pose;
        robot_pose.setIdentity();
        robot_pose.frame_id_ = laser_frame_id_;
        robot_pose.stamp_ = dt;

        try {
            if (!tf_->waitForTransform("/odom", laser_frame_id_, dt, ros::Duration(0.5))) 
            {
                ROS_ERROR("Cannot WaitTransform");
                return false;
            }
            tf_->transformPose("/odom", robot_pose, odom_pose);
        } catch (tf::LookupException& ex) {
            ROS_ERROR("%s\n", ex.what());
            return false;
        } catch (tf::ConnectivityException& ex) {
            ROS_ERROR("%s\n", ex.what());
            return false;
        } catch (tf::ExtrapolationException& ex) {
            ROS_ERROR("%s\n", ex.what());
            return false;
        }

        return true;
    }

    void LaserDeskew::calibrateLaserMotion(
        std::vector<float>& ranges,
        std::vector<double>& angles,
        tf::Stamped<tf::Pose> base_pose,
        tf::Stamped<tf::Pose> start_pose,
        tf::Stamped<tf::Pose> end_pose,
        int start_index,
        int& beam_number)
    {
        tf::Quaternion start_quat = start_pose.getRotation();
        tf::Quaternion end_quat = end_pose.getRotation();
        tf::Vector3 start_xyz(start_pose.getOrigin().getX(), start_pose.getOrigin().getY(), 1);
        tf::Vector3 end_xyz(end_pose.getOrigin().getX(), end_pose.getOrigin().getY(), 1);

        for (size_t i = start_index; i < start_index + beam_number; i++) {
            tf::Vector3 mid_xyz = start_xyz.lerp(end_xyz, (i - start_index) / (beam_number - 1));
            tf::Quaternion mid_quat = start_quat.slerp(end_quat, (i - start_index) / (beam_number - 1));
            tf::Transform mid_frame(mid_quat, mid_xyz);
            double x = ranges[i] * cos(angles[i]);
            double y = ranges[i] * sin(angles[i]);
            tf::Vector3 calibrated_point = base_pose.inverse() * mid_frame * tf::Vector3(x, y, 1);
            ranges[i] = sqrt(calibrated_point[0] * calibrated_point[0] + calibrated_point[1] * calibrated_point[1]);
            angles[i] = atan2(calibrated_point[1], calibrated_point[0]);
        }
    }

    void LaserDeskew::calibrateLaser(
        std::vector<float>& ranges,
        std::vector<double>& angles,
        ros::Time startTime,
        ros::Time endTime,
        tf::TransformListener* tf_)
    {
        int beam_number = angles.size();

        int interpolation_time_duration = 5 * 1000;//[ms]
        tf::Stamped<tf::Pose> frame_start_pose;
        tf::Stamped<tf::Pose> frame_mid_pose;
        tf::Stamped<tf::Pose> frame_base_pose;
        tf::Stamped<tf::Pose> frame_end_pose;

        double start_time = startTime.toSec() * 1000 * 1000;//[us]
        double end_time = endTime.toSec() * 1000 * 1000;
        double time_inc = (end_time - start_time) / beam_number; 

        int start_index = 0;

        if (!getLaserPose(frame_start_pose, ros::Time(start_time / (1000 * 1000)), tf_)) {
            ROS_WARN("Not get StartPose");
            return;
        }

        if (!getLaserPose(frame_end_pose, ros::Time(end_time / (1000 * 1000)), tf_)) {
            ROS_WARN("Not get EndPose");
            return;
        }

        int cnt = 0;
        frame_base_pose = frame_start_pose;
        for (int i = 0; i < beam_number; i++) {
            double mid_time = start_time + time_inc * (i - start_index);
            if (mid_time - start_time > interpolation_time_duration || (i == beam_number - 1)) {
                cnt++;

                if (!getLaserPose(frame_mid_pose, ros::Time(mid_time / 1000000.0), tf_)) {
                    ROS_ERROR("Mid %d Pose Error", cnt);
                    return;
                }

                int interp_cnt = i - start_index + 1;

                calibrateLaserMotion(
                    ranges,
                    angles,
                    frame_base_pose,
                    frame_start_pose,
                    frame_mid_pose,
                    start_index,
                    interp_cnt);

                start_time = mid_time;
                start_index = i;
                frame_start_pose = frame_mid_pose;
            }
        }
    }

    void LaserDeskew::publishScan(
        const sensor_msgs::LaserScanPtr& scan_msg,
        std::vector<float>& ranges,
        std::vector<double>& angles)
    {
        sort( ranges.begin(), ranges.end() );
        float range_max = ranges.back();
        float range_min = ranges.front();

        sensor_msgs::LaserScan deskewed_scan_msg;
        deskewed_scan_msg.header = scan_msg->header;
        deskewed_scan_msg.angle_min = angles[0];
        deskewed_scan_msg.angle_max  = angles[angles.size()-1];
        deskewed_scan_msg.time_increment = scan_msg->time_increment;
        deskewed_scan_msg.scan_time = scan_msg->scan_time;
        deskewed_scan_msg.range_min = range_min;
        deskewed_scan_msg.range_max = range_max;
        deskewed_scan_msg.ranges = ranges;
        deskewed_scan_msg.intensities = scan_msg->intensities;
        double angle_min = angles[0];
        double angle_max  = angles[angles.size()-1];
        if (angle_max > angle_min) {
            angle_min =  M_PI - angle_max;
            angle_max =  M_PI - angle_min;
        } 
        else {
            angle_min =  M_PI - angle_min;
            angle_max =  M_PI - angle_max;
        }
        deskewed_scan_msg.angle_increment =(angle_max - angle_min)/(double)(angles.size()-1);
        deskewed_scan_pub_.publish(deskewed_scan_msg);
    }
