#include "laser_deskew.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_deskew");

    tf::TransformListener tf(ros::Duration(10.0));

    LaserDeskew tmp(&tf);

    ros::spin();
    return 0;
}
