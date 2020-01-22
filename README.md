# laser_deskew
ros package for 2d laser deskew using tf.

## IO

- input  
/scan(sensor_msgs/LaserScan)  
/tf(odom link and laser lnk)

- output  
/deskewed_scan(sensor_msgs/LaserScan)

## param
- laser_frame_id(defalut:"laser_link")