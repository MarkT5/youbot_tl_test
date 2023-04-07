//Including necessary files
//Authors: S. Diane, ...
//Created: 21.10.2014

#ifndef INCLUDES_
#define INCLUDES_

#include <iostream>
#include <sstream>
#include <string>

#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32MultiArray.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/JointState.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>


#include <time.h>
#include <math.h>
#include <cmath>
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitions
#include <time.h>   // time calls
#include <iostream>
#include <cstring>
#include <sys/types.h>
#include <ifaddrs.h>
#include <netinet/in.h> 
#include <arpa/inet.h>
#include <cstdlib>
//beep
#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/kd.h> 

#include <stdlib.h>
#include <sys/socket.h>
#include <pthread.h>    // POSIX Threads
#include <signal.h> // for pthread_kill
#include <netdb.h> // getprotobyname


#include <inttypes.h>
#include<map>

#include <vector>

#include "tcp_server.h"
#include "Helper.h"

#define _DEBUG_ 1

#endif
