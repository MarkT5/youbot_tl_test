//ROS helper functions (implementation)
//Authors: S. Diane, A. Novoselsky
//Created: 21.10.2014
 
#include "includes.h"

#define sign(x) ((x<0)?-1:1)

float LIMANG(float a)
{
	const float pi = M_PI, pi2 = pi * 2;
	while (a > pi) a -= pi2;
	while (a < -pi) a += pi2;
	return a;
}
 void rot(float x, float y, float ang, float *x1, float *y1)
{
	float ang0 = (float)atan2(y, x);
	ang += ang0;
	float r = (float)sqrt(x * x + y * y);
	
	float cs = (float)cos(ang), sn = (float)sin(ang);
	*x1 = r * cs;
	*y1 = r * sn;
}



Helper::Helper()
{
	topic_control="/cmd_vel";
	topic_info="/tactical_info";
	//topic_behaviour="/cmd_behaviour";
	topic_manipulator[0]="/arm_1/arm_controller/position_command";
	topic_gripper[0]="/arm_1/gripper_controller/position_command";
	topic_manip_vel[0]="/arm_1/arm_controller/velocity_command";
	topic_manip_state[0]="/arm_1/joint_states";
	topic_manipulator[1]="/arm_2/arm_controller/position_command";
	topic_gripper[1]="/arm_2/gripper_controller/position_command";
	topic_manip_vel[1]="/arm_2/arm_controller/velocity_command";
	topic_manip_state[1]="/arm_2/joint_states";
	topic_objects="objects";
	topic_odom = "/odom";
	topic_odom_wheels = "/joint_states";
	topic_laser="/scan";
	topic_img="/camera/rgb/image_raw";
	armJointPositions.resize(5);
	armJointPositions2.resize(5);
	armJointVelocities.resize(5);
	armJointVelocities2.resize(5);
	gripperJointPositions.resize(2);
	gripperJointPositions2.resize(2);
	
	cntLaser=cntOdom=cntWheels=cntArmPos=0;
	cntImg=0;
	
	laser_rot=0;
	
	manip_inc_ready=false;	
}

void Helper::init(int argc, char **argv){
	
	
	ros::init(argc, argv, "youbot_tl_test");// !!!ПОМЕНЯЛ НАЗВАНИЕ, ЕСЛИ НЕ РАБОТАЕТ ВЕРНУТЬ ОБРАТНО!!!
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
	
    //Ros communications:
    control_publisher = n.advertise<geometry_msgs::Twist>(topic_control, 1000);	
    info_publisher = n.advertise<std_msgs::String>(topic_info, 1000);	
    manipulator_publisher[0] = n.advertise<brics_actuator::JointPositions>(topic_manipulator[0], 1000);
    manipulator_vel_publisher[0] = n.advertise<brics_actuator::JointVelocities >(topic_manip_vel[0], 1000);
    gripper_publisher[0] =  n.advertise<brics_actuator::JointPositions>(topic_gripper[0], 1000);   
    manipulator_publisher[1] = n.advertise<brics_actuator::JointPositions>(topic_manipulator[1], 1000);
    manipulator_vel_publisher[1] = n.advertise<brics_actuator::JointVelocities >(topic_manip_vel[1], 1000);
    gripper_publisher[1] =  n.advertise<brics_actuator::JointPositions>(topic_gripper[1], 1000);
    cube_found=false;
}

static int lastTime=-1;
float last_uV, last_uTheta;

float Helper::angleBetween(float x, float y, float robot_x, float robot_y, float heading)
{
    float v1_x=x-robot_x;
	float v1_y=y-robot_y;
	float v2_x = cos(heading);
	float v2_y = sin(heading);
	
	double vector_angle = atan2(v1_y,v1_x)-atan2(v2_y,v2_x);
	return LIMANG(vector_angle);
}



//ПАРАМЕТРЫ В ЭКРАННЫХ КООРДИНАТАХ
//mode - боком или с поворотом, v - линейная скорость, w - угловая скорость
float Helper::movePoint(cv::Point2f &target, float v, float w, int mode)
{//Движение к точке
	
	float dx = target.x-pose.XA();	
	float dy = target.y-pose.YA();
	float dr = sqrt(dx*dx+dy*dy);
	
	//смещения в СК платформы робота
	float sn=sin(pose.AA()), cs=cos(pose.AA());
	float dy_=-dy*cs+dx*sn;
	float dx_=dx*cs+dy*sn;
	float d_=sqrt(dx_*dx_+dy_*dy_);
	
	if(mode==0 || mode==2) //боком
	{
		float vx=0,vy=0;
		
		vx=v*(dx_/d_);
		vy=v*(dy_/d_);
		
		SendControl(vx, vy, 0);
	}
	else if(mode==1) //с поворотом
	{
		float dtheta = angleBetween(target.x, target.y, pose.XA(), pose.YA(), pose.AA());
		//поворот в СК платформы робота
		float dtheta_=-dtheta;
		
		//нормализованные рассогласования по углу и угл.скорости (целевая угл.скорость = 0)
		float DW=pose.rot_speed/w;
		float DTh=-dtheta_/3.1415926;
		
		float kp=0, kd=0, kv=0;
		
		int I1=0; if(fabs(DTh)>0.1) I1=1; if(fabs(DTh)>0.4) I1=2; 
		int I2=0; if(fabs(DW)>0.1) I2=1; if(fabs(DW)>0.5) I2=2; 
		
		if(I1==0) //рассогласование маленькое - удерживаем низкую скорость (kd >> 0) но стабильную (kp >> 0), + движемся вперед (kv>>0)
		{
			kv=1;	kd=0.5;	kp=3;
		}
		else if(I1==1) //рассогласование среднее - притормаживаем поворот (kd>0)
		{
			kv=0.5;
			kd=0.2;
			if(I2==0) kp=2;
			else if(I2==1) kp=1; 
			else if(I2==2) kp=0;	
		}
		else if(I1==2)  //рассогласование большое - поворачиваем максимально быстро на месте
		{
			kv=0; kd=0; kp=1; 	
		}
		
		
		float uTheta = -(kp*DTh + kd*DW); uTheta=std::max(-w, std::min(w, uTheta));
		float uV = kv*v;
		
		int Time=(int)time(NULL);
		
		if(Time-lastTime<2) //сглаживание изменений в управлении
		{
			float a=0.3f, b=1-a;
			last_uV=uV=a*uV+b*last_uV;
			last_uTheta=uTheta=a*uTheta+b*last_uTheta;
		}
		
		SendControl(uV,0,uTheta);
		
		if(_DEBUG_)
		{
			//printf("I1 %d, I2 %d, kp %f, kd %f, kv %f, DTh %f, DW %f, uV %f, uTheta %f\r\n",I1, I2,kp,kd,kv, DTh, DW, uV, uTheta);
			//printf("x %f, y %f, tx %f, ty %f, theta %f, dtheta %f\r\n",robotX, robotY, target.x, target.y, theta, dtheta);
		}
		
		
	}
	
	// printf("dx %f, dy %f, dr %f, dtheta %f, theta %f\r\n",dx,dy,dr,dtheta,theta);
	// printf("vx %f, dr %f\r\n",vx,dr);
	// printf("dtheta %f, vtheta %f\r\n",dtheta,vtheta);
	
	
	return dr;
	
	//float t=modf(time,60);
	
}
void Helper::InitCallbacks()
{
	ros::NodeHandle n;
    	odom_subscriber=n.subscribe(topic_odom, 1000, &Helper::odomCallback, this);
    	odom_wheels_subscriber=n.subscribe(topic_odom_wheels, 1000, &Helper::odomWheelsCallback, this);
	laser_subscriber=n.subscribe(topic_laser, 1000, &Helper::LaserScanCallback, this);
	manip_subscriber[0]=n.subscribe(topic_manip_state[0], 1000, &Helper::manip1PosCallback, this);
	manip_subscriber[1]=n.subscribe(topic_manip_state[1], 1000, &Helper::manip2PosCallback, this);
	
	
} 

Helper::~Helper()
{
	if(ros::isStarted()) {
		// cmd_vel_msg.linear.x=0; 
		// cmd_vel_msg.linear.y=0;
		// cmd_vel_msg.angular.z=0;
		ros::shutdown(); // explicitly needed since we use ros::start();
		ros::waitForShutdown();
	}
    wait();
}


bool Helper::isInRange(float mi, float ma, float val){
	if(mi < val && val < ma){
		return true;
	}else{
		return false;
	}
}

void Helper::manip1PosCallback(const sensor_msgs::JointState::ConstPtr& msg){
	manipPosCallbackHandler(0, msg);
}

void Helper::manip2PosCallback(const sensor_msgs::JointState::ConstPtr& msg){
	manipPosCallbackHandler(1, msg);
}

void Helper::manipPosCallbackHandler(int ID, const sensor_msgs::JointState::ConstPtr& msg){
	float k=180/M_PI;
	const std::vector<double> pos = msg->position;
	char str[200];
	sprintf(str, ".manip%i#%f;%f;%f;%f;%f", ID, pos[0]*k, pos[1]*k, pos[2]*k, pos[3]*k, pos[4]*k);
	SendInfo(str);
	//std::cout<< str <<"\n";
	//std::cout<< (!(isInRange(11, 302, pos[0]*k) && isInRange(3, 150, pos[1]*k) && isInRange(-260, -15, pos[2]*k)  && isInRange(10, 195, pos[3]*k) && isInRange(21, 292, pos[4]*k))) << "\n";
    //if (!(isInRange(11, 302, pos[0]*k) && isInRange(3, 150, pos[1]*k) && isInRange(-260, -15, pos[2]*k)  && isInRange(10, 195, pos[3]*k) && isInRange(21, 292, pos[4]*k))){
        //SetManipulator(ID, 0, 0 ,0 ,0 ,0);
        //std::cout<< "pizdec\n";
        //SetManipulatorVel(ID, 0, 0, 0, 0, 0);
    //}
    
}

void Helper::odomWheelsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	cntWheels++;
	if(cntWheels%5==0)
	{
		const std::vector<double> pos = msg->position;
		char str[200];
		sprintf(str, ".wheels#%lf;%lf;%lf;%lf", pos[0], pos[1], pos[2], pos[3]);
		SendInfo(str);
	}
}
void Helper::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	cntOdom++;
	if(cntOdom%5==0)
	{
	pose.UpdateOdometryPose(
		msg->pose.pose.position.x, 
		msg->pose.pose.position.y, 
		tf::getYaw(msg->pose.pose.orientation), 
		msg->twist.twist.angular.z
	);
	
	
		// printf("Odom recieved (5), pose x=%f ,y=%f, a=%f\n",robotX,robotY, theta*180/3.14);

		
		//msg->postion;
		std::stringstream ss_odom;
		ss_odom << ".odom#";
		ss_odom << pose.XA();
		ss_odom << ";";
		ss_odom << pose.YA();
		ss_odom << ";";
		ss_odom << pose.AA();
		ss_odom << "\r\n";

		ss_odom << ".odomspeed#";
		ss_odom << msg->twist.twist.linear.x;
		ss_odom << ";";
		ss_odom << msg->twist.twist.linear.y;
		ss_odom << ";";
		ss_odom << msg->twist.twist.angular.z;
		
		//sprintf(str, ".odom#%f;%f;%f",  pose.XA(), pose.YA(), pose.AA());
		//sprintf(str, ".odom#%f;%f;%f",  pose.XA(), pose.YA(), pose.AA());
		//sprintf(str, ".odomspeed#%f;%f;%f",  msg->twist.twist.linear.x,  msg->twist.twist.linear.y,  msg->twist.twist.angular.z);
		const std::string& tmp = ss_odom.str();
		const char* cstr = tmp.c_str();
		//printf(cstr);
		SendInfo(cstr);
	}
}


void Helper::get_laserscan_integral_value(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	float L=0, R=0; //усредненные интенсивности препятствий слева и справа
	float m=msg->ranges.size()/2.0;
	
	float dmin=0.1, dmax=1;
	//float br_max=1/dmin, br_min=1/dmax;
	
	int cnt=0;
	float danger_max=0;
	for(int i=0;i<msg->ranges.size();i++)
	{
		float x=msg->ranges[i];
		
		if(!(msg->range_min <= msg->ranges[i] && msg->ranges[i] <= msg->range_max)) x=msg->range_max; //Inf, NaN
		
		x=max(dmin, min(dmax, x));
		float br=1/x;
		
		float danger=1 - (x-dmin)/(dmax-dmin);
		if(danger>danger_max) danger_max=danger;
	
		if(i<m) L+=br;
		if(i>m) R+=br;
	
		cnt++;
	}
	
	
	laser_rot=(L-R)/(R+L)*(-1);
	laser_danger=danger_max;
}

void Helper::LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	cntLaser++;
	
	//if(cntLaser%10==0) //чтоб отсылалось не очень часто
	//{
		std::stringstream ss;
		ss.precision(4);
		ss << ".laser#";	
		//msg->ranges - Массив Float32 (в Vector`e)
		
		for(int i=0;i<msg->ranges.size();i++)
		{
			float x=msg->ranges[i];		
			if(!(msg->range_min <= msg->ranges[i] && msg->ranges[i] <= msg->range_max)) x=msg->range_max; //Inf, NaN
			
			ss<<x<<";";
		}
	
		const std::string& tmp = ss.str();
		const char* cstr = tmp.c_str();
		SendInfo(cstr);
		cntLaser=0;	

		//std::stringstream strs;
		//strs << tmp.size();
		//const std::string temp_str = strs.str();
		//const char* ss_size = (char*) temp_str.c_str();
		//printf(tmp.c_str());
	
	//}
	//else if(cntLaser%10==4)	
	//{
	//	get_laserscan_integral_value(msg);
	//	hokuyoRanges=msg->ranges;
	//	char str[20];
	//	sprintf(str, ".laser_mini#%.3f",  laser_rot);
	//	SendInfo(str);
		
		//  SendInfo("hi");
		
	//}
	
}

void Helper::SendControl(float fwd, float left, float rotLeft)
{	
	geometry_msgs::Twist cmd_vel_msg;
	
	cmd_vel_msg.linear.x=fwd;
    cmd_vel_msg.linear.y=left;
    cmd_vel_msg.angular.z=rotLeft;
	
	
	control_publisher.publish(cmd_vel_msg);
}


//Joint values in radians
void Helper::SetManipulator(int ID,float j1, float j2, float j3,float j4, float j5)
{	
	char uri_arr[5][30];
	for(int i=0;i<5;i++)
	{
		std::stringstream ss;
		if(ID==0) ss<<"arm_joint_"; else if(ID==1) ss<<"arm_2_joint_";
		ss<<(i+1);
		strcpy(uri_arr[i], ss.str().c_str());
	}
	
    brics_actuator::JointPositions msg;
    
    std::vector <brics_actuator::JointValue>& ajp=armJointPositions;
    if(ID==1) ajp=armJointPositions2;
    
    ajp[0].joint_uri = uri_arr[0];
    ajp[0].value = j1;
    ajp[0].unit = "rad";
    ajp[1].joint_uri = uri_arr[1];
    ajp[1].value = j2;
    ajp[1].unit = "rad";
    ajp[2].joint_uri = uri_arr[2];
    ajp[2].value = j3;
    ajp[2].unit = "rad";
    ajp[3].joint_uri = uri_arr[3];
    ajp[3].value = j4;
    ajp[3].unit = "rad";
    ajp[4].joint_uri = uri_arr[4];
    ajp[4].value = j5;
    ajp[4].unit = "rad";
    msg.positions = ajp;
	
	
    manipulator_publisher[ID].publish(msg);
	manip_inc_ready=true;
	
}


void Helper::SetManipulatorVel(int ID, float j1, float j2, float j3,float j4, float j5)
{	
std::cout<< "1\n";
	char uri_arr[5][30];
	for(int i=0;i<5;i++)
	{
		std::stringstream ss;
		if(ID==0) ss<<"arm_joint_"; else if(ID==1) ss<<"arm_2_joint_";
		ss<<(i+1);
		strcpy(uri_arr[i], ss.str().c_str());
	}
	std::cout<< "2\n";
	
    brics_actuator::JointVelocities msg;
	
    std::vector <brics_actuator::JointValue>& ajv=armJointVelocities;
    if(ID==1) ajv=armJointVelocities2;
    
    
    std::cout<< "3\n";
    
    ajv[0].joint_uri = uri_arr[0];
    ajv[0].value = j1;
    ajv[0].unit = "s^-1 rad";
    ajv[1].joint_uri = uri_arr[1];
    ajv[1].value = j2;
    ajv[1].unit = "s^-1 rad";
    ajv[2].joint_uri = uri_arr[2];
    ajv[2].value = j3;
    ajv[2].unit = "s^-1 rad";
    ajv[3].joint_uri = uri_arr[3];
    ajv[3].value = j4;
    ajv[3].unit = "s^-1 rad";
    ajv[4].joint_uri = uri_arr[4];
    ajv[4].value = j5;
    ajv[4].unit = "s^-1 rad";
    std::cout<< "4\n";
    msg.velocities = ajv;
	
	std::cout<< "5\n";
    manipulator_vel_publisher[ID].publish(msg);
    std::cout<< "6\n";
	
}

//поворот звена из текущего положения
void Helper::IncrementJoint(int ID,int joint, float val)
{	
	if(!manip_inc_ready)
	{
		printf("Can't increment joint. Set manipulator initial position first.\n");
		return;
	}
	
	std::vector <brics_actuator::JointValue>& ajp=armJointPositions;
	if(ID==1) ajp=armJointPositions2;
	
	ajp[joint].value += val;
	
    brics_actuator::JointPositions msg;
	msg.positions = ajp;
	
    manipulator_publisher[ID].publish(msg);	
}

//Gripper delta in cm
void Helper::SetGripper(int ID,float delta)
{
	if(delta>=0 && delta<=2){
	
	char uri_arr[2][30];
	for(int i=0;i<2;i++)
	{
		std::stringstream ss;
		if(ID==0) ss<<"gripper_finger_joint_"; else if(ID==1) ss<<"gripper_2_finger_joint_";
		ss<<(i==0?"l":"r");
		strcpy(uri_arr[i], ss.str().c_str());
	}
	
		brics_actuator::JointPositions msg;
		gripperJointPositions[0].joint_uri = uri_arr[0];
		gripperJointPositions[0].value = delta/200;
		gripperJointPositions[0].unit = "m";
		gripperJointPositions[1].joint_uri = uri_arr[1];
		gripperJointPositions[1].value = delta/200;
		gripperJointPositions[1].unit = "m";
		msg.positions = gripperJointPositions;
		
		
		gripper_publisher[ID].publish(msg);		
	}    
}

void Helper::SendInfo(const char* info)
{
	//old ROS sending code...
    // std_msgs::String msg;
	// std::stringstream ss;
	// ss << info;
	// msg.data = ss.str();	
    // info_publisher.publish(msg);	
	
	if(ts!=NULL && ts->NumConnections()>0)
	{
		//ts->SendFirst(info);
		ts->SendAll(info);
	}
}
