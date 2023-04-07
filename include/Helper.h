//ROS helper functions
//Authors: S. Diane, ...
//Created: 21.10.2014



 typedef void (*Topic_behaviour_callback)(const std_msgs::String& incoming_msg );
 //typedef void (*Topic_objects_callback)(const std_msgs::Float32MultiArray& incoming_msg );
 //typedef void (*Topic_odom_callback)(const geometry_msgs::Pose& incoming_msg );
 
template <typename T> float sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
float LIMANG(float a);
 void rot(float x, float y, float ang, float *x1, float *y1);
		
   class Pos2d
    {
        public:
		float x, y;
        float a;
		
		void Zero() { a = x = y = 0; }
        Pos2d()
        {
            Zero();
        }
        Pos2d(float x, float y, float a)
        {
            this->a = a; this->x = x; this->y = y;
        }
        
     

        void set_ang(float ang){a = LIMANG(ang);}
        float get_ang() { return a; }
        void inc_ang(float da) { set_ang(a + da); }
    };
class Pose
{	     
        public:

        Pos2d odom_c;//текущее положение по одометрии в СК робота
        Pos2d odom_a;//текущее положение по одометрии в экранной СК
        Pos2d odom_a0;//начальная точка и ориентация одометрии в экранной СК
		float rot_speed;
	
	Pose()
	{
		odom_c = Pos2d();
		odom_a = Pos2d();
		odom_a0 = Pos2d();
		rot_speed=0;
	} 
	
	float XA() { return odom_a.x + odom_a0.x; }
	float YA() { return odom_a.y + odom_a0.y; }
	float AA() { return odom_a.get_ang() + odom_a0.get_ang(); }
		
	
	void UpdateOdometryPose(float xc, float yc, float ac, float w)
	{
		rot_speed=w;
		
		float dxc = xc-odom_c.x;
		float dyc = yc-odom_c.y;
		float dac = ac-odom_c.get_ang();

		float dxb, dyb;
		rot(dxc, dyc, -odom_c.get_ang()-AA(), &dxb, &dyb);

		odom_a.x += dxb; odom_a.y -= dyb; odom_a.inc_ang(-dac);

		odom_c.x = xc;
		odom_c.y = yc;
		odom_c.set_ang(ac);
	}

  void UpdateScreenPose(float xa, float ya, float aa, float miu)
	{
	
	//НЕ РАБОТАЕТ, т.к. одометрия драйвера робота не обнуляется
		// if (miu > 0.99f) //жесткое обновление с заданием начальной точки отсчета одометрии
		// {
			// odom_a0.x = xa;
			// odom_a0.y = ya;
			// odom_a0.set_ang(aa);

			// odom_a.Zero();
			// odom_c.Zero();
		// }
		// else
			//обновление с мягкой (экспоненциальной) коррекцией координат
		{
			float dx=xa - XA();
			float dy=ya - YA();
			float da=LIMANG(aa - AA());
			
			if(miu<0.999999)
			{
				if(abs(dx)>1 || abs(dy)>1 || abs(da)>1)
				{
					printf("Pose update is too big dx=%f, dy=%f, da=%f\n",dx,dy,da);
					return;
				}
			}
			
			odom_a0.x += miu * dx;
			odom_a0.y += miu * dy;
			odom_a0.inc_ang( miu * da);
		}
	}
	

};
 
class Helper
{
public:

TcpServer* ts;

ros::Publisher control_publisher;
std::string topic_control;

ros::Publisher info_publisher;
std::string topic_info;

ros::Publisher manipulator_publisher[2];//Команды на манипулятор
std::string topic_manipulator[2];

ros::Publisher manipulator_vel_publisher[2];//Команды на манипулятор
std::string topic_manip_vel[2];

ros::Publisher gripper_publisher[2];//Команды на схват
std::string topic_gripper[2];

ros::Subscriber behaviour_subscriber;
std::string topic_behaviour;

ros::Subscriber objects_subscriber;//массив найденных объектов
std::string topic_objects;

ros::Subscriber odom_subscriber; //Данные одометрии
std::string topic_odom;

ros::Subscriber odom_wheels_subscriber; //Данные одометрии (колёса)
std::string topic_odom_wheels;

ros::Subscriber img_subscriber; //Данные камеры
std::string topic_img;

ros::Subscriber manip_subscriber[2]; //Данные камеры
std::string topic_manip_state[2];

 
Pose pose; //координаты робота


ros::Subscriber laser_subscriber; //Данные лазерного дальномера
std::string topic_laser;

volatile float laser_rot; //напраление поворота на основе анализа лазерного дальномера (+вправо -влево)
volatile float laser_danger; //опасность движения согласно показаниям лазерного дальномера

int cntLaser,cntOdom,cntArmPos, cntWheels;
int cntImg;

Topic_behaviour_callback tbc;
//Topic_odom_callback odomCallback;
//Topic_objects_callback objectsDetectedCallback;

std::vector <brics_actuator::JointValue> armJointPositions;
std::vector <brics_actuator::JointValue> armJointPositions2;
std::vector <brics_actuator::JointValue> armJointVelocities;
std::vector <brics_actuator::JointValue> armJointVelocities2;
bool manip_inc_ready;

std::vector <brics_actuator::JointValue> gripperJointPositions;
std::vector <brics_actuator::JointValue> gripperJointPositions2;
std::vector<float> hokuyoRanges;
bool cube_found;//Flag for cube detection
std::vector<cv::Point2f> path;//path of robot

Helper();//конструктор, инициализация
~Helper();//деструктор
void init(int argc, char **argv);
float angleBetween(float x, float y, float pt_x, float pt_y, float heading);//Угол на точку
bool isInRange(float mi, float ma, float val);
void InitCallbacks(); //вторая часть инициализации
void SendControl(float fwd, float left, float rotLeft); //команда на движение робота
void SetManipulator(int ID,float j1, float j2, float j3,float j4, float j5);//команды на звенья манипулятора (в радианах)
void SetManipulatorVel(int ID,float j1, float j2, float j3,float j4, float j5);//команды скорости на звенья манипулятора (в радианах)
void IncrementJoint(int ID,int joint, float val);//поворот звена из текущего положения
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
void odomWheelsCallback(const sensor_msgs::JointState::ConstPtr& msg);
void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void manipPosCallbackHandler(int ID, const sensor_msgs::JointState::ConstPtr&msg);
void manip1PosCallback(const sensor_msgs::JointState::ConstPtr& msg);
void manip2PosCallback(const sensor_msgs::JointState::ConstPtr& msg);
float movePoint(cv::Point2f &target, float v, float w, int mode);//Движение к точке
void SetGripper(int ID,float delta);//команды на схват в см
void SendInfo(const char* info); //обратная связь с клиентским приложением

private:

void get_laserscan_integral_value(const sensor_msgs::LaserScan::ConstPtr& msg );
};
