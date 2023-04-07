//Youbot tl test

#include "includes.h"

static bool received_TCP_MSG=false;
char TCP_MSG[10000]; // команда Lua (принимается извне)


string firstIP;
bool firstConn=false;
Helper hlp;


void OnReceive(MSG* msg)//в этой функции можно сделать обработчик поступающих команд и Lua-скриптов
{
//printf("main.cpp: OnReceive (%s): %s\n", msg->ip, msg->data);

if(!firstConn)
{
	firstConn=true;
	firstIP=msg->ip;
}

	strcpy(TCP_MSG, msg->data);
	received_TCP_MSG=true;
	char s[100];
	sprintf(s, "main.cpp: Send, last_ttl=%f\nsecond line\n",msg->last_ttl);
}

void OnEndConnection(const char* ip)//в этой функции можно сделать обработчик разрыва соединения
{
	if(strcmp(firstIP.c_str(), ip)==0)
	{
		hlp.SendControl(0, 0, 0);
		printf("main.cpp:  OnEndConnection(%s). Stopping robot motion. \n", ip);
		
	}
}

double stof(std::string num_st){
    double out = 0;
    bool fract_flag = 0;
	bool neg_flag = 0;
    int fract_count = 0;
    for (int i = 0; i < num_st.length(); i++){
        if(num_st[i] == '.'){
            fract_flag = 1;
        }
		else if(num_st[i] == '-'){
            neg_flag = 1;
        }
        else if(std::isdigit(num_st[i])){
            out = out*10+((int)num_st[i]-48);
            if(fract_flag){
                fract_count++;
            }
        }else{
            throw("invalid data format");
        }
    }
	if(neg_flag){
		out*=-1;
	}
    if(fract_count!=0){
        return (double)out/pow(10, fract_count);
    }else{
        return out;
    }
}



void parse_msg(std::string s){
	std::cout << "MSG: " << s << std::endl;
	float arm_pos[6];
	float base_speed[3];
	float grip_pos[2];
	float k=M_PI/180;
    std::string delimiter = ";";
    std::string arm_start_str = "/arm:";
    std::string grip_start_str = "/grip:";
    std::string base_start_str = "/base:";
    std::string arm_vel_start_str = "/arm_vel:";
    size_t pos = 0;
    int token;
    try{
        if(pos = s.find(arm_start_str) != std::string::npos){
            s.erase(0, arm_start_str.length());
        
            int i = 0;
            while ((pos = s.find(delimiter)) != std::string::npos) {
                arm_pos[i] = stof(s.substr(0, pos));
                i++;
                s.erase(0, pos + delimiter.length());
            }
            arm_pos[i] = stof(s);
            i++;
            if (i != 6){
                throw("wrong data length");
            }
            //std::cout << "send to manip: " << int(arm_pos[0]) << "; " << arm_pos[1]*k << "; " << arm_pos[2]*k << "; " << arm_pos[3]*k << "; " << arm_pos[4]*k << "; " << arm_pos[5]*k << "\n";
            hlp.SetManipulator(int(arm_pos[0]), arm_pos[1]*k, arm_pos[2]*k,arm_pos[3]*k, arm_pos[4]*k, arm_pos[5]*k);
        
        }else if(pos = s.find(arm_vel_start_str) != std::string::npos){
            s.erase(0, arm_vel_start_str.length());
        
            int i = 0;
            while ((pos = s.find(delimiter)) != std::string::npos) {
                arm_pos[i] = stof(s.substr(0, pos));
                i++;
                s.erase(0, pos + delimiter.length());
            }
            arm_pos[i] = stof(s);
            i++;
            if (i != 6){	
                throw("wrong data length");
            }
            hlp.SetManipulatorVel(int(arm_pos[0]), arm_pos[1]*k, arm_pos[2]*k,arm_pos[3]*k, arm_pos[4]*k, arm_pos[5]*k);
            
        }else if(pos = s.find(base_start_str) != std::string::npos){
            s.erase(0, base_start_str.length());
        
            int i = 0;
            while ((pos = s.find(delimiter)) != std::string::npos) {
                base_speed[i] = stof(s.substr(0, pos));
                i++;
                s.erase(0, pos + delimiter.length());
            }
            base_speed[i] = stof(s);
            i++;
            if (i != 3){
                throw("wrong data length");
            }
            hlp.SendControl(base_speed[0], base_speed[1], base_speed[2]);
        }else if(pos = s.find(grip_start_str) != std::string::npos){
            s.erase(0, grip_start_str.length());
            int i = 0;
            while ((pos = s.find(delimiter)) != std::string::npos) {
                grip_pos[i] = stof(s.substr(0, pos));
                i++;
                s.erase(0, pos + delimiter.length());
            }
            grip_pos[i] = stof(s);
            i++;
            if (i != 2){
                throw("wrong data length");
            }
            hlp.SetGripper(int(grip_pos[0]), grip_pos[1]);
        }
    }catch(...){
		printf("msg parse error");
    }
}
int main(int argc, char **argv)
{ 
    std::cout << "Youbot tl started (edited by BZ)" << std::endl;

	int fd =-1, bat_cnt=0;
	
	TcpServer ts;
	ts.SetCallbacks(&OnReceive, &OnEndConnection);
	ts.StartAsync();
	
	std::cout << "TCP Init OK" << std::endl;

    hlp.init(argc, argv);
	std::cout << "ROS Init OK" << std::endl;
	
	hlp.ts=&ts;	
	hlp.InitCallbacks();
	std::cout << "Callbacks Init OK" << std::endl;

    double hertz=80;

    ros::Rate loop_rate(hertz);
	
	int cnt=0;
		
    while(ros::ok())
    {
		if(received_TCP_MSG)
		{			
			received_TCP_MSG=false;
			parse_msg(TCP_MSG);
		}
		ros::spinOnce();
		loop_rate.sleep();
		
	if(cnt++<1) 
		std::cout << "First ROS iter OK" << std::endl;
    }
	
	printf("YTL: Finishing\n");
	hlp.ts->Stop();
	close(fd);
  return 0;
}


