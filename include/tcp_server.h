//by S. Diane (2014-12-XX, 2015-02-XX)

//todo создавать MSG динамически для кажд OnReceive и не использовать mutex-ы
//todo запрет на повторные соединения с того же самого компа
//todo отсоединение, удаление лишних сокетов

#define _POSIX_C_SOURCE 200809L

using namespace std;


struct MSG //string message
{
	int fd;
	char ip[20];
	char data[2048];
	double last_ttl;
};

struct Connection //connectio info
{
	int fd;
	std::string ip;
	double time_to_live;
};
const int TTL_SECONDS=7200;

//======================= CLASS DEFINITION =======================	
	
class TcpServer // server
{
	
double get_time_ns (time_t t0)
{
    struct timespec spec;
    clock_gettime(CLOCK_REALTIME, &spec);
	time_t s  = spec.tv_sec;
    double us = spec.tv_nsec / 1.0e9; // Convert nanoseconds to seconds
	long ds=(long)s-(long)t0;
	return ds+us;
}

	public:
	std::vector<Connection> connected;	//list of connected clients
	
	
	pthread_mutex_t mut;//for stopping the server
	
	void SetCallbacks(void (*OnReceive_)(MSG* msg), void (*OnEndConnection_)(const char* ip))
	{
		OnReceive=OnReceive_; 
		OnEndConnection=OnEndConnection_; 
	}
	
	Connection* GetConnectionByFD(int fd)
	{
		for (std::vector<Connection>::iterator it = connected.begin(); it != connected.end(); ++it) 
		{
			if ((*it).fd==fd) return &(*it);			
		}	
		return NULL;
	}
	void Send(std::vector<Connection>::iterator it, const char* msg)
	{
		//Connection* cn=FindConnectionByFd(fd);
		int fd = (*it).fd;
		int n = write(fd,msg,strlen(msg));
		write(fd, "\r\n", 2);
		if (n < 0) {
			perror("Send function: ERROR writing to socket");
			it->time_to_live=-777;//forcing TTL < 0
			//StartAsync();
			}		
	}
	// void SendFirst(const char* msg)
	// {
		// Send(connected[0].fd, msg);	
	// } 

	void SendAllLong(const char* msg)
	{
		for (std::vector<Connection>::iterator it = connected.begin(); it != connected.end(); ++it) 
		{
			Send(it, msg);
		}
	}

	void SendAll(const char* msg)
	{
		for (std::vector<Connection>::iterator it = connected.begin(); it != connected.end(); ++it) 
		{
			Send(it, msg);
		}
	}

	TcpServer()
	{
		need_close=false;
	}
	
	pthread_t thread1;
	void StartAsync()
	{
		need_close=false;
		pthread_create (&thread1, NULL, &TcpServer::Run_pthreads, this);
		pthread_mutex_unlock(&mut); //2018-03-02
	}
	
	void Stop()
	{
		pthread_mutex_lock(&mut);
		need_close=true;
		pthread_mutex_unlock(&mut);
		
		pthread_join(thread1, NULL);		
	}
		 
	int NumConnections()
	{
		return connected.size();
	}
	
	private:
	bool need_close;
	
	void (*OnReceive)(MSG* msg);
	void (*OnEndConnection)(const char* ip);
	
	void StopThread(const char* msg)
	{
		perror(msg);
		pthread_exit(NULL);
	}
	
	void print_connections(int server, fd_set* p_fds)
	{
		std::stringstream ss;
		ss << "connections: ";
		for (std::vector<Connection>::iterator it = connected.begin(); it != connected.end(); ++it) 
		{
			if (FD_ISSET(server, p_fds)) {
				ss<<"fd"<<(*it).fd<<"="<<(*it).ip<<"; ";
			}
		}	
		ss<<std::endl;
		std::cout<<ss.str();
	}
	
	
	void str_replace( string &s, const string &search, const string &replace ) 
  {
      for( size_t pos = 0; ; pos += replace.length() ) 
  	{
          pos = s.find( search, pos );
         if( pos == string::npos ) break;
 
         s.erase( pos, search.length() );
         s.insert( pos, replace );
     }
 }

	
	void Run()
	{
		signal(SIGPIPE, SIG_IGN);
		fd_set fds;
		int max = 0, reuse = 1;
		struct timeval tv;
		int server;
		
		struct sockaddr_in serv_addr, cli_addr;
		socklen_t cli_len = sizeof(cli_addr);
		bzero((char *) &serv_addr, sizeof(serv_addr)); 
		int portno = 7777;
		serv_addr.sin_family = AF_INET;
		serv_addr.sin_addr.s_addr = INADDR_ANY;
		serv_addr.sin_port = htons(portno);		
		
		// create server listening socket
		server = socket(PF_INET, SOCK_STREAM, getprotobyname("tcp")->p_proto);
		setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(int)); // optional, but recommended
		if (bind(server, (struct sockaddr *)&serv_addr, sizeof(struct sockaddr_in)) < 0) {
			StopThread("Error, could not bind server socket");
		}
		
		if (listen(server, 8) < 0) {
			StopThread("Error, could not listen on server port");
		}
		
		printf("Waiting for connections on port %d\n", portno);
		
		MSG msg;
		
		time_t t0=std::time(NULL);
		
		double curr_t, last_t=get_time_ns(t0);
		
		// loop looking for connections / data to handle
		while (!need_close) 
		{	
			FD_ZERO(&fds);
			FD_SET(server, &fds);
			if (server >= max) max = server + 1;
			
			for (std::vector<Connection>::iterator it = connected.begin(); it != connected.end(); ++it) {
				FD_SET((*it).fd, &fds);
				if ((*it).fd >= max) max = (*it).fd + 1;
			}
			
			//checking incoming connections
			tv.tv_sec = 2; tv.tv_usec = 0;
			if (select(max, &fds, NULL, NULL, &tv) > 0) {
				// something is readable
				if (FD_ISSET(server, &fds)) {
					// it's the listener
					int fd=accept(server, (struct sockaddr *)&cli_addr, &cli_len);
					
					char addr_str[20];
					inet_ntop(AF_INET, &(cli_addr.sin_addr), addr_str, sizeof(addr_str));
					printf("Connection accepted from %s\n", addr_str);
					
					Connection cn;
					cn.fd=fd;
					cn.ip=std::string(addr_str);
					cn.time_to_live=TTL_SECONDS; 
					
					connected.push_back(cn);
					
					print_connections(server, &fds);							
				}				
				
				//checking incoming messages
				for (std::vector<Connection>::iterator it = connected.begin(); it != connected.end(); ++it) 
				{
					if (!FD_ISSET((*it).fd, &fds)) continue;
					
						(*it).time_to_live=TTL_SECONDS;
						
						std::strcpy(msg.ip,(*it).ip.c_str());
						
						int pos=-1, pos_max=sizeof(msg.data);
						int writepos=0;
						while (pos++<pos_max)
						{						
							char ch;
							int n = read((*it).fd,&ch,1);
							if (n < 0){
								(*it).time_to_live=-777;
								break;
							}
							msg.data[writepos]=ch;
							
							msg.data[writepos+1]='\0';
							
							if(writepos>=2 && !strcmp(msg.data + (writepos-2), "^^^")) //if equals
							{
								msg.data[writepos-2]='\0';
								msg.data[writepos-1]='\0';
								msg.data[writepos]='\0';
								
								msg.fd=(*it).fd;
								
								if(writepos>2) break; //non-empty string
								else{
									(*it).time_to_live=-777;
									printf("Empty string received from client\n");
									break;
								}

							}
							writepos++;
						}
						
						 msg.last_ttl=(*it).time_to_live;
							
						
					 
						if(!strcmp(msg.data, "#end#")) //if equals
						{
							printf("#end#\n");
							(*it).time_to_live=-777;//forcing TTL < 0
							
							continue;
						}
						if(!strcmp(msg.data, "#start#")) //if equals
						{
							printf("#start#\n");
							continue;
						}
						if(!strcmp(msg.data, "#ok#")) //if equals
						{
							printf("#ok#\n");
							continue;
						}
						if(!strcmp(msg.data, "#time_check#")) //if equals
						{
							//printf("#time_check#\n");
							Send(it, "#time_check#");
							continue;
						}
						
						pthread_mutex_lock(&mut);
						OnReceive(&msg);
						pthread_mutex_unlock(&mut);
						//int n = read((*it).fd, msg.data, sizeof(msg.data));
						//if (n < 0) StopThread("Error reading from socket");
						
						pthread_mutex_lock(&mut);
						OnReceive(&msg); //printf("Here is the message: %s\n",buffer);
						pthread_mutex_unlock(&mut);
						
						//close((*it).fd);
						
						
						//printf("END MSG.\n");
				}			
			}
			
			//removing inactive connections
				last_t=curr_t; curr_t=get_time_ns(t0); double dt=curr_t-last_t;
				for (std::vector<Connection>::iterator it = connected.begin(); it != connected.end();) {	
					double* t=&((*it).time_to_live);
					(*t)-=dt;
					if((*t)<=0) 
					{
						int fd=(*it).fd;
						const char* ip=(*it).ip.c_str();
						close(fd);
						connected.erase(it);
						if((*t)<-777*0.99) printf("Connection %d closed (client disconnected)\n", fd);
						else printf("Connection %d closed (zero TTL) ttl=%f\n", fd, *t);
						
						pthread_mutex_lock(&mut);
						OnEndConnection(ip);
						pthread_mutex_unlock(&mut);						
					}
					else 
					{
						++it;
					}
					
				}
			usleep(20*1000);
		}
		close(server);
		printf("Finishing TCP server thread");
	}
	static void* Run_pthreads(void* context)
	{
		((TcpServer *)context)->Run();
		return NULL;
	}
};		
