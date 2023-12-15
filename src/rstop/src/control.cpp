#include <iostream>
#include <vector>

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class server{
public:

	server(){}

	server(int port, int packet_type){

		// packet_type = 1: tcp packets
		// packet_type = 2: udp packets
		
		this->socket_family = AF_INET;
		this->use_internet_protocol = 0;
		this->buffer = new char[1024];

		this->server_addr_len = sizeof(this->server_address);
		this->client_addr_len = sizeof(this->client_address);

		this->socket_opt = 1;

		this->server_fd = socket(AF_INET, packet_type, 0);
		if (server_fd < 0){
			printf("failed to create a socket\n");
			exit(EXIT_FAILURE);
		}

		if (setsockopt(
				this->server_fd,
				SOL_SOCKET,
				SO_REUSEADDR|SO_REUSEPORT,
				&this->socket_opt,
				sizeof(this->socket_opt))){
			printf("failed to force socket to the assigned port\n");
			exit(EXIT_FAILURE);
		}

		this->server_address.sin_family = this->socket_family;
		this->server_address.sin_addr.s_addr = INADDR_ANY;
		this->server_address.sin_port = htons(port);

		if (bind(
				this->server_fd,
				(struct sockaddr *)&this->server_address,
				this->server_addr_len) < 0){
			printf("unable to bind\n");
			exit(EXIT_FAILURE);
		}

		printf("successfully binded to port %d\n", port);
	}

	bool listener(){

		int captured_voice = listen(this->server_fd, 3);
		if (captured_voice < 0){
			printf("error while listening");
			exit(EXIT_FAILURE);
		}
		printf("listening...");

		this->client_socket = accept(
				this->server_fd, 
				&this->client_address, 
				&this->client_addr_len);

		if (client_socket < 0){
			printf("error accepting a client");
			exit(EXIT_FAILURE);
		}

		this->read_len = read(
				this->client_socket, 
				this->buffer, 
				1024-1); // last character = EOL = '\0'

		printf("[client]: %s\n", this->buffer);
		return true;
	}

private:

	int socket_family; // ipv4 or ipv6
	int use_internet_protocol; // = 0

	int server_fd; // server socket file descriptor
	int client_socket; // client socket file descriptor

	int socket_opt; // set-socket-opt option

	struct sockaddr_in server_address; // server address holder;
	struct sockaddr client_address; // client address holder;

	socklen_t server_addr_len; // size of server address socket holder
	socklen_t client_addr_len; // size of client address socket holder

	char* buffer; // client message buffer 
	ssize_t read_len; // holds the size of the received message

};

class control{
public:

	control(int argc, char **argv){

		ros::init(argc, argv, "control");
		ros::NodeHandle node;

		if (argc < 3){
			printf("arguments: port packet_type\n");
			exit(EXIT_FAILURE);
		}

		char *port_ = argv[1];
		char *packet_type_ = argv[2];

		int port = std::stoi(port_);
		int packet_type = std::stoi(packet_type_);

		if (packet_type != 1 && packet_type != 2){
			printf("packet_type = 1: tcp\n");
			printf("packet_type = 2: tcp\n");
			exit(EXIT_FAILURE);
		}

		this->stop_sender = server(port, packet_type);

		this->stop_triggered = false;
		this->zero_vel.linear.x = 0;
		this->zero_vel.angular.z = 0;

		this->cmd_vel_sub = node.subscribe("/cmd_vel", 1, &control::get_cmd_vel, this);
		this->safe_vel_pub = node.advertise<geometry_msgs::Twist>("/safe_vel", 1, true);
	}

	void main(){
		
		ros::AsyncSpinner spinner(10);
		spinner.start();

		this->stop_triggered = this->stop_sender.listener();
		std::cout << "STOP TRIGGERED" << std::endl;

		ros::Rate rate(30);
		while (ros::ok()){
			rate.sleep();
		}

	}

private:

	void get_cmd_vel(const geometry_msgs::Twist::ConstPtr &cmd_vel){

		if (this->stop_triggered){
			this->safe_vel_pub.publish(this->zero_vel);
			return ;
		}

		geometry_msgs::Twist safe_vel;
		
		safe_vel.linear.x = std::min((float)cmd_vel->linear.x, this->max_linear_vel);
		safe_vel.linear.x = std::max((float)safe_vel.linear.x, this->min_linear_vel);

		safe_vel.angular.z = std::min((float)cmd_vel->angular.z, this->max_angular_vel);
		safe_vel.angular.z = std::max((float)safe_vel.angular.z, this->min_angular_vel);

		this->safe_vel_pub.publish(safe_vel);
	}

	server stop_sender;
	
	ros::Subscriber cmd_vel_sub;
	ros::Publisher safe_vel_pub;

	const float min_linear_vel = -0.3;
	const float max_linear_vel = 0.7;

	const float min_angular_vel = -0.5;
	const float max_angular_vel = 0.5;

	geometry_msgs::Twist zero_vel;
	bool stop_triggered;

};

int main(int argc, char **argv){

	control c(argc, argv);
	c.main();

	return 0;
}

