#include "me3_ctrl.h"

//#define MODULE_TEST

ME3_ctrl::ME3_ctrl()
{
	const char* default_ip = "192.168.1.105";
	this->ip = strdup(default_ip);
    this->port = PORT;
}

// ME3_ctrl::ME3_ctrl(const char* me3_ip)
// {
//     this->ip = strdup(me3_ip);
//     this->port = PORT;
// 	this->connected = 0;

// }

ME3_ctrl::~ME3_ctrl()
{
    free(this->ip);
    
}

int ME3_ctrl::init_ME3_connect()
{
	struct sockaddr_in serv_addr;	
	if ((this->sock  = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n Socket creation error \n");
		return -1;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(this->port);
	
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, this->ip, &serv_addr.sin_addr)<=0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		printf("\nConnection Failed \n");
		return -1;
	}
	return 0;
    
}


// int ME3_ctrl::send_angle_to_ME3(arm_ctrl* arm){
//     int ret = 0;
// 	const char *hello = "+IPD : L_UP_ARM  15 20 00\n L_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)\n R_UP_ARM  25 20 00\n R_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)";
// 	send(this->sock , hello , strlen(hello) , 0 );
	

//     return ret;
// }

int ME3_ctrl::send_bytes_to_ME3(char* data){
    int ret = 0;
	//this->connected = 0;
	struct sockaddr_in serv_addr;	
	if ((this->sock  = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n Socket creation error \n");
		return -1;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(this->port);
	
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, this->ip, &serv_addr.sin_addr)<=0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		printf("\nConnection Failed \n");
		return -1;
	}
	//this->connected = 1;
	//const char *hello = "+IPD : L_UP_ARM  15 20 00\n L_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)\n R_UP_ARM  25 20 00\n R_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)";
	send(this->sock , data , strlen(data) , 0 );

	printf(data);
	

    return ret;
}

#ifdef MODULE_TEST
int main()
{
	const char* ip = "192.168.1.122";
	//arm_ctrl dummy_arm = arm_ctrl()
	char *hello = "+IPD : L_UP_ARM  15 20 00\n L_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)\n R_UP_ARM  25 20 00\n R_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)";
	
	ME3_ctrl robot_ctrl = ME3_ctrl();
	robot_ctrl.setIP(ip);
	robot_ctrl.send_bytes_to_ME3(hello);
	robot_ctrl.send_bytes_to_ME3(hello);
	robot_ctrl.send_bytes_to_ME3(hello);

    return 0;
}
#endif