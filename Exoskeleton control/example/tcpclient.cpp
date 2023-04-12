// Client side C/C++ program to demonstrate Socket programming
#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#define PORT 	1234



//	ME3 robot standby default angles
// +IPD : L_UP_ARM  25 20 00
//  L_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)
//  R_UP_ARM  25 20 00
//  R_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)

int main(int argc, char const *argv[])
{
	int sock = 0, valread;
	const char ME3_IP[] = {"192.168.21.118"};
	struct sockaddr_in serv_addr;
	char *hello = "+IPD : L_UP_ARM  15 20 00\n L_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)\n R_UP_ARM  25 20 00\n R_FOREARM 00 00 00 (thumb: 0x700 fore: 0x700 middle: 0x700 ring: 0x700 pinky: 0x700)";
	char buffer[1024] = {0};
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		printf("\n Socket creation error \n");
		return -1;
	}

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);
	
	// Convert IPv4 and IPv6 addresses from text to binary form
	if(inet_pton(AF_INET, ME3_IP, &serv_addr.sin_addr)<=0)
	{
		printf("\nInvalid address/ Address not supported \n");
		return -1;
	}

	if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
	{
		printf("\nConnection Failed \n");
		return -1;
	}
	send(sock , hello , strlen(hello) , 0 );
	printf("Hello message sent\n");
	valread = read( sock , buffer, 1024);
	printf("%s\n",buffer );
	return 0;
}
