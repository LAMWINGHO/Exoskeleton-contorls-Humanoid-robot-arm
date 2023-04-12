#ifndef ME3_CTRL_H
#define ME3_CTRL_H_

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>
#include <cstdlib>

//#define PORT 	25022
#define PORT 	1234



class ME3_ctrl
{
private:
    char* ip;
    int port;
    int sock;

    char send_buffer[1024];
public:
    ME3_ctrl();
    ~ME3_ctrl();
    int init_ME3_connect();
    int send_bytes_to_ME3(char* data);
    void setIP(const char* set_ip) {
      strcpy(ip,set_ip);
    }
    

};


#endif