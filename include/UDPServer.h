#ifndef _EXS_UDPSERVER_H
#define _EXS_UDPSERVER_H
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define bool char
#define true 1
#define false 0

#define CTRL_STOP           0x00
#define CTRL_MOVETO_POS     0x01  
#define CTRL_MOVETO_NAME    0x02
#define CTRL_GRAB           0x03
#define CTRL_PASS           0x04

typedef struct ST_Ctrl
{
    int ctrl;
    float x;
    float y;
    float angle;
    char wp_name;
}ST_Ctrl;

void InitUDPServer(int inPort);
int UDPServerLoop();
bool GetCtrlCmd(ST_Ctrl* inCtrl);
#endif
