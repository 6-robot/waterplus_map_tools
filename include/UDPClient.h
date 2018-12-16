#ifndef _EXS_UDPCLIENT_H
#define _EXS_UDPCLIENT_H
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
void InitUDPClient(char* inServerIP, int inPort);
void UDPClientSend(unsigned char* inData,int inLen);
void SendRobotState(int inID, int inState, float inX, float inY, float inAngle);
#endif
