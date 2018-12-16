/*!
 @header     UDPClient.c
 @abstract   UDP客户端
 @author     张万杰
 */
#include "UDPClient.h"

static char arRemoteIP[16] = "127.0.0.1";
static int nRemotePort;
static int s,len,addr_len;
static struct sockaddr_in addr;
static unsigned char arSendBuf[1024];
static int bInited = 0;

void InitUDPClient(char* inServerIP, int inPort)
{
	printf("[UDP_Client]InitUDPClient(IP = %s port = %d )...\n",inServerIP,inPort);
	memcpy(arRemoteIP,inServerIP,16);
	nRemotePort = inPort;

    addr_len =sizeof(struct sockaddr_in);
    /* 建立socket*/
    if((s = socket(AF_INET,SOCK_DGRAM,IPPROTO_UDP))<0){
        perror("socket");
        return;
    }
    /* 填写sockaddr_in*/
    bzero(&addr,sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(nRemotePort);
    addr.sin_addr.s_addr = inet_addr(arRemoteIP);

	bInited = 1;
}

void WordToBytes(int inVal,unsigned char* inDest)
{
    inDest[0] = (unsigned char )(inVal >> 8);
    inDest[1] = (unsigned char )(inVal);
}

void Int32ToBytes(int inVal,unsigned char* inDest)
{
    inDest[0] = (unsigned char )(inVal >> 24);
    inDest[1] = (unsigned char )(inVal >> 16);
    inDest[2] = (unsigned char )(inVal >> 8);
    inDest[3] = (unsigned char )(inVal);
}

void FloatToBytes(float inVal,unsigned char* inDest)
{
    memcpy(inDest,(unsigned char*)&inVal,sizeof(float));
}


void ReportResult(int inX,int inY,int inSum)
{
    arSendBuf[0] = 0x55;
    arSendBuf[1] = 0xaa;
    arSendBuf[2] = 13;      //len
    arSendBuf[3] = 0x00;    //flag
    //x
    WordToBytes(inX,&arSendBuf[4]);
    //y
    WordToBytes(inY,&arSendBuf[6]);
    //sum
    Int32ToBytes(inSum,&arSendBuf[8]);

    arSendBuf[arSendBuf[2]-1] = 0;
    int i=0;
    for(i=0;i<arSendBuf[2]-1;i++)
    {
        arSendBuf[arSendBuf[2]-1] += arSendBuf[i];
    }

    UDPClientSend(arSendBuf,arSendBuf[2]);
}

void UDPClientSend(unsigned char* inData,int inLen)
{
	if(bInited == 0)
	return;
    //printf("UDPClientSend( len = %d )...\n",inLen);
    /*printf("[UDP_Send] ");
    int i =0;
    for(i=0;i<inLen;i++)
    {
        printf("%.2X ",inData[i]);
    }
    printf("\n");*/
    /* 将字符串传送给server端*/
    sendto(s,inData,inLen,0,(struct sockaddr *)&addr,addr_len);
}

void SendRobotState(int inID, int inState, float inX, float inY, float inAngle)
{
    arSendBuf[0] = 0x55;
    arSendBuf[1] = 0xaa;
    arSendBuf[2] = inID;      
    arSendBuf[3] = inState;
    
    FloatToBytes(inX,&arSendBuf[4]);
    FloatToBytes(inY,&arSendBuf[8]);
    FloatToBytes(inAngle,&arSendBuf[12]);

    UDPClientSend(arSendBuf,16);
}
