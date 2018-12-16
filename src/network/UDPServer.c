/*!
 @header     UDPServer.c
 @abstract   UDP通讯服务
 @author     张万杰
 */
#include "UDPServer.h"
#include "UDPClient.h"

static char buffer[1024];
static int sockfd,len;
static struct sockaddr_in addr;
static int addr_len;
static pthread_t hThread;

static int bFrameStart = 0;
static int nParseIndex = 0;
static int nLenToParse = 10;
static unsigned char m_byteLast = 0;
static unsigned char arParseBuf[1024];

static bool bNewCmd = false;
static ST_Ctrl ctrl_recv;

int BytesToWord(unsigned char* inBuf)
{
    int res = 0;
    res = inBuf[0];
    res <<= 8;
    res |= inBuf[1];
    return res;
}

float BytesToFloat(unsigned char* inBuf)
{
    float res = 0;
    memcpy((char*)&res,inBuf,sizeof(float));
    return res;
}

void UDP_Server_ParseFrame(unsigned char* inBuf,int inLen)
{
    printf("%d [UDP_Recv] ",inLen);
    int i =0;
    for(i=0;i<inLen;i++)
    {
        printf("%.2X ",inBuf[i]);
    }
    printf("\n");

    ctrl_recv.ctrl = inBuf[2];
    if(ctrl_recv.ctrl == CTRL_MOVETO_POS)
    {
        ctrl_recv.x = BytesToFloat(&(inBuf[3]));
        ctrl_recv.y = BytesToFloat(&(inBuf[7]));
        ctrl_recv.angle = BytesToFloat(&(inBuf[11]));
        printf("[CTRL_MOVETO_POS] ( %.2f , %.2f ) - %.2f\n",ctrl_recv.x,ctrl_recv.y,ctrl_recv.angle);
    }
    if(ctrl_recv.ctrl == CTRL_MOVETO_NAME)
    {
        ctrl_recv.wp_name = inBuf[3];
        printf("[CTRL_MOVETO_NAME]  - \'%c\'\n",ctrl_recv.wp_name);
    }

    bNewCmd = true;
   
}

void UDP_Server_Parse(unsigned char inChar)
{
    if(bFrameStart == 0)
    {
        if(m_byteLast == 0x55 && inChar == 0xaa)
        {
            bFrameStart = 1;
            arParseBuf[0] = 0x55;
            arParseBuf[1] = 0xaa;
            nParseIndex = 2;
        }
        else
        {
            m_byteLast = inChar;
        }
    }
    else
    {
        //开始缓存
        if(nParseIndex == 2)
        {
            if(inChar == CTRL_STOP)
            {
                nLenToParse = 3;
            }
            if(inChar == CTRL_MOVETO_POS)
            {
                nLenToParse = 15;
            }
            if(inChar == CTRL_MOVETO_NAME)
            {
                nLenToParse = 4;
            }
        }
        arParseBuf[nParseIndex] = inChar;
        nParseIndex ++;

        if(nParseIndex >= nLenToParse)
        {
            UDP_Server_ParseFrame((unsigned char*)arParseBuf,nLenToParse);
            bFrameStart = 0;
            nParseIndex = 0;
            m_byteLast = 0;
        }
    }
}

void *threadUDPServer_func()
{
    printf ("[UDP_Server]threadUDPServer_func start...\n");
    while(1)
	{
        bzero(buffer,sizeof(buffer));
        len = recvfrom(sockfd,buffer,sizeof(buffer), 0 , (struct sockaddr *)&addr ,&addr_len);
        /*显示client端的网络地址*/
        printf("receive %d from %s\n" , len , inet_ntoa( addr.sin_addr));

        int i = 0;
        for(i=0;i<len;i++)
        {
            UDP_Server_Parse(buffer[i]);
        }

        /*初始化udp_client端**/
        //InitUDPClient(inet_ntoa( addr.sin_addr),10010);

   }
    printf("[UDP_Server]threadUDPServer_func exit\n");
    pthread_exit(NULL);
}

void InitUDPServer(int inPort)
{
	printf("[UDP_Server]InitUDPServer( port = %d )...\n",inPort);

    addr_len = sizeof(struct sockaddr_in);
    /*建立socket*/
    if((sockfd=socket(AF_INET,SOCK_DGRAM,0))<0)
    {
        perror ("socket");
        return;
    }
    /*填写sockaddr_in 结构*/
    bzero ( &addr, sizeof(addr) );
    addr.sin_family=AF_INET;
    addr.sin_port=htons(inPort);
    addr.sin_addr.s_addr=htonl(INADDR_ANY) ;
    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr))<0)
    {
        perror("connect");
        return;
    }

    int res;
    res = pthread_create(&hThread, NULL, threadUDPServer_func, NULL);
}

int UDPServerLoop()
{
    unsigned char ip[10] = "hello";

    while(1)
    {
        bzero(buffer,sizeof(buffer));
        len = recvfrom(sockfd,buffer,sizeof(buffer), 0 , (struct sockaddr *)&addr ,&addr_len);
        /*显示client端的网络地址*/
        printf("receive %d from %s\n" , len , inet_ntoa( addr.sin_addr));
        /*将字串返回给client端**/
        //InitUDPClient(inet_ntoa( addr.sin_addr),20180);
        //UDPClientSend(ip,10);

        usleep(1000*10);
   }
}


bool GetCtrlCmd(ST_Ctrl* inCtrl)
{

    if(bNewCmd == true)
    {
        memcpy(inCtrl,&ctrl_recv,sizeof(ctrl_recv));
        bNewCmd = false;
        return true;
    }
    else
    {
        return false;
    }
}
