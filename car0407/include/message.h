/*
	Define the data structure of messages flowing on the network
*/
#include "para.h"
#ifndef MESSAGE_H__
#define MESSAGE_H__
#include "DSP2833x_Device.h"

// Msg: message sent through wireless network
#if defined(XBEE)
typedef struct {	


	Uint32 destH;	//目标地址4个高位字节
	Uint32 destL;	//目标地址4个低位字节
	Uint32 srcH;	//
	Uint32 srcL;
	Uint16 len;		// 载荷长度
	unsigned char frame[90];	// payload

} Msg;
#else
typedef struct {
	unsigned char len;		// 载荷长度
	unsigned char frame[60];	// payload
	Uint16	dest;			// 目标地址
	Uint16	src;
} Msg;
#endif

// Cmd: command. can be the payload of a message
#if defined(XBEE)
typedef struct{
    unsigned char code;     // 指令码
    int16   para[9];        // 参数
    Uint32  destH;          // 目标地址高位4Bytes
    Uint32  destL;          // 目标地址低位4Bytes
    Uint32  srcH;           // 源地址高位4Bytes
    Uint32  srcL;           // 源地址低位4Bytes
    Uint32  len;            // 参数个数
} Cmd;
#else
typedef	struct{	
	unsigned char code;		// 指令码
	int16	para[9];		// 参数
	Uint16	dest;			// 目标地址
	Uint16	src;			// 源地址
	unsigned char len;		// 参数个数
} Cmd;
#endif

/*
	Set the DESTINATION/SOURCE of a message;
	Reset a message;
*/
#ifdef XBEE
void	MsgSetDest(Msg* msg, const Uint32* addr_server);
#else
void	MsgSetDest(Msg* msg, const Uint16* addr_server);
#endif
void	MsgSetSrc(Msg* msg);
void	MsgReset(Msg* msg);

/*
	Transformation between MSG / CMD
*/
void	GetCmd(const Msg* msg, Cmd* cmd);
void	Cmd2Msg(const Cmd* cmd, Msg* msg);

/*	
	Calculate the XORSUM of a given string
*/
unsigned char XorSum(const unsigned char* str, unsigned char len);//异或相加
unsigned char XORCheck(const unsigned char* str, unsigned char len);//异或相加结果检查

/*
	Get TYPE of a command
*/
unsigned char GetCmdType(Cmd* cmd);
void MsgAppendAddFrame(Msg *msg);
/*
	Find if message is empty
*/
unsigned char IsMsgEmpty(Msg* msg);
void MsgAppendFrame(Msg *msg,const Uint32* addr_server);
#endif
